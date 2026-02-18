// SPDX-License-Identifier: GPL-3.0-or-later
mod render;
use crate::common::is_on_lane;
pub use crate::render::render_run;

use json::JsonValue;
use num_complex::Complex;
use ordered_float::OrderedFloat;
use std::cell::RefCell;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::fs;
use std::rc::Rc;

mod common;
pub mod drivers;
pub mod setting;

use common::{Action, Shared, THETA_ALLOW, unwrap_theta};
pub use drivers::{Driver, DriverFactory};
pub use setting::RoundaboutSimSetting;
use setting::SwitchPolicy;

const DIST_ALLOW: f32 = 1e-2;
const MIN_UPDATE_TICK: f32 = 1e-2;

#[derive(Debug)]
pub struct Car {
    pub id: usize,
    pos: Complex<f32>, // to tacke polar
    vel: f32,
    lane: usize,       // 0 is the outermost
    dst: Complex<f32>, // destination polar
    action: Action,
}

impl Car {
    fn finished(&self) -> bool {
        self.lane == 0 && (self.dst - self.pos).norm() <= DIST_ALLOW
    }
    /**
        called when action is granted
    */
    fn set_action(&mut self, action: Action) {
        self.action = action;
    }
    /**
        update according to verified action
    */
    fn update(&mut self, tick: f32, setting: &RoundaboutSimSetting) {
        match self.action {
            Action::Switch(ref diff_lane) => {
                let next_r = self.pos.norm() + ((-diff_lane as f32) * self.vel * tick);
                let target_r = setting.r_lanes[((self.lane as i32) + diff_lane) as usize];
                if *diff_lane < 0 && next_r >= target_r || /* switch out */
                   *diff_lane > 0 && next_r <= target_r
                /* switch in */
                {
                    self.pos = Complex::from_polar(target_r, self.pos.arg());
                    self.lane = ((self.lane as i32) + diff_lane) as usize;
                } else {
                    self.pos = Complex::from_polar(next_r, self.pos.arg());
                }
            }
            Action::Straight => {
                let mv = Complex::from_polar(1.0, (self.vel * tick) / setting.r_lanes[self.lane]);
                let next_pos = self.pos * mv;
                if (next_pos / self.dst).arg() > 0.0 && (self.pos / self.dst).arg() < 0.0 {
                    // cross the dst
                    self.pos = Complex::from_polar(setting.r_lanes[self.lane], self.dst.arg());
                } else {
                    self.pos = next_pos;
                }
            }
            Action::Stop => {}
        }
    }
}

pub struct RoundaboutSim {
    pub t: f32, // current time,
    pub setting: RoundaboutSimSetting,
    pub finished_cars: Vec<Shared<Car>>,
    cars: Vec<Shared<Car>>,
    drivers: Vec<Box<dyn Driver>>,
}

impl RoundaboutSim {
    pub fn new(setting: RoundaboutSimSetting, jobj: &JsonValue) -> Option<RoundaboutSim> {
        if !jobj.has_key("init") {
            return None;
        }
        let jinit = &jobj["init"];
        let mut cars = vec![];
        let mut drivers = vec![];
        for (key, value) in jinit.entries() {
            let lane = value["lane"].as_usize()?;
            let r = *setting.r_lanes.get(lane)?;
            let theta = value["theta"].as_f32()?;
            assert!(lane < setting.r_lanes.len());
            cars.push(Rc::new(RefCell::new(Car {
                id: key.parse().ok()?,
                pos: Complex::from_polar(r, theta),
                vel: value["vel"].as_f32()?,
                lane,
                dst: Complex::from_polar(
                    setting.r_lanes[0],
                    2.0 * PI / (setting.n_inter as f32) * value["dst"].as_f32()?,
                ),
                action: Action::Straight,
            })));
            // TODO: may be provided from setting
            drivers.push(DriverFactory::make_default_driver_boxed());
        }
        for (i, driver) in drivers.iter_mut().enumerate() {
            driver.init(&cars[i].borrow(), &setting);
        }
        Some(RoundaboutSim {
            t: 0.0,
            setting,
            cars,
            drivers,
            finished_cars: vec![],
        })
    }
    fn from_json(filename: &str) -> Option<RoundaboutSim> {
        let contents = fs::read_to_string(filename).expect("File not found");
        let jobj = json::parse(&contents).expect("file format error");
        let settings = RoundaboutSimSetting::new(&jobj).expect("some required key not specified");
        let sim = RoundaboutSim::new(settings, &jobj).expect("init cars format error");
        Some(sim)
    }
    /**
     * return a bool indicating finished
     */
    pub fn update(&mut self) -> bool {
        let setting = &self.setting;
        let mut by_lane = HashMap::<usize, Vec<Shared<Car>>>::new();
        // TODO: Now is O(n lgn)
        self.cars
            .sort_by_key(|car| OrderedFloat(car.borrow().pos.arg()));
        for car in &self.cars {
            let same_lane = by_lane.entry(car.borrow().lane).or_insert(vec![]);
            same_lane.push(car.clone());
        }
        let mut tick = setting.tick;
        // every car determines its action
        for (i, car) in self.cars.iter().enumerate() {
            let car_ref = &mut car.borrow_mut();
            let action = { self.drivers[i].drive(car_ref, self.t, setting) };
            car_ref.set_action(action);
        }
        // Staight action while a car is switching is not allowed
        for car in self.cars.iter_mut() {
            let car_ref = &mut car.borrow_mut();
            if let Action::Straight = car_ref.action
                && !is_on_lane(&car_ref.pos, setting.r_lanes[car_ref.lane])
            {
                car_ref.set_action(Action::Stop);
                println!(
                    "Car {}'s action force reset to stop due to straight while switching policy, this is a software bug.",
                    car_ref.id
                );
                return true;
            }
        }
        // detect straight collision, happens to the same lane
        let possible_straight_collision = |car_follow: &mut Car, car_precede: &Car| {
            let time_to_collide = self.straight_collision(car_follow, car_precede);
            if time_to_collide <= MIN_UPDATE_TICK {
                car_follow.set_action(Action::Stop);
                // println!("Car {} makes Car {} stop", car_follow.id, car_precede.id);
                f32::MAX
            } else {
                time_to_collide
            }
        };
        for same_lane in by_lane.values() {
            for (i, car_follow) in same_lane.iter().enumerate() {
                if let Some(car_precede) = same_lane.get((i + 1) % same_lane.len())
                    && same_lane.len() > 1
                {
                    let this_tick = possible_straight_collision(
                        &mut car_follow.borrow_mut(),
                        &car_precede.borrow(),
                    );
                    if this_tick < tick {
                        tick = this_tick;
                        // println!("Car {} and Car {} restrict update time to {}", car_follow.borrow().id, car_precede.borrow().id, this_tick)
                    }
                }
            }
        }
        // detect switch collision
        let possbile_switch_collision = |switching_car: &mut Car, car_follow: &mut Car| {
            if self.switch_collision(switching_car, car_follow, tick) {
                match setting.switch_policy {
                    SwitchPolicy::StraightFirst => {
                        switching_car.set_action(Action::Stop);
                    }
                    _ => {
                        car_follow.set_action(Action::Stop);
                    }
                }
            }
        };
        for (lane, same_lane) in &by_lane {
            for car in same_lane {
                let mut switching_car = car.borrow_mut();
                let switching_theta = switching_car.pos.arg();
                if let Action::Switch(diff_lane) = switching_car.action {
                    let next_lane = (*lane as i32) + diff_lane;
                    if let Some(other_lane) = by_lane.get(&(next_lane as usize)) {
                        // detect from lower bound on other lane
                        let idx = match other_lane.binary_search_by(|probe| {
                            OrderedFloat(probe.borrow().pos.arg())
                                .cmp(&OrderedFloat(switching_theta))
                        }) {
                            Ok(i) => i,
                            Err(i) => i,
                        };
                        if idx > 0 {
                            let idx = idx - 1;
                            let mut car_follow = other_lane[idx].borrow_mut();
                            possbile_switch_collision(&mut switching_car, &mut car_follow);
                        }
                        // detect from max on other lane
                        if let Some(car_follow) = other_lane.last() {
                            let mut car_follow = car_follow.borrow_mut();
                            possbile_switch_collision(&mut switching_car, &mut car_follow);
                        }
                    }
                }
            }
        }
        // detect side collision
        // returns true if car_other falls in the neighborhood of car_center
        let in_side_coliision_range = |car_center: &Car, car_other: &Car, delta: f32| -> bool {
            let diff = (car_center.pos / car_other.pos).arg();
            diff.abs() < delta
        };
        let possible_side_collision = |car_center: &Car, car_other: &Car, tick: f32| -> f32 {
            if let Action::Switch(car_diff) = car_center.action
                && let Action::Switch(ref other_diff) = car_other.action
            {
                let car_r = car_center.pos.norm();
                let other_r = car_other.pos.norm();
                // both switch in/out, return time to collide if relative position correct
                return if car_diff == *other_diff && (car_diff as f32) * (car_r - other_r) > 0.0 {
                    (car_r - other_r).abs() / car_center.vel
                } else {
                    tick
                };
            }
            tick
        };
        let detect_side_collision_routine =
            |car_center: &mut Car, car_other: &Car, tick: f32| -> f32 {
                let this_tick = possible_side_collision(car_center, car_other, tick);
                if this_tick < MIN_UPDATE_TICK {
                    car_center.set_action(Action::Stop);
                    // println!("Car {} stops Car {} ", car_other.id, car_center.id);
                    tick
                } else if this_tick < tick {
                    // println!("Car {} and Car {} restrict update time to {}", car_center.id, car_other.id, this_tick);
                    this_tick
                } else {
                    tick
                }
            };
        for same_lane in by_lane.values() {
            let n = same_lane.len();
            for (i, car) in same_lane.iter().enumerate() {
                let car_center = &mut car.borrow_mut();
                if let Action::Switch(_diff) = car_center.action {
                    // search through left
                    let mut left = (i + (n - 1)) % n;
                    while left != i {
                        let car_other = &same_lane[left].borrow();
                        if !in_side_coliision_range(car_center, car_other, THETA_ALLOW / 2.0) {
                            break;
                        }
                        tick = detect_side_collision_routine(car_center, car_other, tick);
                        left = (left + (n - 1)) % n;
                    }
                    // search through right
                    let mut right = (i + 1) % n;
                    while right != i && right != left {
                        let car_other = &same_lane[right].borrow();
                        if !in_side_coliision_range(car_center, car_other, THETA_ALLOW / 2.0) {
                            break;
                        }
                        tick = detect_side_collision_routine(car_center, car_other, tick);
                        right = (right + 1) % n;
                    }
                }
            }
        }
        self.t += tick;
        let mut has_progress = false;
        // TODO: Another chance for changing their actions?
        // update phase
        let mut next_cars = vec![];
        let n_cars = self.finished_cars.len() + self.cars.len();
        for (i, car) in self.cars.iter().enumerate() {
            {
                let car_ref = &mut car.borrow_mut();
                car_ref.update(tick, setting);
                self.drivers[i].update(car_ref, self.t, setting);
            }
            match car.borrow().action {
                Action::Stop => {}
                _ => {
                    has_progress = true;
                }
            };
            if car.borrow().finished() {
                self.finished_cars.push(car.clone());
                has_progress = true;
                println!(
                    "Car {} finishes at time {}, ({} / {n_cars})",
                    car.borrow().id,
                    self.t,
                    self.finished_cars.len()
                );
            } else {
                next_cars.push(car.clone());
            }
        }
        self.cars = next_cars;
        let all_finished = self.cars.is_empty();
        assert!(
            has_progress || all_finished,
            "everyone stops but not finished"
        );
        if all_finished {
            println!("===== simulation finished in: {} =====", self.t);
        } else {
            /*
            println!("===== t: {} (+{}) =====", self.t, tick);
            for car in &self.cars {
                println!("{:?}", car.borrow());
            }
            */
        }
        all_finished
    }
    /**
        Collision if @car_switch is switch in/out to the @car_other.lane and
        @car_other.polar.theta is in the arc occupied by @car_other with time @tick
    */
    fn switch_collision(&self, car_switch: &Car, car_other: &Car, tick: f32) -> bool {
        if let Action::Straight = car_other.action {
            let setting = &self.setting;
            match car_switch.action {
                Action::Switch(diff_lane) => {
                    let lane = (car_switch.lane as i32 + diff_lane) as usize;
                    if lane != car_other.lane {
                        return false;
                    }
                    let r_lane = setting.r_lanes[lane];
                    let switch_target_pos = Complex::from_polar(r_lane, car_switch.pos.arg());
                    let other_curr_pos = Complex::from_polar(r_lane, car_other.pos.arg());
                    let other_target_pos = Complex::from_polar(
                        r_lane,
                        car_other.pos.arg() + car_other.vel * tick / r_lane,
                    );
                    // @car_switch is in the arc of @car_other
                    let other_curr_2_swtich_target = switch_target_pos / other_curr_pos;
                    let switch_target_2_other_target = other_target_pos / switch_target_pos;
                    other_curr_2_swtich_target.arg() >= 0.0
                        && switch_target_2_other_target.arg() <= 0.0
                }
                _ => false,
            }
        } else {
            false
        }
    }
    /**
        returns the time for @car_follow to collide with @car_precede
        assuming @car_precede stays still, and @car_follow take straight action
        Check return value <= 0 as a signal to update @car_follow or not
    */
    fn straight_collision(&self, car_follow: &Car, car_precede: &Car) -> f32 {
        match car_follow.action {
            Action::Straight => {
                assert_eq!(
                    car_follow.lane, car_precede.lane,
                    "on the same lane but straight-straight collision called",
                );
                assert_ne!(car_follow.id, car_precede.id, "have the same id");
                let margin_theta = unwrap_theta((car_precede.pos.fdiv(car_follow.pos)).arg());
                let lane = car_follow.lane;
                margin_theta * self.setting.r_lanes[lane] / car_follow.vel
            }
            _ => f32::MAX,
        }
    }
}

pub fn sim_run(filename: &str, max_t: f32) -> Option<RoundaboutSim> {
    let mut sim = RoundaboutSim::from_json(filename)?;
    let mut finished = false;
    while (sim.t < max_t || max_t < 0.0) && !finished {
        finished |= sim.update();
    }
    if finished { Some(sim) } else { None }
}
