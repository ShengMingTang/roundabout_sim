// SPDX-License-Identifier: GPL-3.0-or-later
mod render;
pub use crate::render::render_run;

use json::{JsonValue, object};
use num_complex::Complex;
use std::f32::consts::PI;
use std::fs;
use std::rc::Rc;
use std::cell::RefCell;
use std::collections::{HashMap};
use ordered_float::OrderedFloat;
use rand;

type Shared<T> = Rc<RefCell<T>>;

#[derive(Debug)]
enum Action {
    Switch(i32), // switch in/out, > 0 means to inner, == 0 means stop
    Straight,
    Stop,
}

pub trait Driver {
    fn drive(&self, car: Shared<Car>, setting: &RoundaboutSimSetting, others: &Vec<Shared<Car>>) -> Action;
}

struct SimpleDriver;
impl Driver for SimpleDriver {
    fn drive(&self, car: Shared<Car>, setting: &RoundaboutSimSetting, others: &Vec<Shared<Car>>) -> Action {
        let car = &car.borrow();
        let rem_theta = (car.dst / car.pos).to_polar().1.abs(); // remaining
        if car.finished() { // finished
            Action::Stop
        }
        else if car.lane > 0 && rem_theta <= THETA_ALLOW { // switch out
            Action::Switch(-1)
        }
        else { // greedy
            // cost for straight then switch out
            let r0 = setting.r_lanes[0];
            let r_curr = setting.r_lanes[car.lane];
            let unwrapped_theta = unwrap_theta((car.dst / car.pos).arg());
            // (arc) + (switch out)
            let straight_dist = (r_curr * unwrapped_theta) + (r0 - r_curr);
            let switch_in_dist = if car.lane >= setting.r_lanes.len() - 1 { // can't switch in
                std::f32::INFINITY
            } else { // 2 * (switch one in/out) + (inner arc) + (switch from curr to outter most)
                let r_inner = setting.r_lanes[car.lane + 1];
                (2.0 * r_inner) + (r_inner * unwrapped_theta) + (r0 - r_curr)
            };
            
            if switch_in_dist < straight_dist && car.lane < setting.r_lanes.len() - 1 {
                Action::Switch(1)
            } else {
                Action::Straight
            }
        }
    }
}

pub struct Car {
    pub id: usize,
    pos: Complex<f32>, // to tacke polar
    vel: f32,
    lane: usize, // 0 is the outermost
    dst: Complex<f32>, // destination polar
    action: Action,
    driver: Box<dyn Driver>, // driver to drive this car
}

const DIST_ALLOW: f32 = 1e-2;
const THETA_ALLOW: f32 = 1e-2 * PI;
const MIN_UPDATE_TICK: f32 = 1e-2;

fn unwrap_theta(theta: f32) -> f32 {
    if theta < 0.0 {
        theta + 2.0 * PI
    } else {
        theta
    }
}

impl Car {
    fn to_json(&self) -> JsonValue {
        object!{
            vel: self.vel,
            lane: self.lane,
            dst: self.dst.to_string(),
            theta: self.pos.to_polar().1,
        }
    }
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
                   *diff_lane > 0 && next_r <= target_r    /* switch in */ {
                    self.pos = Complex::from_polar(target_r, self.pos.arg());
                    self.lane = ((self.lane as i32) + diff_lane) as usize;
                }
                else {
                    self.pos = Complex::from_polar(next_r, self.pos.arg());
                }
            },
            Action::Straight => {
                let mv_theta = (self.vel * tick) / setting.r_lanes[self.lane];
                let next_theta = unwrap_theta(self.pos.arg()) + mv_theta;
                let target_theta = unwrap_theta(self.dst.arg());
                if next_theta >= target_theta {
                    self.pos = Complex::from_polar(self.pos.norm(), target_theta);
                }
                else {
                    self.pos = Complex::from_polar(self.pos.norm(), next_theta);
                }
            },
            Action::Stop => {},
        }
    }
}

#[derive(Debug)]
pub enum SwitchPolicy { // Handles the collision arising from switch to another lane
    SwitchFirst,
    StraightFirst,
    // Random(f32), // switch will succed with probability f32, but this will create an imprecise simulation 
}

#[derive(Debug)]
pub struct RoundaboutSimSetting {
    n_inter: usize, // intersection
    r_lanes: Vec<f32>, // radius of each lane
    tick: f32, // simulation update interval
    switch_policy: SwitchPolicy,
}
impl RoundaboutSimSetting {
    pub fn default() -> RoundaboutSimSetting {
        RoundaboutSimSetting {
            n_inter: 2,
            r_lanes: vec![1.0],
            tick: 0.1,
            switch_policy: SwitchPolicy::SwitchFirst,
        }
    }
    pub fn to_json(&self) -> JsonValue {
        object!{
            n_inter: self.n_inter,
            tick: self.tick,
            r_lanes: self.r_lanes.clone(),
        }
    }
    pub fn gen_random(n_cars: usize, n_inter: usize, r_lanes: &[f32]) -> JsonValue {
        assert!(n_cars > 0);
        assert!(n_inter > 0);
        assert!(r_lanes.len() > 0);
        {
            let mut r_lanes_reverse = r_lanes.to_vec();
            r_lanes_reverse.reverse();
            assert!(r_lanes_reverse.is_sorted());
        }
        let mut cars_json = JsonValue::new_object();
        for id in 0..n_cars {
            let mut cjson = Car {
                id,
                pos: Complex::default(),
                vel: rand::random::<f32>() + 0.2,
                lane: rand::random_range(0..r_lanes.len()),
                dst: Complex::default(),
                action: Action::Straight,
                driver: Box::new(SimpleDriver{}),
            }.to_json();
            cjson["dst"] = rand::random_range(0..n_inter).into();
            cjson["theta"] = (rand::random::<f32>() * 2.0 * PI).into();
            cars_json[id.to_string()] = cjson;
        }
        let setting = RoundaboutSimSetting {
            n_inter,
            r_lanes: r_lanes.to_vec(),
            ..RoundaboutSimSetting::default()
        };
        let mut jobj = setting.to_json();
        jobj["init"] = cars_json;
        jobj
    }
    pub fn gen_circular(n_cars: usize) -> JsonValue {
        assert!(n_cars > 0);
        let mut cars_json = JsonValue::new_object();
        for id in 0..n_cars {
            let mut cjson = Car {
                id,
                pos: Complex::default(),
                vel: 1.0,
                lane: 0,
                dst: Complex::default(),
                action: Action::Straight,
                driver: Box::new(SimpleDriver{})
            }.to_json();
            cjson["dst"] = ((id + 1) % n_cars).into();
            cjson["theta"] = (2.0 * PI / (n_cars as f32) * (id as f32)).into();
            cars_json[id.to_string()] = cjson;
        }
        let setting = RoundaboutSimSetting {
            n_inter: n_cars,
            ..RoundaboutSimSetting::default()
        };
        let mut jobj = setting.to_json();
        jobj["init"] = cars_json;
        jobj
    }
    pub fn new(jobj: &JsonValue) -> Option<RoundaboutSimSetting> {
        let mut r_lanes = vec![];
        for it in jobj["r_lanes"].members() {
            r_lanes.push(it.as_f32()?);
        }
        {
            let mut r_lanes_reverse = r_lanes.clone();
            r_lanes_reverse.reverse();
            assert!(r_lanes.len() > 0 && r_lanes_reverse.is_sorted(), "lanes should be of len > 0 and sorted in decreasing order");
        }
        let ret = RoundaboutSimSetting {
            n_inter: jobj["n_inter"].as_usize()?,
            r_lanes,
            tick: jobj["tick"].as_f32()?,
            switch_policy: if jobj.has_key("switch_policy") {
                match jobj["switch_policy"].as_str()? {
                    "SwitchFirst" => SwitchPolicy::SwitchFirst,
                    _ => SwitchPolicy::StraightFirst,
                }
            } else {
                RoundaboutSimSetting::default().switch_policy
            },
        };
        if ret.r_lanes.len() == 0 {
            None
        }
        else {
            Some(ret)
        }
    }
}

pub struct RoundaboutSim {
    pub t: f32, // current time,
    pub setting: RoundaboutSimSetting,
    pub finished_cars: Vec<Shared<Car>>,
    cars: Vec<Shared<Car>>,
}

impl RoundaboutSim {
    pub fn new(setting: RoundaboutSimSetting, jobj: &JsonValue) -> Option<RoundaboutSim> {
        if jobj.has_key("init") == false {
            return None;
        }
        let jinit = &jobj["init"];
        let mut cars = vec![];
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
                dst: Complex::from_polar(setting.r_lanes[0], 2.0 * PI / (setting.n_inter as f32) * value["dst"].as_f32()?),
                action: Action::Straight,
                driver: Box::new(SimpleDriver{}),
            })))
        }
        Some(RoundaboutSim {
            t: 0.0,
            setting,
            cars,
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
        return Some(seconds simulated) if simulation finished
        else None to indicate continue
    */
    pub fn update(&mut self) -> bool {
        let setting = &self.setting;
        let mut by_lane = HashMap::< usize, Vec< Shared<Car> > >::new();
        // TODO: Now is O(n lgn)
        self.cars.sort_by_key(|car| {OrderedFloat(car.borrow().pos.arg())});
        for car in &self.cars {
            let same_lane = by_lane.entry(car.borrow().lane)
                .or_insert(vec![]);
            same_lane.push(car.clone());
        }
        let mut tick = setting.tick;
        // every car determines its action
        for car in &self.cars {
            let action = {
                car.borrow().driver.drive(car.clone(), setting, &self.cars)
            };
            car.borrow_mut().set_action(action);
        }
        // detect straight collision, happens to the same lane
        let possible_straight_collision = |car_follow: &mut Car, car_precede: &Car| {
            let time_to_collide = self.straight_collision(
                car_follow,
                car_precede,
            );
            if time_to_collide <= MIN_UPDATE_TICK {
                car_follow.set_action(Action::Stop);
                // println!("Car {} makes Car {} stop", car_follow.id, car_precede.id);
                f32::MAX
            }
            else {
                time_to_collide
            }
        };
        for (lane, same_lane) in &by_lane {
            for (i, car_follow) in same_lane.iter().enumerate() {
                if let Some(ref car_precede) = same_lane.get((i + 1) % same_lane.len()) && same_lane.len() > 1 {
                    let this_tick = possible_straight_collision(
                            &mut car_follow.borrow_mut(),
                            &car_precede.borrow()
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
                        switching_car.pos = Complex::from_polar(setting.r_lanes[switching_car.lane], switching_car.pos.arg());
                        switching_car.set_action(Action::Stop);
                    },
                    _ => {
                        car_follow.set_action(Action::Stop);
                    },
                }
            }
        };
        for (lane, same_lane) in &by_lane {
            for car in same_lane {
                let mut switching_car = car.borrow_mut();
                let switching_theta = switching_car.pos.arg();
                match switching_car.action {
                    Action::Switch(diff_lane) => {
                        let next_lane = (*lane as i32) + diff_lane; 
                        match by_lane.get(&(next_lane as usize)) {
                            Some(ref other_lane) => {
                                // detect from lower bound on other lane
                                let idx = match other_lane.binary_search_by(|probe| {
                                    OrderedFloat(probe.borrow().pos.arg()).cmp(&OrderedFloat(switching_theta))
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
                                match other_lane.last() {
                                    Some(ref car_follow) => {
                                        let mut car_follow = car_follow.borrow_mut();
                                        possbile_switch_collision(&mut switching_car, &mut car_follow);
                                    },
                                    _ => {},
                                }
                            },
                            _ => {},
                        }                   
                    },
                    _ => {},
                }                
            }
        }
        self.t += tick;
        let mut has_progress = false;
        // TODO: Another chance for changing their actions?
        // update phase
        let mut next_cars = vec![];
        let n_cars = self.finished_cars.len() + self.cars.len();
        for car in self.cars.iter() {
            {
                car.borrow_mut().update(tick, setting);
            }
            match car.borrow().action {
                Action::Stop => {
                },
                _ => {
                    has_progress = true;
                },
            };
            if car.borrow().finished() {
                self.finished_cars.push(car.clone());
                has_progress = true;
                println!("Car {} finishes at time {}, ({} / {n_cars})",
                    car.borrow().id, self.t,
                    self.finished_cars.len()
                );
            }
            else {
                next_cars.push(car.clone());
            }
        }
        self.cars = next_cars;
        let all_finished = self.cars.len() == 0;
        assert!(has_progress || all_finished, "everyone stops but not finished");
        if all_finished {
            println!("===== simulation finished in: {} =====", self.t);
        }
        else {
            // println!("===== t: {} (+{}) =====", self.t, tick);
            // for car in &self.cars {
            //     println!("id: {}: {:?}", car.borrow().id, car.borrow());
            // }            
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
                    let other_target_pos = Complex::from_polar(r_lane, car_other.pos.arg() + car_other.vel * tick / (r_lane as f32));
                    // @car_switch is in the arc of @car_other
                    let other_curr_2_swtich_target = switch_target_pos / other_curr_pos;
                    let switch_target_2_other_target = other_target_pos / switch_target_pos;
                    other_curr_2_swtich_target.arg() >= 0.0 && switch_target_2_other_target.arg() <= 0.0
                },
                _ => false,                    
            }
        }
        else {
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
                assert_eq!(car_follow.lane, car_precede.lane,
                    "on the same lane but straight-straight collision called",
                );
                assert_ne!(car_follow.id, car_precede.id,
                    "have the same id"
                );
                let margin_theta = unwrap_theta((car_precede.pos.fdiv(car_follow.pos)).arg());
                let lane = car_follow.lane;
                margin_theta * self.setting.r_lanes[lane] / car_follow.vel
            },
            _ => {return f32::MAX;},
        }
    }
}



pub fn sim_run(filename: &str, max_t: f32) -> Option<RoundaboutSim> {
    let mut sim = RoundaboutSim::from_json(filename)?;
    let mut finished = false;
    while (sim.t < max_t || max_t < 0.0) && finished == false {
        finished |= sim.update();
    }
    return if finished {
        Some(sim)
    } else {
        None
    }
}