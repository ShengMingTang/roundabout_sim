use json::{JsonValue, object};
use num_complex::Complex;
use std::f32::consts::PI;
use std::fs;
use std::rc::Rc;
use std::cell::RefCell;
use std::collections::{HashMap, BTreeMap};
use ordered_float::OrderedFloat;
use std::ops::Bound;

type Shared<T> = Rc<RefCell<T>>;

#[derive(Debug)]
enum Action {
    Switch(i32), // switch in/out, > 0 means to inner, == 0 means stop
    Straight,
    Stop,
}

#[derive(Debug)]
struct Car {
    id: usize,
    pos: Complex<f32>, // to tacke polar
    vel: f32,
    lane: usize, // 0 is the outermost
    dst: Complex<f32>, // destination polar
    action: Action,
}

const DIST_ALLOW: f32 = 1e-2;
const THETA_ALLOW: f32 = 1e-2 * PI;
const SAFE_DISTANCE: f32 = 1e-2;

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
            id: self.id,
            pos: self.pos.to_string(),
            vel: self.vel,
            lane: self.lane,
            dst: self.dst.to_string(),
            action: format!("{:?}", self.action),
            r: self.pos.to_polar().0,
            theta: self.pos.to_polar().1,
        }
    }
    fn finished(&self) -> bool {        
        self.lane == 0 && (self.dst - self.pos).norm() <= DIST_ALLOW
    }
    /**
        provide an action
    */
    fn action(&self, setting: &RoundaboutSimSetting) -> Action {
        let rem_theta = (self.dst / self.pos).to_polar().1.abs(); // remaining
        if self.finished() { // finished
            Action::Stop
        }
        else if self.lane > 0 && rem_theta <= THETA_ALLOW { // switch out
            Action::Switch(-1)
        }
        else { // greedy
            // cost for straight then switch out
            let r0 = setting.r_lanes[0];
            let r_curr = setting.r_lanes[self.lane];
            let unwrapped_theta = unwrap_theta((self.dst / self.pos).arg());
            // (arc) + (switch out)
            let straight_dist = (r_curr * unwrapped_theta) + (r0 - r_curr);
            let switch_in_dist = if self.lane >= setting.r_lanes.len() - 1 { // can't switch in
                std::f32::INFINITY
            } else { // 2 * (switch one in/out) + (inner arc) + (switch from curr to outter most)
                let r_inner = setting.r_lanes[self.lane + 1];
                (2.0 * r_inner) + (r_inner * unwrapped_theta) + (r0 - r_curr)
            };
            
            if switch_in_dist < straight_dist && self.lane < setting.r_lanes.len() - 1 {
                Action::Switch(1)
            } else {
                Action::Straight
            }
        }
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
                    self.lane = ((self.lane as i32)+ diff_lane) as usize;
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
    Random(f32), // switch will succed with probability f32
}

#[derive(Debug)]
pub struct RoundaboutSimSetting {
    n_inter: usize, // intersection
    n_cars: usize, // no. cars
    r_lanes: Vec<f32>, // radius of each lane
    tick: f32, // simulation update interval
    switch_policy: SwitchPolicy,
}
impl RoundaboutSimSetting {
    pub fn default() -> RoundaboutSimSetting {
        RoundaboutSimSetting {
            n_inter: 2,
            n_cars: 1,
            r_lanes: vec![1.0],
            tick: 0.1,
            switch_policy: SwitchPolicy::StraightFirst,
        }
    }
    pub fn to_json(&self) -> JsonValue {
        object!{
            n_inter: self.n_inter,
            n_cars: self.n_cars,
            tick: self.tick,
            r_lanes: self.r_lanes.clone(),
        }
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
            }.to_json();
            cjson["src"] = id.into();
            cjson["dst"] = ((id + 1) % n_cars).into();
            cjson["theta"] = (2.0 * PI / (n_cars as f32) * (id as f32)).into();
            cars_json[id.to_string()] = cjson;
        }
        let setting = RoundaboutSimSetting {
            n_inter: n_cars,
            n_cars,
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
        let mut r_lanes_reverse = r_lanes.clone();
        r_lanes_reverse.reverse();
        assert!(r_lanes.len() > 0 && r_lanes_reverse.is_sorted(), "lanes should be of len > 0 and sorted in decreasing order");
        let ret = RoundaboutSimSetting {
            n_inter: jobj["n_inter"].as_usize()?,
            n_cars: jobj["n_cars"].as_usize()?,
            r_lanes,
            tick: jobj["tick"].as_f32()?,
            switch_policy: if jobj.has_key("switch_policy") {
                match jobj["switch_policy"].as_str()? {
                    "SwitchFirst" => SwitchPolicy::SwitchFirst,
                    "StraightFirst" => SwitchPolicy::StraightFirst,
                    _ => SwitchPolicy::Random(jobj["switch_policy"].as_str()?.parse::<f32>().ok()?),
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

#[derive(Debug)]
pub struct RoundaboutSim {
    t: f32, // current time,
    setting: RoundaboutSimSetting,
    cars: Vec<Shared<Car>>,
    finished_cars: Vec<Shared<Car>>,
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
                action: Action::Switch(0),
            })))
        }
        Some(RoundaboutSim {
            t: 0.0,
            setting,
            cars,
            finished_cars: vec![],
        })
    }
    /**
        return Some(seconds simulated) if simulation finished
        else None to indicate continue
    */
    pub fn update(&mut self) -> bool {
        let setting = &self.setting;
        let mut by_lane = HashMap::< usize, BTreeMap<OrderedFloat::<f32>, Shared<Car>> >::new();
        for car in &self.cars {
            let same_lane = by_lane.entry(car.borrow().lane)
                .or_insert(BTreeMap::<OrderedFloat::<f32>, Shared<Car>>::new());
            same_lane.insert(
                OrderedFloat::<f32>(car.borrow().pos.arg()),
                car.clone()
            );
        }
        let mut tick = setting.tick;
        for car in &self.cars {
            let action = {
                car.borrow().action(setting)
            };
            car.borrow_mut().set_action(action);
        }
        // detect straight collision, happens to the same lane
        let possible_straight_collision = |car_follow: &mut Car, car_precede: &Car| {
            let time_to_collide = self.straight_collision(
                car_follow,
                car_precede,
            );
            if time_to_collide <= 0.0 {
                car_follow.set_action(Action::Stop);
                println!("Reject Car {} by Car {}", car_follow.id, car_precede.id);
                f32::MAX
            }
            else {
                time_to_collide
            }
        };
        for (_lane, same_lane) in &by_lane {
            for ((_, car_follow), (_, car_precede)) in same_lane.iter().zip(same_lane.iter().skip(1)) {
                tick = f32::min(
                    possible_straight_collision(&mut car_follow.borrow_mut(), &car_precede.borrow()),
                    tick
                );
            }
            // last one is missed
            if same_lane.len() > 1 {
                let first_car = same_lane.first_key_value().unwrap().1.borrow();
                let mut last_car = same_lane.last_key_value().unwrap().1.borrow_mut();
                tick = f32::min(
                    possible_straight_collision(&mut last_car, &first_car),
                    tick
                );
            }
        }
        // detect switch collision
        let possbile_switch_collision = |switching_car: &mut Car, car_follow: &mut Car| {
            if self.switch_collision(switching_car, car_follow, tick) {
                match setting.switch_policy {
                    SwitchPolicy::StraightFirst => {
                        switching_car.set_action(Action::Stop);
                    },
                    _ => {
                        car_follow.set_action(Action::Stop);
                    },
                }
            }
        };
        for (lane, same_lane) in &by_lane {
            for (theta, car) in same_lane {
                let mut switching_car = car.borrow_mut();
                match switching_car.action {
                    Action::Switch(diff_lane) => {
                        let next_lane = (*lane as i32) + diff_lane;
                        match by_lane.get(&(next_lane as usize)) {
                            Some(ref other_lane) => {
                                // let cursor = other_lane.lower_bound(Bound::Included(theta));
                                // if let Some((_k, car_follow)) = cursor.peek_prev() {
                                //     let mut car_follow = car_follow.borrow_mut();
                                //     possbile_switch_collision(&mut switching_car, &mut car_follow);
                                // }
                                // tail condition
                                match other_lane.last_key_value() {
                                    Some(ref car_follow) => {
                                        let mut car_follow = car_follow.1.borrow_mut();
                                        possbile_switch_collision(&mut switching_car, &mut car_follow);
                                    },
                                    _ => {},
                                }
                            },
                            _ => {},
                        }
                        if next_lane < 0 || next_lane >= setting.r_lanes.len() as i32 {
                            continue;
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
            }
            else {
                next_cars.push(car.clone());
            }
        }
        self.cars = next_cars;
        let all_finished = self.cars.len() == 0;
        assert!(has_progress || all_finished, "everyone stops but not finished");
        println!("===== t: {} (+{}) =====", self.t, tick);
        for car in &self.cars {
            println!("id: {}: {}", car.borrow().id, json::stringify(car.borrow().to_json()));
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
                if (car_precede.pos - car_follow.pos).norm() < SAFE_DISTANCE &&
                    (car_precede.pos / car_follow.pos).arg() > 0.0 {
                    return 0.0;
                }
                let margin_theta = unwrap_theta((car_precede.pos / car_follow.pos).arg());
                let lane = car_follow.lane;
                margin_theta / (car_follow.vel / self.setting.r_lanes[lane])
            },
            _ => {return f32::MAX;},
        }
    }
}



pub fn sim_run(filename: &str, max_t: f32) -> Option<f32> {
    let contents = fs::read_to_string(filename).expect("File not found");
    let jobj = json::parse(&contents).expect("file format error");
    let settings = RoundaboutSimSetting::new(&jobj).expect("some required key not specified");
    let mut sim = RoundaboutSim::new(settings, &jobj).expect("init cars format error");
    println!("sim init: {sim:?}");
    let mut finished = false;
    while sim.t < max_t && finished == false {
        finished |= sim.update();
    }
    return if finished {
        Some(sim.t)
    } else {
        None
    }
}