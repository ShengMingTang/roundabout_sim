use json::{JsonValue, object};
use num_complex::Complex;
use std::f32::consts::PI;
use std::fs;
use std::rc::Rc;
use std::cell::RefCell;
// to polar will use [-pi, +pi]


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
    fn action(&self, tick: f32, setting: &RoundaboutSimSetting) -> Action {
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
        default action if first action is rejected
    */
    fn default_action(&self) -> Action {
        Action::Stop
    }
    /**
        update according to verified action
    */
    fn update(&mut self, tick: f32, setting: &RoundaboutSimSetting, action: Action) {
        self.action = action;
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
pub struct RoundaboutSimSetting {
    n_inter: usize, // intersection
    n_cars: usize, // no. cars
    r_lanes: Vec<f32>, // radius of each lane
    tick: f32, // simulation update interval
}
impl RoundaboutSimSetting {
    pub fn default() -> RoundaboutSimSetting {
        RoundaboutSimSetting {
            n_inter: 2,
            n_cars: 1,
            r_lanes: vec![1.0],
            tick: 0.1,
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
    cars: Vec<Car>,
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
            cars.push(Car {
                id: key.parse().ok()?,
                pos: Complex::from_polar(r, theta),
                vel: value["vel"].as_f32()?,
                lane,
                dst: Complex::from_polar(setting.r_lanes[0], 2.0 * PI / (setting.n_inter as f32) * value["dst"].as_f32()?),
                action: Action::Switch(0),
            })
        }
        Some(RoundaboutSim {
            t: 0.0,
            setting,
            cars,
        })
    }
    /**
        max_t: maximum tick simulated

    */
    pub fn start(self, max_t: f32) -> Result<String, String> {
        let mut t = 0.0;
        let setting = &self.setting;
        let cars: Vec<Rc<RefCell<Car>>> = self.cars.into_iter().map(|car| {Rc::new(RefCell::new(car))}).collect();
        loop {
            let mut actions = vec![];
            for car in &cars {
                actions.push(car.borrow().action(setting.tick, setting));
            }
            let mut all_finished = true;
            for (car, action) in cars.iter().zip(actions.into_iter()) {
                {
                    car.borrow_mut().update(setting.tick, setting, action);
                }
                all_finished &= car.borrow().finished();
            }
            t += self.setting.tick;
            println!("===== t: {t} =====");
            for car in &cars {
                println!("id: {}: {}", car.borrow().id, json::stringify(car.borrow().to_json()));
            }
            if all_finished {
                return Ok(String::from("finished"));
            }
            if t >= max_t {
                return Err(String::from("timeout"));
            }
        }
        Err(String::from("error"))
    }
    /**
        Collision if c0 is switch in/out to the c1.lane and
        c1.polar.theta is in the arc occupied by c1
        or 
        vice versa of c0 and c1
        * TODO:
    */
    fn switch_collision(&self, c0: &Car, c1: &Car, tick: f32) -> bool {
        if let Action::Straight = c1.action {
            match c0.action {
                Action::Switch(diff_lane) => {
                    let next_lane = (c0.lane as i32 + diff_lane) as usize;
                    let t_switch = (self.setting.r_lanes[c0.lane] - self.setting.r_lanes[next_lane]).abs();
                    let diff_theta = c1.vel / self.setting.r_lanes[c1.lane];
                    let rotate = Complex::from_polar(1.0, diff_theta);
                    (next_lane == c1.lane) && 
                    (false) // TODO: c0 is in the arc of c2
                },
                _ => false,                    
            }
        }
        else {
            false
        }
    }
}



pub fn sim_run(filename: &str) -> Result<String, String> {
    let contents = fs::read_to_string(filename).expect("File not found");
    let jobj = json::parse(&contents).expect("file format error");
    let settings = RoundaboutSimSetting::new(&jobj).expect("some required key not specified");
    let mut sim = RoundaboutSim::new(settings, &jobj).expect("init cars format error");
    println!("sim init: {sim:?}");
    sim.start(10.0)
}