use json::{JsonValue, object};
use std::f32::consts::PI;
use crate::DriverFactory;
#[derive(Debug)]
pub enum SwitchPolicy { // Handles the collision arising from switch to another lane
    SwitchFirst,
    StraightFirst,
    // Random(f32), // switch will succed with probability f32, but this will create an imprecise simulation 
}

#[derive(Debug)]
pub struct RoundaboutSimSetting {
    pub n_inter: usize, // intersection
    pub r_lanes: Vec<f32>, // radius of each lane
    pub tick: f32, // simulation update interval
    pub switch_policy: SwitchPolicy,
    // TODO: may provide DriverFactory so that other does not need to know detail
}

impl RoundaboutSimSetting {
    pub fn default() -> RoundaboutSimSetting {
        RoundaboutSimSetting {
            n_inter: 2,
            r_lanes: vec![1.0],
            tick: 0.1,
            switch_policy: SwitchPolicy::StraightFirst,
        }
    }
    pub fn to_json(&self) -> JsonValue {
        object!{
            n_inter: self.n_inter,
            tick: self.tick,
            r_lanes: self.r_lanes.clone(),
            switch_policy: format!("{:?}", SwitchPolicy::StraightFirst),
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
            let cjson = object!{
                vel: rand::random::<f32>() + 0.2,
                lane: rand::random_range(0..r_lanes.len()),
                dst: rand::random_range(0..n_inter),
                theta: rand::random::<f32>() * 2.0 * PI,
            };
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
            let cjson = object!{
                vel: 1.0,
                lane: 0,
                dst: (id + 1) % n_cars,
                theta: 2.0 * PI / (n_cars as f32) * (id as f32),
            };
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