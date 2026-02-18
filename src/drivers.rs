use crate::Car;
use crate::RoundaboutSimSetting;
use crate::common::is_on_lane;
use crate::common::{Action, THETA_ALLOW, unwrap_theta};
use crate::common::{SWITCH_IN, SWITCH_OUT};
use core::f32;
use json::JsonValue;
use num_complex::Complex;

pub trait Driver {
    fn init(&mut self, _car: &Car, _setting: &RoundaboutSimSetting) {}
    fn drive(&self, car: &Car, _ts: f32, setting: &RoundaboutSimSetting) -> Action;
    fn update(&mut self, _car: &Car, _ts: f32, _setting: &RoundaboutSimSetting) {}
}

pub struct DriverFactory {}

struct ShortestDistDriver;

const SHORTEST_TIME_DRIVER_MIN_STAY: f32 = 5.0;
const SHORTEST_TIME_DRIVER_REFRESH: f32 = 20.0;

struct ShortestTimeDriver {
    lane_last_ts: Vec<f32>,
    lane_last_pos: Vec<Complex<f32>>,
    lane_vel: Vec<f32>,
    prev_lane: usize,
}

impl DriverFactory {
    fn make_shortest_dist_driver_boxed() -> Box<dyn Driver> {
        Box::new(ShortestDistDriver {})
    }
    fn make_shortest_time_driver_boxed() -> Box<dyn Driver> {
        Box::new(ShortestTimeDriver {
            lane_last_ts: Vec::new(),
            lane_last_pos: Vec::new(),
            lane_vel: Vec::new(),
            prev_lane: 0,
        })
    }
    fn make_default_driver_boxed() -> Box<dyn Driver> {
        DriverFactory::make_shortest_dist_driver_boxed()
    }
    pub fn make_boxed_from_json(jobj: &JsonValue) -> Box<dyn Driver> {
        if let JsonValue::Short(short) = jobj {
            let name = short.as_str();
            if name == "ShortestDist" {
                DriverFactory::make_shortest_dist_driver_boxed()
            } else if name == "ShortestTime" {
                DriverFactory::make_shortest_time_driver_boxed()
            } else {
                DriverFactory::make_default_driver_boxed()
            }
        } else {
            DriverFactory::make_default_driver_boxed()
        }
    }
}

impl Driver for ShortestDistDriver {
    fn drive(&self, car: &Car, _ts: f32, setting: &RoundaboutSimSetting) -> Action {
        let rem_theta = (car.dst / car.pos).to_polar().1.abs(); // remaining
        if car.finished() {
            // finished
            Action::Stop
        } else if car.lane > 0 && rem_theta <= THETA_ALLOW {
            // switch out
            SWITCH_OUT
        } else {
            // greedy
            // cost for straight then switch out
            let r0 = setting.r_lanes[0];
            let r_curr = setting.r_lanes[car.lane];
            let unwrapped_theta = unwrap_theta((car.dst / car.pos).arg());
            // (arc) + (switch out)
            let straight_dist = (r_curr * unwrapped_theta) + (r0 - r_curr);
            let switch_in_dist = if car.lane + 1 >= setting.r_lanes.len() {
                // can't switch in
                f32::INFINITY
            } else {
                // 2 * (switch one in/out) + (inner arc) + (switch from curr to outter most)
                let r_inner = setting.r_lanes[car.lane + 1];
                (2.0 * (r_curr - r_inner)) + (r_inner * unwrapped_theta) + (r0 - r_curr)
            };

            if switch_in_dist < straight_dist && car.lane < setting.r_lanes.len() - 1 {
                SWITCH_IN
            } else {
                Action::Straight
            }
        }
    }
}

impl Driver for ShortestTimeDriver {
    fn init(&mut self, car: &Car, setting: &RoundaboutSimSetting) {
        self.lane_last_ts = vec![0.0; setting.r_lanes.len()];
        self.lane_last_pos = vec![car.pos; setting.r_lanes.len()];
        self.lane_vel = vec![car.vel; setting.r_lanes.len()];
    }
    fn drive(&self, car: &Car, ts: f32, setting: &RoundaboutSimSetting) -> Action {
        /*
         * Choose the lane that gives the earlies arrival time
         * Discard lane vel record that are too long ago
         */
        if ts - self.lane_last_ts[car.lane] < SHORTEST_TIME_DRIVER_MIN_STAY
            && is_on_lane(&car.pos, setting.r_lanes[car.lane])
        {
            return Action::Straight;
        }
        let mut min_time = f32::INFINITY;
        let mut min_lane = car.lane;
        for (i, r_lane) in setting.r_lanes.iter().enumerate() {
            let lane_vel = self.lane_vel[i];
            let unwrapped_theta = unwrap_theta((car.dst / car.pos).arg());
            let lane_time = r_lane * unwrapped_theta / lane_vel
                + (setting.r_lanes[i] - setting.r_lanes[car.lane]).abs() / car.vel
                + (setting.r_lanes[0] - setting.r_lanes[i]) / car.vel;
            if lane_time < min_time {
                min_time = lane_time;
                min_lane = i;
            }
        }
        if min_lane < car.lane {
            SWITCH_OUT
        } else if min_lane > car.lane {
            SWITCH_IN
        } else {
            // min_kane == car.lane
            if is_on_lane(&car.pos, setting.r_lanes[car.lane]) {
                Action::Straight
            } else if car.pos.norm() > setting.r_lanes[car.lane] {
                // halfway switching out, continue
                SWITCH_OUT
            } else {
                SWITCH_IN
            }
        }
    }
    fn update(&mut self, car: &Car, ts: f32, setting: &RoundaboutSimSetting) {
        let Car {
            lane,
            action,
            pos,
            vel,
            ..
        } = *car;
        // reset lane record
        if self.prev_lane != car.lane {
            self.prev_lane = lane;
            self.lane_vel[lane] = vel;
            self.lane_last_pos[lane] = pos;
            self.lane_last_ts[lane] = ts;
        }
        // refresh the oldest record
        let mut need_refresh = None;
        let mut oldest_ts = f32::INFINITY;
        for (i, last_ts) in self.lane_last_ts.iter_mut().enumerate() {
            if ts - *last_ts > SHORTEST_TIME_DRIVER_REFRESH && i != lane && *last_ts < oldest_ts {
                oldest_ts = *last_ts;
                need_refresh = Some(i);
            }
        }
        if let Some(i) = need_refresh {
            self.lane_vel[i] *= 2.0;
            if self.lane_vel[i] > vel {
                self.lane_vel[i] = vel;
            }
            self.lane_last_ts[i] = ts;
        }
        match action {
            Action::Straight | Action::Stop => {
                self.lane_vel[lane] = setting.r_lanes[lane]
                    * unwrap_theta((pos / self.lane_last_pos[lane]).arg())
                    / (ts - self.lane_last_ts[lane]);
                if self.lane_vel[lane] > vel {
                    self.lane_vel[lane] = vel;
                }
            }
            _ => {}
        }
        self.prev_lane = car.lane;
    }
}
