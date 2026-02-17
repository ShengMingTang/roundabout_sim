use crate::Car;
use crate::RoundaboutSimSetting;
use crate::common::{Action, Shared, THETA_ALLOW, unwrap_theta};
use crate::common::{SWITCH_IN, SWITCH_OUT};

pub trait Driver {
    fn drive(
        &self,
        car: Shared<Car>,
        setting: &RoundaboutSimSetting,
        others: &[Shared<Car>],
    ) -> Action;
}

pub struct DriverFactory {}

struct SimpleDriver;

impl DriverFactory {
    pub fn make_default_driver_boxed() -> Box<dyn Driver> {
        Box::new(SimpleDriver {})
    }
}

impl Driver for SimpleDriver {
    fn drive(
        &self,
        car: Shared<Car>,
        setting: &RoundaboutSimSetting,
        _others: &[Shared<Car>],
    ) -> Action {
        let car = &car.borrow();
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
