use num_complex::Complex;
use std::cell::RefCell;
use std::f32::consts::PI;
use std::rc::Rc;

pub type Shared<T> = Rc<RefCell<T>>;
pub const THETA_ALLOW: f32 = 1e-2 * PI;
const DRIFT_ALLOW: f32 = 1e-2;

#[derive(Debug)]
pub enum Action {
    Switch(i32), // switch in/out, > 0 means to inner, == 0 means stop
    Straight,
    Stop,
}

pub const SWITCH_OUT: Action = Action::Switch(-1);
pub const SWITCH_IN: Action = Action::Switch(1);

pub fn unwrap_theta(theta: f32) -> f32 {
    if theta < 0.0 { theta + 2.0 * PI } else { theta }
}

pub fn is_on_lane(pos: &Complex<f32>, r_lane: f32) -> bool {
    let ideal_pos = Complex::from_polar(r_lane, pos.arg());
    (pos - ideal_pos).norm() <= DRIFT_ALLOW
}
