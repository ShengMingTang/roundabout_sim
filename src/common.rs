use std::rc::Rc;
use std::cell::RefCell;
use std::f32::consts::PI;

pub type Shared<T> = Rc<RefCell<T>>;
pub const THETA_ALLOW: f32 = 1e-2 * PI;

#[derive(Debug)]
pub enum Action {
    Switch(i32), // switch in/out, > 0 means to inner, == 0 means stop
    Straight,
    Stop,
}

pub fn unwrap_theta(theta: f32) -> f32 {
    if theta < 0.0 {
        theta + 2.0 * PI
    } else {
        theta
    }
}