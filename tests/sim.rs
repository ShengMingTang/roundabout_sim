use roundabout_sim::*;
use std::f32::consts::PI;
use approx::{assert_abs_diff_eq, assert_relative_eq};

#[cfg(test)]
mod tests {
    use super::*;
    const RELATIVE: f32 = 1e-1;
    #[test]
    fn sim_single_test() {
        assert_relative_eq!(sim_run("single.json").unwrap(), PI, max_relative = RELATIVE);
        assert_abs_diff_eq!(sim_run("single_finish.json").unwrap(), 0.0, epsilon = RELATIVE);
        assert_relative_eq!(sim_run("single_switch_in.json").unwrap(), 2.57, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("single_no_switch.json").unwrap(), 0.64, max_relative = RELATIVE);
    }

    #[test]
    /**
        single lane, circular for straight-straight collision test
        Ensure every one finishes
    */
    fn sim_circular_deadlock_test() {
        assert_relative_eq!(sim_run("circular_4.json").unwrap(), 2.0 * PI / 4.0, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("circular_36.json").unwrap(), 2.0 * PI / 36.0, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("circular_360.json").unwrap(), 2.0 * PI / 360.0, max_relative = RELATIVE);
    }

    #[test]
    /**
        single lane, fast slow test, simulation ends with the slower one
    */
    fn sim_fast_slow_test() {
        assert_relative_eq!(sim_run("fast_slow_2.json").unwrap(), (PI - 0.2) / 0.5 + 0.1, max_relative = RELATIVE);
    }
}