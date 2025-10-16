use roundabout_sim::*;
use std::f32::consts::PI;
use approx::{assert_abs_diff_eq, assert_relative_eq};

#[cfg(test)]
mod tests {
    use super::*;
    const RELATIVE: f32 = 1e-1;

    fn check_completion_order(filename: &str, max_t: f32, order: &[usize]) {
        let sim = sim_run(filename, max_t).unwrap();
        assert_eq!(sim.finished_cars.len(), order.len(), "not all cars finished");
        for (i, car) in sim.finished_cars.iter().enumerate() {
            assert_eq!(car.borrow().id, order[i]);
        }
    }

    #[test]
    fn sim_single_test() {
        assert_relative_eq!(sim_run("single.json", 10.0).unwrap().t, PI, max_relative = RELATIVE);
        assert_abs_diff_eq!(sim_run("single_finish.json", 10.0).unwrap().t, 0.0, epsilon = RELATIVE);
        assert_relative_eq!(sim_run("single_switch_in.json", 10.0).unwrap().t, 2.57, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("single_no_switch.json", 10.0).unwrap().t, 0.64, max_relative = RELATIVE);
    }

    #[test]
    /**
        single lane, circular for straight-straight collision test
        Ensure every one finishes
    */
    fn sim_circular_deadlock_test() {
        assert_relative_eq!(sim_run("circular_4.json", 10.0).unwrap().t, 2.0 * PI / 4.0, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("circular_36.json", 10.0).unwrap().t, 2.0 * PI / 36.0, max_relative = RELATIVE);
        assert_relative_eq!(sim_run("circular_360.json", 10.0).unwrap().t, 2.0 * PI / 360.0, max_relative = RELATIVE);
    }

    #[test]
    /**
        single lane, fast slow test, simulation ends with the slower one
    */
    fn sim_fast_slow_test() {
        assert_relative_eq!(sim_run("fast_slow_2.json", 10.0).unwrap().t, (PI - 0.2) / 0.5 + 0.1, max_relative = RELATIVE);
    }

    #[test]
    /**
        Verify switch-first by check their completion order
    */
    fn sim_switch_first_policy() {
        check_completion_order("first_switch.json", 10.0, &[0, 1]);
        check_completion_order("first_switch_3.json", 10.0, &[0, 1, 2]);
    }
    #[test]
    /**
        Verify straight-first by check their completion order
    */
    fn sim_straight_first_policy() {
        check_completion_order("first_straight.json", 10.0, &[1, 0]);
        check_completion_order("first_straight_3.json", 10.0, &[2, 1, 0]);
    }

    #[test]
    #[ignore]
    /**
        Verify straight-first by check their completion order
    */
    fn long_test() {
        sim_run("rand_30_4_5..1.json", 3000.0).unwrap();
        sim_run("rand_300_8_5..1.json", 3000.0).unwrap();
    }
}