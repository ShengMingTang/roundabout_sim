use roundabout_sim::*;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sim_single_test() {
        sim_run("single.json").unwrap();
        sim_run("single_finish.json").unwrap();
        sim_run("single_switch_in.json").unwrap();
    }
}