use roundabout_sim::*;
use json;
use num_complex::Complex;
use std::env;

fn gen_test() {
    let jobj = RoundaboutSimSetting::gen_circular(4);
    let s = json::stringify(jobj);
    println!("{s}");
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert!(args.len() > 1);
    sim_run(&args[1]).unwrap();
}
