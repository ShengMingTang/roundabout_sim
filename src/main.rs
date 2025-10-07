use roundabout_sim::*;
use json;
use std::env;

fn gen_circular(i: usize) {
    let jobj = RoundaboutSimSetting::gen_circular(i);
    let s = json::stringify(jobj);
    println!("{s}");
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert!(args.len() > 1);
    if args[1] == "circular" {
        assert!(args.len() > 2, "circular <number of cars> evens placed");
        gen_circular(args[2].parse().expect("expect usize"));
    }
    else {
        sim_run(&args[1]).unwrap();
    }
}
