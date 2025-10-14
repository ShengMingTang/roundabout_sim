use macroquad::prelude::*;
use roundabout_sim::*;
use json;
use std::env;

#[macroquad::main("Roundabout")]
async fn main() {
    let args: Vec<String> = env::args().collect();
    assert!(args.len() > 1);
    render_run(&args[1], 10.0).await;
}

fn gen_circular(i: usize) {
    let jobj = RoundaboutSimSetting::gen_circular(i);
    let s = json::stringify(jobj);
    println!("{s}");
}

fn headless() {
    let args: Vec<String> = env::args().collect();
    assert!(args.len() > 1);
    if args[1] == "circular" {
        assert!(args.len() > 2, "circular <number of cars> evens placed");
        gen_circular(args[2].parse().expect("expect usize"));
    }
    else {
        sim_run(&args[1], 1e2).unwrap();
    }
}
