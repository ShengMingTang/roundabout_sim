use macroquad::prelude::*;
use roundabout_sim::*;
use json;
use std::env;

fn help() {
    println!("     : cargo run -- headless <path_to_json>");
    println!("     : cargo run -- <path_to_json>");
    println!("usage: cargo run -- gen_circular <n_cars>");
    println!("usage: cargo run -- gen_random <n_cars> <n_inter> <r_lanes[0]> <r_lanes[1]> ...");
}

#[macroquad::main("Roundabout")]
async fn main() {
    let args: Vec<String> = env::args().collect();
    if args[1] == "gen_random" {
        if args.len() < 5 {
            help();
        }
        let n_cars = args[2].parse::<usize>().expect("expect usize");
        let n_dst = args[3].parse::<usize>().expect("expect usize");
        let mut r_lanes = vec![];
        for i in 4..args.len() {
            r_lanes.push(args[i].parse::<f32>().expect("expect f32"));
        }
        let jobj = RoundaboutSimSetting::gen_random(n_cars, n_dst, &r_lanes);
        println!("{}", json::stringify(jobj));
    }
    else if args[1] == "gen_circular" {
        if args.len() < 3 {
            help();
        }
        let i = args[2].parse().expect("expect usize");
        let jobj = RoundaboutSimSetting::gen_circular(i);
        println!("{}", json::stringify(jobj));
    }
    else if args[1] == "headless" {
        sim_run(&args[2], 1e2).unwrap();
    }
    else {
        render_run(&args[1], 10.0).await;
    }
}
