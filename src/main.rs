// SPDX-License-Identifier: GPL-3.0-or-later
use macroquad::prelude::*;
use roundabout_sim::*;
use std::env;

fn help() {
    println!("     : cargo run --bin roundabout_sim -- <path_to_json>");
}

#[macroquad::main("Roundabout")]
async fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 || args[1] == "help" {
        help();
    } else if args[1] == "gen_random" {
        if args.len() < 5 {
            help();
        }
        let n_cars = args[2].parse::<usize>().expect("expect usize");
        let n_dst = args[3].parse::<usize>().expect("expect usize");
        let mut r_lanes = vec![];
        for arg in args.iter().skip(4) {
            r_lanes.push(arg.parse::<f32>().expect("expect f32"));
        }
        let jobj = RoundaboutSimSetting::gen_random(n_cars, n_dst, &r_lanes);
        println!("{}", json::stringify(jobj));
    } else if args[1] == "gen_circular" {
        if args.len() < 3 {
            help();
        }
        let i = args[2].parse().expect("expect usize");
        let jobj = RoundaboutSimSetting::gen_circular(i);
        println!("{}", json::stringify(jobj));
    } else {
        render_run(&args[1], -1.0).await;
    }
}
