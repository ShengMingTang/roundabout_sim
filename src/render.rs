// SPDX-License-Identifier: GPL-3.0-or-later
use macroquad::prelude::*;
use crate::*;

pub async fn render_run(filename: &str, max_t: f32) -> Option<RoundaboutSim> {
    let mut sim = RoundaboutSim::from_json(filename)?;
    println!("sim init: {sim:?}");
    let mut finished = false;
    while (sim.t < max_t || max_t < 0.0) && finished == false {
        finished |= sim.update();
        render_update(&mut sim);
        next_frame().await
    }
    Some(sim)
}
fn render_update(sim: &RoundaboutSim) {
    clear_background(LIGHTGRAY);
    let scale = if screen_height() >  screen_width() {
        screen_width() / 2.0
    } else {
        screen_height() / 2.0
    };
    set_camera(&Camera2D {
        zoom: vec2(2.0 / screen_width(), 2.0 / screen_height()),
        ..Default::default()
    });
    draw_circle(0.0, 0.0, scale, YELLOW);
}