// SPDX-License-Identifier: GPL-3.0-or-later
use crate::*;
use macroquad::prelude::*;
use num_complex::Complex;
use std::f32::consts::PI;

pub async fn render_run(filename: &str, max_t: f32) -> Option<RoundaboutSim> {
    let mut sim = RoundaboutSim::from_json(filename)?;
    let mut finished = false;
    while (sim.t < max_t || max_t < 0.0) && !finished {
        finished |= sim.update();
        render_update(&sim);
        next_frame().await
        // TODO: delay such that simulation time is proportional to wall clock time
    }
    Some(sim)
}

fn render_update(sim: &RoundaboutSim) {
    let lane_colors = [
        Color::new(0.78, 0.62, 0.78, 1.0), // MAUVE
        Color::new(0.0, 0.5, 0.5, 1.0),    // TEAL
        Color::new(0.85, 0.65, 0.13, 1.0), // GOLDENROD
        Color::new(0.8, 0.45, 0.5, 1.0),   // DUSTY_ROSE
        Color::new(0.2, 0.2, 0.2, 1.0),    // CHARCOAL
    ];
    let aux_line_thinkness: f32 = 5.0;
    let aux_line_color = BLACK;
    let match_action_to_color = |action: &Action| -> Color {
        match action {
            Action::Switch(diff) => {
                if *diff > 0 {
                    DARKBLUE
                } else {
                    BROWN
                }
            }
            Action::Straight => DARKGREEN,
            Action::Stop => RED,
        }
    };
    let car_size = DIST_ALLOW * 5.0;

    clear_background(LIGHTGRAY);
    let setting = &sim.setting;
    let scale = if screen_height() > screen_width() {
        screen_width() / 2.0
    } else {
        screen_height() / 2.0
    };
    set_camera(&Camera2D {
        zoom: vec2(2.0 / screen_width(), 2.0 / screen_height()),
        ..Default::default()
    });
    // draw lanes
    let scale = scale / setting.r_lanes[0]; // world -1.0 ~ 1.0
    for (i, r) in setting.r_lanes.iter().enumerate() {
        draw_circle(0.0, 0.0, r * scale, lane_colors[i % lane_colors.len()]);
    }
    for i in 0..setting.n_inter {
        let pos = Complex::from_polar(
            setting.r_lanes[0] * scale,
            (i as f32) * 2.0 * PI / (setting.n_inter as f32),
        );
        draw_line(0.0, 0.0, pos.re, pos.im, aux_line_thinkness, aux_line_color);
    }
    // draw cars
    for car in &sim.cars {
        let pos = &car.borrow().pos;
        draw_circle(
            pos.re * scale,
            -pos.im * scale, // TODO: coordinate by trial, Study...
            car_size * scale,
            match_action_to_color(&car.borrow().action),
        );
    }
}
