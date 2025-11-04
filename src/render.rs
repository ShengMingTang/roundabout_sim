// SPDX-License-Identifier: GPL-3.0-or-later
use macroquad::prelude::*;
use crate::*;
use num_complex::Complex;
use std::f32::consts::PI;

pub async fn render_run(filename: &str, max_t: f32) -> Option<RoundaboutSim> {
    let mut sim = RoundaboutSim::from_json(filename)?;
    let mut finished = false;
    while (sim.t < max_t || max_t < 0.0) && finished == false {
        finished |= sim.update();
        render_update(&mut sim);
        next_frame().await
    }
    Some(sim)
}

fn render_update(sim: &RoundaboutSim) {
    // TODO: move to const
    let LANE_COLORS = vec![
        Color::new(0.78, 0.62, 0.78, 1.0), // MAUVE
        Color::new(0.0, 0.5, 0.5, 1.0), // TEAL
        Color::new(0.85, 0.65, 0.13, 1.0), // GOLDENROD
        Color::new(0.8, 0.45, 0.5, 1.0), // DUSTY_ROSE
        Color::new(0.2, 0.2, 0.2, 1.0), // CHARCOAL
    ];
    let AUX_LINE_THINKNESS: f32 = 5.0;
    let AUX_LINE_COLOR = BLACK;
    let match_action_to_color = |action: &Action| -> Color {
        match action {
            Action::Switch(diff) => {
                if *diff > 0 {
                    DARKBLUE
                } else {
                    BROWN
                }
            },
            Action::Straight => DARKGREEN,
            Action::Stop => RED,
        }
    };
    let CAR_SIZE = DIST_ALLOW * 5.0;
    // TODO: move to const

    clear_background(LIGHTGRAY);
    let setting = &sim.setting;
    let scale = if screen_height() >  screen_width() {
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
        draw_circle(0.0, 0.0,
            r * scale, LANE_COLORS[i % LANE_COLORS.len()]
        );
    }
    for i in 0..setting.n_inter {
        let pos = Complex::from_polar(setting.r_lanes[0] * scale, (i as f32) * 2.0 * PI / (setting.n_inter as f32));
        draw_line(0.0, 0.0,
            pos.re, pos.im,
            AUX_LINE_THINKNESS,
            AUX_LINE_COLOR,
        );
    }
    // draw cars
    for car in &sim.cars {
        let pos = &car.borrow().pos;
        draw_circle(pos.re * scale, -pos.im * scale, // TODO: coordinate by trial, Study...
            CAR_SIZE * scale, match_action_to_color(&car.borrow().action)
        );
    }
}