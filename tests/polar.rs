// SPDX-License-Identifier: GPL-3.0-or-later
use num_complex::Complex;
use std::f32::consts::PI;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn polar_pi() {
        let p = Complex::from_polar(1.0, -1.0 * PI * 0.75);
        let p = p.to_polar();
        assert!(p.1 <= PI && p.1 >= -PI, "should in [-pi, pi]");
        let p = Complex::from_polar(1.0, 2.0 * PI * 0.9);
        let p = p.to_polar();
        assert!(p.1 <= PI && p.1 >= -PI, "should in [-pi, pi]");
    }
}