use std::{
    f64::consts::PI,
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use lerp::Lerp;

pub fn floor_mod(value: f64, modulus: f64) -> f64 {
    value - (value / modulus).floor() * modulus
}

pub fn wrap_angle(angle: f64) -> f64 {
    floor_mod(angle + PI, PI * 2.0) - PI
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2f {
    pub x: f64,
    pub y: f64,
}

impl Vec2f {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn new_angle(mag: f64, angle: f64) -> Self {
        Self {
            x: mag * angle.cos(),
            y: mag * angle.sin(),
        }
    }

    pub fn length_sq(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }

    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }

    pub fn rotate(self, angle: f64) -> Self {
        let sin = angle.sin();
        let cos = angle.cos();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }

    pub fn distance_sq(self, other: Vec2f) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    pub fn closest_along_segment(self, p1: Vec2f, p2: Vec2f) -> Vec2f {
        let l2 = p1.distance_sq(p2);
        if l2 == 0.0 {
            return p1;
        }

        let t = ((self.x - p1.x) * (p2.x - p1.x) + (self.y - p1.y) * (p2.y - p1.y)) / l2;
        let t = t.clamp(0.0, 1.0);

        p1.lerp(p2, t)
    }

    pub fn segment_dist_sq(self, p1: Vec2f, p2: Vec2f) -> f64 {
        self.distance_sq(self.closest_along_segment(p1, p2))
    }

    pub fn norm(self) -> Self {
        self / self.length()
    }

    pub fn dot(self, other: Vec2f) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

impl Add for Vec2f {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Vec2f {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for Vec2f {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<f64> for Vec2f {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl Display for Vec2f {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}
