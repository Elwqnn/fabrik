//! FABRIK (Forward And Backward Reaching Inverse Kinematics) implementation.

use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

/// 2D point/vector
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline]
    pub fn distance_squared(self, other: Self) -> f32 {
        (self - other).length_squared()
    }

    #[inline]
    pub fn distance(self, other: Self) -> f32 {
        self.distance_squared(other).sqrt()
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len == 0.0 {
            Self::ZERO
        } else {
            self * (1.0 / len)
        }
    }
}

impl From<(f32, f32)> for Vec2 {
    #[inline]
    fn from((x, y): (f32, f32)) -> Self {
        Self { x, y }
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

/// Configuration for a FABRIK chain
#[derive(Debug, Clone)]
pub struct ChainConfig {
    pub segment_count: usize,
    pub segment_length: f32,
    pub tolerance: f32,
    pub max_iterations: usize,
}

impl Default for ChainConfig {
    fn default() -> Self {
        Self {
            segment_count: 8,
            segment_length: 50.0,
            tolerance: 0.5,
            max_iterations: 10,
        }
    }
}

/// A kinematic chain of joints for FABRIK IK
#[derive(Debug, Clone)]
pub struct Chain {
    pub joints: Vec<Vec2>,
    pub lengths: Vec<f32>,
    pub tolerance: f32,
    pub max_iterations: usize,
    origin: Vec2,
    total_length: f32,
}

impl Chain {
    /// Create a new chain from config, extending upward from origin
    pub fn new(origin: Vec2, config: &ChainConfig) -> Self {
        let lengths = vec![config.segment_length; config.segment_count];
        Self::with_lengths(origin, lengths, config.tolerance, config.max_iterations)
    }

    /// Create a chain with variable segment lengths
    pub fn with_lengths(
        origin: Vec2,
        lengths: Vec<f32>,
        tolerance: f32,
        max_iterations: usize,
    ) -> Self {
        let total_length = lengths.iter().sum();
        let mut joints = Vec::with_capacity(lengths.len() + 1);

        joints.push(origin);
        let mut pos = origin;
        for &len in &lengths {
            pos.y -= len;
            joints.push(pos);
        }

        Self {
            joints,
            lengths,
            tolerance,
            max_iterations,
            origin,
            total_length,
        }
    }

    /// Rebuild chain with new config (preserves origin)
    pub fn rebuild(&mut self, config: &ChainConfig) {
        *self = Self::new(self.origin, config);
    }

    /// Update origin position
    pub fn set_origin(&mut self, origin: Vec2) {
        self.origin = origin;
        self.joints[0] = origin;
    }

    /// Get origin position
    #[inline]
    pub fn origin(&self) -> Vec2 {
        self.origin
    }

    /// Total reach of the chain (cached)
    #[inline]
    pub fn total_length(&self) -> f32 {
        self.total_length
    }

    /// Number of joints
    #[inline]
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Number of segments
    #[inline]
    pub fn segment_count(&self) -> usize {
        self.lengths.len()
    }

    /// Solve IK toward target using FABRIK
    pub fn solve(&mut self, target: Vec2) {
        let base = self.joints[0];
        let dist_sq = base.distance_squared(target);
        let total_len = self.total_length;

        // If target is unreachable, stretch toward it
        if dist_sq >= total_len * total_len {
            let dir = (target - base).normalize();
            let mut pos = base;
            for (i, &len) in self.lengths.iter().enumerate() {
                pos += dir * len;
                self.joints[i + 1] = pos;
            }
            return;
        }

        let tolerance_sq = self.tolerance * self.tolerance;

        // FABRIK iterations
        for _ in 0..self.max_iterations {
            let end_effector = *self.joints.last().unwrap();
            if end_effector.distance_squared(target) < tolerance_sq {
                break;
            }

            self.forward_reach(target);
            self.backward_reach(base);
        }
    }

    /// Forward pass: move end effector to target, propagate to base
    #[inline]
    fn forward_reach(&mut self, target: Vec2) {
        let n = self.joints.len();
        self.joints[n - 1] = target;

        for i in (0..n - 1).rev() {
            let dir = (self.joints[i] - self.joints[i + 1]).normalize();
            self.joints[i] = self.joints[i + 1] + dir * self.lengths[i];
        }
    }

    /// Backward pass: anchor base, propagate to end
    #[inline]
    fn backward_reach(&mut self, base: Vec2) {
        self.joints[0] = base;

        for i in 0..self.lengths.len() {
            let dir = (self.joints[i + 1] - self.joints[i]).normalize();
            self.joints[i + 1] = self.joints[i] + dir * self.lengths[i];
        }
    }
}
