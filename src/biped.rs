use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Represents the state of a biped robot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BipedState {
    /// Center of Mass position [x, y, z]
    pub com_position: na::Vector3<f64>,

    /// Center of Mass velocity [vx, vy, vz]
    pub com_velocity: na::Vector3<f64>,

    /// Base orientation (quaternion)
    pub orientation: na::UnitQuaternion<f64>,

    /// Angular velocity
    pub angular_velocity: na::Vector3<f64>,

    /// Joint positions (hip, knee, ankle for each leg)
    pub joint_positions: Vec<f64>,

    /// Joint velocities
    pub joint_velocities: Vec<f64>,
}

impl BipedState {
    pub fn new() -> Self {
        Self {
            com_position: na::Vector3::new(0.0, 0.0, 0.8), // Standing height
            com_velocity: na::Vector3::zeros(),
            orientation: na::UnitQuaternion::identity(),
            angular_velocity: na::Vector3::zeros(),
            joint_positions: vec![0.0; 12], // 6 joints per leg
            joint_velocities: vec![0.0; 12],
        }
    }
}

/// Physical parameters of the biped
#[derive(Debug, Clone)]
pub struct BipedParams {
    /// Total mass (kg)
    pub mass: f64,

    /// Leg length (m)
    pub leg_length: f64,

    /// Foot dimensions [length, width] (m)
    pub foot_size: na::Vector2<f64>,

    /// Hip width (m)
    pub hip_width: f64,
}

impl Default for BipedParams {
    fn default() -> Self {
        Self {
            mass: 60.0,           // 60 kg robot
            leg_length: 0.8,      // 80 cm legs
            foot_size: na::Vector2::new(0.2, 0.1), // 20cm x 10cm feet
            hip_width: 0.3,       // 30 cm hip width
        }
    }
}
