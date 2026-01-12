use nalgebra as na;

/// Simple analytical IK solver for a 2-link leg (hip-knee or knee-ankle)
///
/// This uses basic trigonometry for a planar 2-link chain.
/// For full 3D leg IK, you'll want to use the `k` crate with proper URDF models.
pub struct SimpleLegIK {
    /// Length of upper link (e.g., thigh)
    pub upper_length: f64,

    /// Length of lower link (e.g., shin)
    pub lower_length: f64,
}

impl SimpleLegIK {
    pub fn new(upper_length: f64, lower_length: f64) -> Self {
        Self {
            upper_length,
            lower_length,
        }
    }

    /// Solve 2D IK for a planar leg
    ///
    /// Given target position (x, y) in the leg's local frame,
    /// returns joint angles [hip/upper_joint, knee/lower_joint]
    ///
    /// Returns None if target is unreachable
    pub fn solve_2d(&self, target_x: f64, target_y: f64) -> Option<[f64; 2]> {
        let l1 = self.upper_length;
        let l2 = self.lower_length;

        // Distance to target
        let distance_sq = target_x * target_x + target_y * target_y;
        let distance = distance_sq.sqrt();

        // Check if target is reachable
        if distance > l1 + l2 || distance < (l1 - l2).abs() {
            return None;
        }

        // Law of cosines to find knee angle
        let cos_knee = (l1 * l1 + l2 * l2 - distance_sq) / (2.0 * l1 * l2);
        let knee_angle = cos_knee.acos();

        // Find hip angle
        let alpha = target_y.atan2(target_x);
        let cos_beta = (l1 * l1 + distance_sq - l2 * l2) / (2.0 * l1 * distance);
        let beta = cos_beta.acos();
        let hip_angle = alpha - beta;

        Some([hip_angle, knee_angle])
    }

    /// Forward kinematics: compute end-effector position from joint angles
    pub fn forward_kinematics(&self, joint_angles: &[f64; 2]) -> na::Vector2<f64> {
        let l1 = self.upper_length;
        let l2 = self.lower_length;
        let theta1 = joint_angles[0];
        let theta2 = joint_angles[1];

        let x = l1 * theta1.cos() + l2 * (theta1 + theta2).cos();
        let y = l1 * theta1.sin() + l2 * (theta1 + theta2).sin();

        na::Vector2::new(x, y)
    }
}

/// Jacobian-based IK solver using the `k` crate for more complex robots
///
/// This is a placeholder for integration with the `k` crate for full
/// biped kinematics with URDF models
pub struct JacobianIKSolver {
    // TODO: Integrate with k::Chain for full robot IK
}

impl JacobianIKSolver {
    pub fn new() -> Self {
        Self {}
    }

    // TODO: Implement Jacobian IK using k::JacobianIkSolver
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_ik_reachable() {
        let ik = SimpleLegIK::new(0.4, 0.4); // 40cm segments

        // Target at 45 degrees, fully extended
        let target_x = 0.565;
        let target_y = 0.565;

        let solution = ik.solve_2d(target_x, target_y);
        assert!(solution.is_some());
    }

    #[test]
    fn test_simple_ik_unreachable() {
        let ik = SimpleLegIK::new(0.4, 0.4);

        // Target too far
        let target_x = 1.0;
        let target_y = 1.0;

        let solution = ik.solve_2d(target_x, target_y);
        assert!(solution.is_none());
    }

    #[test]
    fn test_forward_kinematics() {
        let ik = SimpleLegIK::new(0.4, 0.4);

        // Both joints at 0 degrees (straight leg along x-axis)
        let pos = ik.forward_kinematics(&[0.0, 0.0]);
        assert!((pos.x - 0.8).abs() < 1e-6);
        assert!(pos.y.abs() < 1e-6);
    }
}
