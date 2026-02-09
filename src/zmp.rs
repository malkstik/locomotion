use nalgebra as na;
use crate::biped::BipedState;

/// Zero Moment Point calculator
///
/// The ZMP is a key stability criterion for biped locomotion.
/// For static stability, the ZMP must remain inside the support polygon.
pub struct ZmpCalculator {
    gravity: f64,
}

impl ZmpCalculator {
    pub fn new() -> Self {
        Self {
            gravity: 9.81, // m/s^2
        }
    }

    /// Calculate the Zero Moment Point given the current robot state
    ///
    /// Returns the ZMP position in the ground plane [x, y]
    ///
    /// ZMP equation (simplified):
    /// x_zmp = x_com - (z_com / g) * x_com_ddot
    /// y_zmp = y_com - (z_com / g) * y_com_ddot
    pub fn calculate_zmp(
        &self,
        state: &BipedState,
        com_acceleration: &na::Vector3<f64>,
    ) -> na::Vector2<f64> {
        let z_com = state.com_position.z;

        // ZMP calculation
        let x_zmp = state.com_position.x - (z_com / self.gravity) * com_acceleration.x;
        let y_zmp = state.com_position.y - (z_com / self.gravity) * com_acceleration.y;

        na::Vector2::new(x_zmp, y_zmp)
    }

    /// Check if ZMP is within the support polygon
    ///
    /// For a single support phase (one foot on ground), the support polygon
    /// is defined by the foot boundaries
    pub fn is_stable(
        &self,
        zmp: &na::Vector2<f64>,
        support_polygon: &[na::Vector2<f64>],
    ) -> bool {
        // Simple implementation: check if ZMP is inside a rectangular support area
        // TODO: Implement proper point-in-polygon test for arbitrary convex polygons

        if support_polygon.len() < 3 {
            return false;
        }

        // For now, use axis-aligned bounding box
        let min_x = support_polygon.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
        let max_x = support_polygon.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
        let min_y = support_polygon.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
        let max_y = support_polygon.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);

        zmp.x >= min_x && zmp.x <= max_x && zmp.y >= min_y && zmp.y <= max_y
    }

    /// Get the support polygon for a foot at the given position
    pub fn get_foot_support_polygon(
        &self,
        foot_center: &na::Vector2<f64>,
        foot_size: &na::Vector2<f64>,
    ) -> Vec<na::Vector2<f64>> {
        // Return the four corners of the rectangular foot
        let half_length = foot_size.x / 2.0;
        let half_width = foot_size.y / 2.0;

        vec![
            na::Vector2::new(foot_center.x - half_length, foot_center.y - half_width),
            na::Vector2::new(foot_center.x + half_length, foot_center.y - half_width),
            na::Vector2::new(foot_center.x + half_length, foot_center.y + half_width),
            na::Vector2::new(foot_center.x - half_length, foot_center.y + half_width),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zmp_calculation() {
        let calc = ZmpCalculator::new();
        let mut state = BipedState::new();
        state.com_position = na::Vector3::new(0.0, 0.0, 0.8);

        let acceleration = na::Vector3::new(1.0, 0.0, 0.0);
        let zmp = calc.calculate_zmp(&state, &acceleration);

        // With forward acceleration, ZMP should be behind CoM
        assert!(zmp.x < 0.0);
    }
}
