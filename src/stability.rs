use nalgebra as na;
use crate::biped::BipedState;
use crate::zmp::ZmpCalculator;

/// Simple PD controller for maintaining stability
///
/// This controller generates desired CoM accelerations to keep
/// the ZMP within the support polygon
pub struct StabilityController {
    /// Proportional gain
    pub kp: f64,

    /// Derivative gain
    pub kd: f64,

    /// ZMP calculator
    zmp_calc: ZmpCalculator,
}

impl StabilityController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            zmp_calc: ZmpCalculator::new(),
        }
    }

    /// Compute desired CoM acceleration to stabilize the robot
    ///
    /// This uses a simple PD control law to drive the ZMP towards
    /// the center of the support polygon
    pub fn compute_com_acceleration(
        &self,
        state: &BipedState,
        desired_zmp: &na::Vector2<f64>,
        current_com_accel: &na::Vector3<f64>,
    ) -> na::Vector3<f64> {
        // Current ZMP
        let current_zmp = self.zmp_calc.calculate_zmp(state, current_com_accel);

        // ZMP error
        let zmp_error = desired_zmp - current_zmp;

        // ZMP velocity (approximated from CoM velocity projected to ground)
        let zmp_velocity = na::Vector2::new(
            state.com_velocity.x,
            state.com_velocity.y,
        );

        // PD control law
        // The relationship between CoM acceleration and ZMP is:
        // x_zmp = x_com - (z_com / g) * x_com_ddot
        // Rearranging: x_com_ddot = (x_com - x_zmp) * g / z_com

        let gravity = 9.81;
        let z_com = state.com_position.z;

        let control_x = self.kp * zmp_error.x - self.kd * zmp_velocity.x;
        let control_y = self.kp * zmp_error.y - self.kd * zmp_velocity.y;

        // Convert control signal to desired CoM acceleration
        let desired_accel_x = control_x * gravity / z_com;
        let desired_accel_y = control_y * gravity / z_com;

        na::Vector3::new(desired_accel_x, desired_accel_y, 0.0)
    }
}

/// Linear Inverted Pendulum Model (LIPM) controller
///
/// The LIPM is a common simplification for biped locomotion that
/// treats the robot as an inverted pendulum with constant height
pub struct LIPMController {
    /// Natural frequency of the pendulum (omega = sqrt(g/z))
    omega: f64,
}

impl LIPMController {
    pub fn new(com_height: f64) -> Self {
        let gravity = 9.81;
        let omega = (gravity / com_height).sqrt();

        Self { omega }
    }

    /// Compute required CoM acceleration for LIPM dynamics
    pub fn compute_lipm_acceleration(
        &self,
        com_position: &na::Vector2<f64>,
        zmp_position: &na::Vector2<f64>,
    ) -> na::Vector2<f64> {
        // LIPM dynamics: x_ddot = omega^2 * (x - x_zmp)
        let omega_sq = self.omega * self.omega;

        na::Vector2::new(
            omega_sq * (com_position.x - zmp_position.x),
            omega_sq * (com_position.y - zmp_position.y),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lipm_controller() {
        let controller = LIPMController::new(0.8); // 80cm height

        let com = na::Vector2::new(0.0, 0.0);
        let zmp = na::Vector2::new(0.1, 0.0); // ZMP ahead of CoM

        let accel = controller.compute_lipm_acceleration(&com, &zmp);

        // Should accelerate backward (negative) to bring ZMP back
        assert!(accel.x < 0.0);
    }
}
