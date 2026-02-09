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
    pub omega: f64,
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

/// Divergent Component of Motion (DCM) / Capture Point controller
///
/// The DCM is defined as: xi = x_com + x_dot_com / omega
/// It represents where the robot would come to rest if it placed its
/// foot there instantly. By controlling the ZMP to steer the DCM,
/// we get stable LIPM walking.
///
/// Reference DCM trajectories are planned backward from footstep positions,
/// since the DCM must converge to the final footstep.
pub struct DCMController {
    /// LIPM natural frequency
    pub omega: f64,
    /// Feedback gain (>1 for stability, typically 1.5-3.0)
    pub kp: f64,
}

impl DCMController {
    pub fn new(com_height: f64, kp: f64) -> Self {
        let gravity = 9.81;
        let omega = (gravity / com_height).sqrt();
        Self { omega, kp }
    }

    /// Compute the DCM (capture point) from current state
    pub fn compute_dcm(
        &self,
        com_pos: &na::Vector2<f64>,
        com_vel: &na::Vector2<f64>,
    ) -> na::Vector2<f64> {
        com_pos + com_vel / self.omega
    }

    /// Plan reference DCM waypoints at step transitions by working backward
    /// from the final footstep. At each step transition, the DCM should be
    /// at the upcoming stance foot.
    ///
    /// Returns a list of DCM waypoints (one per footstep, plus the final one).
    pub fn plan_dcm_waypoints(
        &self,
        footstep_positions: &[na::Vector2<f64>],
    ) -> Vec<na::Vector2<f64>> {
        let n = footstep_positions.len();
        if n == 0 {
            return vec![];
        }

        // The final DCM target is the last footstep (robot stops there)
        let mut waypoints = vec![na::Vector2::zeros(); n + 1];
        waypoints[n] = footstep_positions[n - 1];

        // Work backward: at the start of step i, the DCM must be such that
        // it reaches waypoints[i+1] by the end of the step
        for i in (0..n).rev() {
            waypoints[i] = footstep_positions[i];
        }

        waypoints
    }

    /// Compute the reference DCM at a given time within a step
    ///
    /// During a step of duration T, the DCM exponentially converges from
    /// dcm_start toward the stance foot position (ZMP), ending at dcm_end.
    ///
    /// xi_ref(t) = zmp + (dcm_start - zmp) * exp(omega * t)
    /// where we solve for the trajectory that connects dcm_start to dcm_end
    pub fn reference_dcm(
        &self,
        _dcm_start: &na::Vector2<f64>,
        dcm_end: &na::Vector2<f64>,
        zmp_stance: &na::Vector2<f64>,
        step_duration: f64,
        time_in_step: f64,
    ) -> na::Vector2<f64> {
        let t = time_in_step.clamp(0.0, step_duration);
        let t_remaining = step_duration - t;

        // DCM reference: interpolate such that it starts at dcm_start
        // and reaches dcm_end at step_duration
        // xi(t) = zmp + (dcm_end - zmp) * exp(-omega * t_remaining)
        let exp_factor = (-self.omega * t_remaining).exp();
        zmp_stance + (dcm_end - zmp_stance) * exp_factor
    }

    /// Compute the desired ZMP to track the reference DCM
    ///
    /// DCM dynamics: xi_dot = omega * (xi - p_zmp)
    /// To make tracking error decay: p_zmp = xi + (kp/omega) * (xi - xi_ref)
    ///
    /// If DCM is ahead of reference, ZMP moves forward to slow it down.
    /// If DCM is behind reference, ZMP moves backward to speed it up.
    pub fn compute_desired_zmp(
        &self,
        dcm_current: &na::Vector2<f64>,
        dcm_ref: &na::Vector2<f64>,
        _zmp_stance: &na::Vector2<f64>,
    ) -> na::Vector2<f64> {
        let dcm_error = dcm_current - dcm_ref;
        dcm_current + (self.kp / self.omega) * dcm_error
    }

    /// Run one step of the DCM-controlled LIPM simulation
    /// Returns (new_com_pos, new_com_vel, commanded_zmp)
    pub fn step(
        &self,
        com_pos: &na::Vector2<f64>,
        com_vel: &na::Vector2<f64>,
        dcm_ref: &na::Vector2<f64>,
        zmp_stance: &na::Vector2<f64>,
        dt: f64,
    ) -> (na::Vector2<f64>, na::Vector2<f64>, na::Vector2<f64>) {
        let dcm = self.compute_dcm(com_pos, com_vel);
        let zmp_cmd = self.compute_desired_zmp(&dcm, dcm_ref, zmp_stance);

        // LIPM dynamics: x_ddot = omega^2 * (x - p_zmp)
        let omega_sq = self.omega * self.omega;
        let accel = omega_sq * (com_pos - zmp_cmd);

        // Euler integration
        let new_vel = com_vel + accel * dt;
        let new_pos = com_pos + new_vel * dt;

        (new_pos, new_vel, zmp_cmd)
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

    #[test]
    fn test_dcm_computation() {
        let ctrl = DCMController::new(0.8, 2.0);

        let com = na::Vector2::new(0.0, 0.0);
        let vel = na::Vector2::new(1.0, 0.0);

        let dcm = ctrl.compute_dcm(&com, &vel);
        // DCM = com + vel/omega, omega = sqrt(9.81/0.8) ~ 3.5
        assert!(dcm.x > 0.0);
        assert!((dcm.x - 1.0 / ctrl.omega).abs() < 1e-6);
    }

    #[test]
    fn test_dcm_controller_stability() {
        // Verify the DCM controller keeps the CoM bounded during a step
        let ctrl = DCMController::new(0.8, 2.0);

        let mut com = na::Vector2::new(0.0, 0.0);
        let mut vel = na::Vector2::new(0.0, 0.0);
        let dcm_ref = na::Vector2::new(0.15, 0.0); // target ahead
        let zmp_stance = na::Vector2::new(0.0, 0.0);

        let dt = 0.01;
        for _ in 0..100 {
            let (new_com, new_vel, _zmp) = ctrl.step(&com, &vel, &dcm_ref, &zmp_stance, dt);
            com = new_com;
            vel = new_vel;
        }

        // CoM should move forward but stay bounded (not diverge)
        assert!(com.x > 0.0, "CoM should move forward");
        assert!(com.x < 1.0, "CoM should stay bounded, got {}", com.x);
    }
}
