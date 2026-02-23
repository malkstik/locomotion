use crate::biped::BipedParams;
use crate::gait::{Footstep, GaitPhase, GaitPlanner, WalkingStateMachine};
use crate::ik::SimpleLegIK;
use crate::stability::DCMController;
use crate::zmp::ZmpCalculator;
use nalgebra as na;

/// Controller that bridges high-level planning (DCM, gait) with low-level joint commands
pub struct BipedController {
    // Existing physics-agnostic controllers (PRESERVED)
    pub dcm_controller: DCMController,
    pub gait_planner: GaitPlanner,
    pub state_machine: WalkingStateMachine,
    pub leg_ik: SimpleLegIK,
    pub zmp_calc: ZmpCalculator,

    // Planned trajectory
    pub footsteps: Vec<Footstep>,
    pub dcm_waypoints: Vec<na::Vector2<f64>>,

    // Controller state
    pub params: BipedParams,
    pub time: f64,

    // Internal state for closed-loop control
    com_2d: na::Vector2<f64>,
    com_vel_2d: na::Vector2<f64>,
}

impl BipedController {
    pub fn new(params: BipedParams, num_steps: usize) -> Self {
        let gait_planner = GaitPlanner::new(0.3, params.hip_width / 2.0);
        let footsteps = gait_planner.plan_footsteps(num_steps);
        let state_machine = WalkingStateMachine::new(params.hip_width);

        let dcm_controller = DCMController::new(params.leg_length, 2.0);
        let leg_ik = SimpleLegIK::new(params.leg_length / 2.0, params.leg_length / 2.0);
        let zmp_calc = ZmpCalculator::new();

        // Plan DCM waypoints from footstep positions
        let footstep_positions: Vec<na::Vector2<f64>> =
            footsteps.iter().map(|f| f.position).collect();
        let dcm_waypoints = dcm_controller.plan_dcm_waypoints(&footstep_positions);

        Self {
            dcm_controller,
            gait_planner,
            state_machine,
            leg_ik,
            zmp_calc,
            footsteps,
            dcm_waypoints,
            params,
            time: 0.0,
            com_2d: na::Vector2::zeros(),
            com_vel_2d: na::Vector2::zeros(),
        }
    }

    /// Main control update - called at 200 Hz
    /// Returns joint angle commands for MuJoCo [12 values]
    pub fn update(
        &mut self,
        actual_com: &na::Vector3<f64>,
        actual_com_vel: &na::Vector3<f64>,
        dt: f64,
    ) -> [f64; 12] {
        // 1. Update internal state from MuJoCo feedback
        self.com_2d = na::Vector2::new(actual_com.x, actual_com.y);
        self.com_vel_2d = na::Vector2::new(actual_com_vel.x, actual_com_vel.y);

        // 2. Get current gait phase and timing
        let stance_pos = self.state_machine.stance_foot_pos();
        let step_idx = self.state_machine.step_index;

        // 3. Determine phase duration
        let phase_duration = match self.state_machine.phase {
            GaitPhase::DoubleSupport => self.gait_planner.double_support_duration,
            _ => self.gait_planner.swing_duration,
        };
        let time_in_phase = self.state_machine.phase_time;

        // 4. Compute desired joint angles via IK
        // Note: DCM reference is computed in get_dcm_reference() for visualization
        let joint_commands = self.compute_joint_commands(&stance_pos, step_idx, time_in_phase, phase_duration);

        // 6. Advance state machine
        self.state_machine
            .update(dt, &self.gait_planner, &self.footsteps);
        self.time += dt;

        joint_commands
    }

    /// Compute desired joint angles using IK for both legs
    fn compute_joint_commands(
        &self,
        _stance_pos: &na::Vector2<f64>,
        step_idx: usize,
        time_in_phase: f64,
        phase_duration: f64,
    ) -> [f64; 12] {
        let mut commands = [0.0; 12];

        // Calculate progress through current phase
        let progress = if phase_duration > 0.0 {
            (time_in_phase / phase_duration).min(1.0)
        } else {
            0.0
        };

        // Compute swing foot trajectory if in single support
        let (left_foot_target, right_foot_target) = match self.state_machine.phase {
            GaitPhase::DoubleSupport => {
                // Both feet on ground - convert Vector2 to Vector3
                let left_2d = self.state_machine.left_foot_pos;
                let right_2d = self.state_machine.right_foot_pos;
                (
                    na::Vector3::new(left_2d.x, left_2d.y, 0.0),
                    na::Vector3::new(right_2d.x, right_2d.y, 0.0),
                )
            }
            GaitPhase::LeftStance => {
                // Left foot stance, right foot swinging
                let swing_start = self.state_machine.right_foot_pos;
                let swing_end = if step_idx < self.footsteps.len() {
                    self.footsteps[step_idx].position
                } else {
                    swing_start
                };
                let swing_target = self
                    .gait_planner
                    .swing_foot_trajectory(&swing_start, &swing_end, progress);

                let left_2d = self.state_machine.left_foot_pos;
                (na::Vector3::new(left_2d.x, left_2d.y, 0.0), swing_target)
            }
            GaitPhase::RightStance => {
                // Right foot stance, left foot swinging
                let swing_start = self.state_machine.left_foot_pos;
                let swing_end = if step_idx < self.footsteps.len() {
                    self.footsteps[step_idx].position
                } else {
                    swing_start
                };
                let swing_target = self
                    .gait_planner
                    .swing_foot_trajectory(&swing_start, &swing_end, progress);

                let right_2d = self.state_machine.right_foot_pos;
                (swing_target, na::Vector3::new(right_2d.x, right_2d.y, 0.0))
            }
        };

        // Simple standing pose for hip height (CoM height is 0.8m, hips at 0.8 + 0.1 = 0.9m)
        let hip_height = self.params.leg_length; // 0.8m from ground to hip

        // Solve IK for left leg (relative to left hip position)
        let left_hip_world = na::Vector3::new(self.com_2d.x, self.com_2d.y + self.params.hip_width / 2.0, hip_height);
        let left_foot_world = na::Vector3::new(left_foot_target.x, left_foot_target.y, left_foot_target.z);

        // Convert to leg frame (simple 2D for now - sagittal plane)
        let left_target_x = left_foot_world.x - left_hip_world.x;
        let left_target_y = left_hip_world.z - left_foot_world.z; // vertical distance

        if let Some(left_angles) = self.leg_ik.solve_2d(left_target_x, left_target_y) {
            commands[2] = left_angles[0]; // left_hip_pitch
            commands[3] = left_angles[1]; // left_knee_pitch
        } else {
            // Fallback: standing pose
            commands[2] = 0.0;
            commands[3] = 0.0;
        }

        // Solve IK for right leg
        let right_hip_world = na::Vector3::new(self.com_2d.x, self.com_2d.y - self.params.hip_width / 2.0, hip_height);
        let right_foot_world = na::Vector3::new(right_foot_target.x, right_foot_target.y, right_foot_target.z);

        let right_target_x = right_foot_world.x - right_hip_world.x;
        let right_target_y = right_hip_world.z - right_foot_world.z;

        if let Some(right_angles) = self.leg_ik.solve_2d(right_target_x, right_target_y) {
            commands[8] = right_angles[0]; // right_hip_pitch
            commands[9] = right_angles[1]; // right_knee_pitch
        } else {
            // Fallback: standing pose
            commands[8] = 0.0;
            commands[9] = 0.0;
        }

        // Hip yaw and roll stay at 0 for now (planar walking)
        // Ankle angles for now also 0 (could add ankle strategy later)

        commands
    }

    /// Get current DCM reference for visualization
    pub fn get_dcm_reference(&self) -> na::Vector2<f64> {
        let stance_pos = self.state_machine.stance_foot_pos();
        let step_idx = self.state_machine.step_index;
        let phase_duration = match self.state_machine.phase {
            GaitPhase::DoubleSupport => self.gait_planner.double_support_duration,
            _ => self.gait_planner.swing_duration,
        };

        if step_idx < self.dcm_waypoints.len().saturating_sub(1) {
            self.dcm_controller.reference_dcm(
                &self.dcm_waypoints[step_idx],
                &self.dcm_waypoints[step_idx + 1],
                &stance_pos,
                phase_duration,
                self.state_machine.phase_time,
            )
        } else {
            stance_pos
        }
    }

    /// Get actual DCM computed from current state
    pub fn get_actual_dcm(
        &self,
        com: &na::Vector2<f64>,
        vel: &na::Vector2<f64>,
    ) -> na::Vector2<f64> {
        self.dcm_controller.compute_dcm(com, vel)
    }

    /// Check if walking is complete
    pub fn is_done(&self) -> bool {
        self.state_machine.is_done(&self.footsteps)
    }

    /// Get current phase for status printing
    pub fn get_phase(&self) -> GaitPhase {
        self.state_machine.phase
    }

    /// Get current step index
    pub fn get_step_index(&self) -> usize {
        self.state_machine.step_index
    }
}
