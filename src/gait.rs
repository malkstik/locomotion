use nalgebra as na;

/// Walking phase of the gait cycle
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GaitPhase {
    /// Both feet on the ground, shifting weight to the stance foot
    DoubleSupport,
    /// Left foot is the stance foot, right foot is swinging
    LeftStance,
    /// Right foot is the stance foot, left foot is swinging
    RightStance,
}

/// Which foot is which
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Foot {
    Left,
    Right,
}

/// A planned footstep target
#[derive(Debug, Clone)]
pub struct Footstep {
    pub foot: Foot,
    pub position: na::Vector2<f64>, // [x, y] on the ground plane
}

/// Simple gait planner that generates alternating footsteps
pub struct GaitPlanner {
    /// Step length in the forward (x) direction
    pub step_length: f64,
    /// Lateral offset from centerline (half of hip width)
    pub step_width: f64,
    /// Duration of a single swing phase (seconds)
    pub swing_duration: f64,
    /// Duration of double support phase (seconds)
    pub double_support_duration: f64,
    /// Maximum foot lift height during swing
    pub swing_height: f64,
}

impl GaitPlanner {
    pub fn new(step_length: f64, step_width: f64) -> Self {
        Self {
            step_length,
            step_width,
            swing_duration: 0.4,
            double_support_duration: 0.1,
            swing_height: 0.05,
        }
    }

    /// Generate a sequence of footsteps for walking forward
    pub fn plan_footsteps(&self, num_steps: usize) -> Vec<Footstep> {
        let mut steps = Vec::new();

        for i in 0..num_steps {
            let foot = if i % 2 == 0 { Foot::Left } else { Foot::Right };
            let y_offset = match foot {
                Foot::Left => self.step_width,
                Foot::Right => -self.step_width,
            };
            // First step is a half step to get started
            let x = if i == 0 {
                self.step_length * 0.5
            } else {
                self.step_length * (i as f64 + 0.5)
            };

            steps.push(Footstep {
                foot,
                position: na::Vector2::new(x, y_offset),
            });
        }

        steps
    }

    /// Compute the swing foot trajectory at a given phase (0.0 to 1.0)
    /// Returns the foot position [x, y, z] interpolated between start and end
    pub fn swing_foot_trajectory(
        &self,
        start: &na::Vector2<f64>,
        end: &na::Vector2<f64>,
        phase: f64,
    ) -> na::Vector3<f64> {
        let phase = phase.clamp(0.0, 1.0);

        // Linear interpolation for x and y
        let x = start.x + (end.x - start.x) * phase;
        let y = start.y + (end.y - start.y) * phase;

        // Parabolic arc for z (foot lift)
        let z = self.swing_height * 4.0 * phase * (1.0 - phase);

        na::Vector3::new(x, y, z)
    }

    /// Compute the desired ZMP trajectory during a step
    /// During single support, ZMP should be at the stance foot
    /// During double support, ZMP transitions between feet
    pub fn desired_zmp(
        &self,
        stance_foot_pos: &na::Vector2<f64>,
        next_stance_pos: &na::Vector2<f64>,
        phase: GaitPhase,
        transition_progress: f64,
    ) -> na::Vector2<f64> {
        match phase {
            GaitPhase::DoubleSupport => {
                // Smoothly shift ZMP from previous stance to next stance
                let t = transition_progress.clamp(0.0, 1.0);
                stance_foot_pos.lerp(next_stance_pos, t)
            }
            GaitPhase::LeftStance | GaitPhase::RightStance => {
                // ZMP stays at the stance foot
                *stance_foot_pos
            }
        }
    }
}

/// Manages the walking state machine
pub struct WalkingStateMachine {
    pub phase: GaitPhase,
    pub phase_time: f64,
    pub step_index: usize,
    pub left_foot_pos: na::Vector2<f64>,
    pub right_foot_pos: na::Vector2<f64>,
}

impl WalkingStateMachine {
    pub fn new(hip_width: f64) -> Self {
        Self {
            phase: GaitPhase::DoubleSupport,
            phase_time: 0.0,
            step_index: 0,
            left_foot_pos: na::Vector2::new(0.0, hip_width / 2.0),
            right_foot_pos: na::Vector2::new(0.0, -hip_width / 2.0),
        }
    }

    /// Returns true if all planned steps have been completed.
    pub fn is_done(&self, footsteps: &[Footstep]) -> bool {
        self.step_index >= footsteps.len() && self.phase == GaitPhase::DoubleSupport
    }

    /// Advance the state machine by dt seconds.
    /// Returns true if we transitioned to a new phase.
    pub fn update(&mut self, dt: f64, planner: &GaitPlanner, footsteps: &[Footstep]) -> bool {
        if self.is_done(footsteps) {
            return false;
        }

        self.phase_time += dt;

        let phase_duration = match self.phase {
            GaitPhase::DoubleSupport => planner.double_support_duration,
            GaitPhase::LeftStance | GaitPhase::RightStance => planner.swing_duration,
        };

        if self.phase_time >= phase_duration {
            self.phase_time = 0.0;
            self.transition(footsteps);
            return true;
        }

        false
    }

    fn transition(&mut self, footsteps: &[Footstep]) {
        match self.phase {
            GaitPhase::DoubleSupport => {
                if self.step_index < footsteps.len() {
                    let next_step = &footsteps[self.step_index];
                    self.phase = match next_step.foot {
                        // If moving the left foot, right is stance
                        Foot::Left => GaitPhase::RightStance,
                        Foot::Right => GaitPhase::LeftStance,
                    };
                }
            }
            GaitPhase::LeftStance => {
                // Right foot just landed
                if self.step_index < footsteps.len() {
                    self.right_foot_pos = footsteps[self.step_index].position;
                    self.step_index += 1;
                }
                self.phase = GaitPhase::DoubleSupport;
            }
            GaitPhase::RightStance => {
                // Left foot just landed
                if self.step_index < footsteps.len() {
                    self.left_foot_pos = footsteps[self.step_index].position;
                    self.step_index += 1;
                }
                self.phase = GaitPhase::DoubleSupport;
            }
        }
    }

    /// Get the current stance foot position
    pub fn stance_foot_pos(&self) -> na::Vector2<f64> {
        match self.phase {
            GaitPhase::LeftStance => self.left_foot_pos,
            GaitPhase::RightStance => self.right_foot_pos,
            GaitPhase::DoubleSupport => {
                // Midpoint between feet
                (self.left_foot_pos + self.right_foot_pos) / 2.0
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_footstep_planning() {
        let planner = GaitPlanner::new(0.3, 0.15);
        let steps = planner.plan_footsteps(4);

        assert_eq!(steps.len(), 4);
        assert_eq!(steps[0].foot, Foot::Left);
        assert_eq!(steps[1].foot, Foot::Right);
        // Left foot should be at positive y, right at negative y
        assert!(steps[0].position.y > 0.0);
        assert!(steps[1].position.y < 0.0);
    }

    #[test]
    fn test_swing_trajectory_arc() {
        let planner = GaitPlanner::new(0.3, 0.15);
        let start = na::Vector2::new(0.0, 0.15);
        let end = na::Vector2::new(0.3, 0.15);

        // At start and end, z should be 0
        let p0 = planner.swing_foot_trajectory(&start, &end, 0.0);
        let p1 = planner.swing_foot_trajectory(&start, &end, 1.0);
        assert!(p0.z.abs() < 1e-10);
        assert!(p1.z.abs() < 1e-10);

        // At midpoint, z should be at max height
        let mid = planner.swing_foot_trajectory(&start, &end, 0.5);
        assert!((mid.z - planner.swing_height).abs() < 1e-10);
    }

    #[test]
    fn test_state_machine_transitions() {
        let planner = GaitPlanner::new(0.3, 0.15);
        let steps = planner.plan_footsteps(4);
        let mut sm = WalkingStateMachine::new(0.3);

        assert_eq!(sm.phase, GaitPhase::DoubleSupport);

        // Advance past double support
        sm.update(planner.double_support_duration + 0.001, &planner, &steps);
        assert!(sm.phase == GaitPhase::RightStance || sm.phase == GaitPhase::LeftStance);
    }
}
