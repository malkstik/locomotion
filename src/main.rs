mod biped;
mod gait;
mod ik;
mod stability;
mod zmp;

use biped::{BipedParams, BipedState};
use gait::{GaitPhase, GaitPlanner, WalkingStateMachine};
use ik::SimpleLegIK;
use stability::DCMController;
use zmp::ZmpCalculator;

use nalgebra as na;
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::nalgebra as na_viz; // kiss3d uses nalgebra 0.30
use kiss3d::window::Window;

fn main() {
    println!("Biped Locomotion Simulator");
    println!("==========================\n");

    // --- Setup ---
    let params = BipedParams::default();
    let mut state = BipedState::new();

    let zmp_calc = ZmpCalculator::new();
    let dcm_ctrl = DCMController::new(state.com_position.z, 2.0);
    let leg_ik = SimpleLegIK::new(params.leg_length / 2.0, params.leg_length / 2.0);

    let planner = GaitPlanner::new(0.3, params.hip_width / 2.0);
    let footsteps = planner.plan_footsteps(6);
    let mut state_machine = WalkingStateMachine::new(params.hip_width);

    // Plan DCM waypoints from footstep positions
    let footstep_positions: Vec<na::Vector2<f64>> =
        footsteps.iter().map(|f| f.position).collect();
    let dcm_waypoints = dcm_ctrl.plan_dcm_waypoints(&footstep_positions);

    println!("Robot: {:.0} kg, leg length {:.1} m, hip width {:.1} m",
        params.mass, params.leg_length, params.hip_width);
    println!("Walking with DCM control (omega = {:.2} rad/s)\n",
        dcm_ctrl.omega);
    println!("Planned {} footsteps (step length: {:.1} m)\n",
        footsteps.len(), planner.step_length);

    // --- Visualization Setup ---
    let mut window = Window::new("Biped Locomotion Visualization");
    window.set_light(Light::StickToCamera);

    // Setup camera
    let eye = na_viz::Point3::new(2.0f32, 2.0f32, 1.5f32);
    let at = na_viz::Point3::new(0.5f32, 0.0f32, 0.0f32);
    let mut camera = ArcBall::new(eye, at);

    // Create ground plane
    let mut ground = window.add_quad(4.0, 4.0, 1, 1);
    ground.set_color(0.3, 0.5, 0.3);
    ground.append_rotation(&na_viz::UnitQuaternion::from_axis_angle(
        &na_viz::Vector3::x_axis(),
        std::f32::consts::PI / 2.0,
    ));

    // Create scene objects
    let mut com_sphere = window.add_sphere(0.05);
    com_sphere.set_color(1.0, 0.0, 0.0);

    let mut left_foot = window.add_cube(0.05, 0.12, 0.02);
    left_foot.set_color(0.2, 0.2, 0.8);

    let mut right_foot = window.add_cube(0.05, 0.12, 0.02);
    right_foot.set_color(0.8, 0.2, 0.2);

    let mut zmp_marker = window.add_sphere(0.025);
    zmp_marker.set_color(1.0, 1.0, 0.0);

    let mut dcm_marker = window.add_sphere(0.025);
    dcm_marker.set_color(0.0, 1.0, 1.0);

    let mut dcm_ref_marker = window.add_sphere(0.02);
    dcm_ref_marker.set_color(0.0, 0.7, 0.7);

    // --- Simulation loop ---
    let dt = 0.005; // 200 Hz for better integration accuracy
    let max_time = 5.0;
    let mut t = 0.0;
    let mut print_interval = 0.0;

    let mut com_2d = na::Vector2::new(state.com_position.x, state.com_position.y);
    let mut com_vel_2d = na::Vector2::new(state.com_velocity.x, state.com_velocity.y);

    println!("{:<6} {:<14} {:<24} {:<18} {:<18} {:<6}",
        "Time", "Phase", "CoM [x, y, z]", "DCM [x, y]", "ZMP [x, y]", "Stable");
    println!("{}", "-".repeat(90));

    while t < max_time && !state_machine.is_done(&footsteps) && window.render_with_camera(&mut camera) {
        // 1. Determine gait timing
        let phase_duration = match state_machine.phase {
            GaitPhase::DoubleSupport => planner.double_support_duration,
            _ => planner.swing_duration,
        };
        let time_in_phase = state_machine.phase_time;

        // 2. Get stance foot and DCM reference for current step
        let stance_pos = state_machine.stance_foot_pos();
        let step_idx = state_machine.step_index;

        let dcm_ref = if step_idx < dcm_waypoints.len().saturating_sub(1) {
            let dcm_end = dcm_waypoints[step_idx + 1];
            dcm_ctrl.reference_dcm(
                &dcm_waypoints[step_idx],
                &dcm_end,
                &stance_pos,
                phase_duration,
                time_in_phase,
            )
        } else {
            // Final step: DCM converges to stance
            stance_pos
        };

        // 3. DCM-controlled LIPM step
        let (new_com, new_vel, zmp_cmd) =
            dcm_ctrl.step(&com_2d, &com_vel_2d, &dcm_ref, &stance_pos, dt);

        com_2d = new_com;
        com_vel_2d = new_vel;

        // Update full 3D state
        state.com_position.x = com_2d.x;
        state.com_position.y = com_2d.y;
        state.com_velocity.x = com_vel_2d.x;
        state.com_velocity.y = com_vel_2d.y;

        // 4. Compute actual ZMP and check stability
        let com_accel_3d = na::Vector3::new(
            dcm_ctrl.omega * dcm_ctrl.omega * (com_2d.x - zmp_cmd.x),
            dcm_ctrl.omega * dcm_ctrl.omega * (com_2d.y - zmp_cmd.y),
            0.0,
        );
        let actual_zmp = zmp_calc.calculate_zmp(&state, &com_accel_3d);
        let support = zmp_calc.get_foot_support_polygon(&stance_pos, &params.foot_size);
        let stable = zmp_calc.is_stable(&actual_zmp, &support);

        // 5. Compute DCM for display
        let dcm = dcm_ctrl.compute_dcm(&com_2d, &com_vel_2d);

        // 6. Solve IK for the swing leg
        let progress = if phase_duration > 0.0 {
            time_in_phase / phase_duration
        } else {
            0.0
        };

        if state_machine.phase == GaitPhase::LeftStance
            || state_machine.phase == GaitPhase::RightStance
        {
            let swing_start = match state_machine.phase {
                GaitPhase::LeftStance => state_machine.right_foot_pos,
                _ => state_machine.left_foot_pos,
            };
            let swing_end = if step_idx < footsteps.len() {
                footsteps[step_idx].position
            } else {
                swing_start
            };

            let swing_pos = planner.swing_foot_trajectory(&swing_start, &swing_end, progress);

            let hip_x = state.com_position.x;
            let hip_z = state.com_position.z;
            let target_x = swing_pos.x - hip_x;
            let target_y = -(hip_z - swing_pos.z);

            if let Some(angles) = leg_ik.solve_2d(target_x, target_y) {
                let offset = match state_machine.phase {
                    GaitPhase::LeftStance => 6,
                    _ => 0,
                };
                state.joint_positions[offset] = angles[0];
                state.joint_positions[offset + 1] = angles[1];
            }
        }

        // 7. Update visualization
        com_sphere.set_local_translation(na_viz::Translation3::new(
            state.com_position.x as f32,
            state.com_position.y as f32,
            state.com_position.z as f32,
        ));

        left_foot.set_local_translation(na_viz::Translation3::new(
            state_machine.left_foot_pos.x as f32,
            state_machine.left_foot_pos.y as f32,
            0.01,
        ));

        right_foot.set_local_translation(na_viz::Translation3::new(
            state_machine.right_foot_pos.x as f32,
            state_machine.right_foot_pos.y as f32,
            0.01,
        ));

        zmp_marker.set_local_translation(na_viz::Translation3::new(
            actual_zmp.x as f32,
            actual_zmp.y as f32,
            0.005,
        ));

        dcm_marker.set_local_translation(na_viz::Translation3::new(
            dcm.x as f32,
            dcm.y as f32,
            0.005,
        ));

        dcm_ref_marker.set_local_translation(na_viz::Translation3::new(
            dcm_ref.x as f32,
            dcm_ref.y as f32,
            0.005,
        ));

        // Draw lines from CoM to feet (simple leg visualization)
        window.draw_line(
            &na_viz::Point3::new(state.com_position.x as f32, state.com_position.y as f32, state.com_position.z as f32),
            &na_viz::Point3::new(state_machine.left_foot_pos.x as f32, state_machine.left_foot_pos.y as f32, 0.01),
            &na_viz::Point3::new(0.2, 0.2, 0.8),
        );
        window.draw_line(
            &na_viz::Point3::new(state.com_position.x as f32, state.com_position.y as f32, state.com_position.z as f32),
            &na_viz::Point3::new(state_machine.right_foot_pos.x as f32, state_machine.right_foot_pos.y as f32, 0.01),
            &na_viz::Point3::new(0.8, 0.2, 0.2),
        );

        // 8. Print state periodically
        if t >= print_interval {
            let phase_str = match state_machine.phase {
                GaitPhase::DoubleSupport => "DblSupport",
                GaitPhase::LeftStance => "L-Stance",
                GaitPhase::RightStance => "R-Stance",
            };

            println!(
                "{:<6.2} {:<14} [{:>6.3}, {:>6.3}, {:>5.3}]   [{:>5.2}, {:>5.2}]   [{:>5.2}, {:>5.2}]       {}",
                t, phase_str,
                state.com_position.x, state.com_position.y, state.com_position.z,
                dcm.x, dcm.y,
                actual_zmp.x, actual_zmp.y,
                if stable { "OK" } else { "!!" },
            );
            print_interval += 0.2;
        }

        // 9. Advance gait state machine
        let transitioned = state_machine.update(dt, &planner, &footsteps);
        if transitioned && state_machine.step_index <= footsteps.len() {
            let phase_str = match state_machine.phase {
                GaitPhase::DoubleSupport => "-> Double Support",
                GaitPhase::LeftStance => "-> Left Stance (right foot swinging)",
                GaitPhase::RightStance => "-> Right Stance (left foot swinging)",
            };
            println!("       {}", phase_str);
        }

        t += dt;
    }

    println!("\n--- Simulation Complete (window closed) ---");
    println!("Final CoM: [{:.3}, {:.3}, {:.3}]",
        state.com_position.x, state.com_position.y, state.com_position.z);
    println!("Final CoM velocity: [{:.3}, {:.3}]",
        state.com_velocity.x, state.com_velocity.y);
    println!("Steps completed: {}/{}", state_machine.step_index, footsteps.len());
}
