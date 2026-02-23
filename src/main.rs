mod biped;
mod controller;
mod gait;
mod ik;
mod mujoco_sim;
mod stability;
mod zmp;

use biped::{BipedParams, BipedState};
use controller::BipedController;
use gait::GaitPhase;
use mujoco_sim::MujocoSimulator;
use nalgebra as na;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MuJoCo Biped Locomotion Simulator");
    println!("===================================\n");

    // 1. Initialize MuJoCo
    println!("Loading MuJoCo model...");
    let mut sim = MujocoSimulator::new("models/biped_simple.xml")?;
    println!("✓ Model loaded successfully");

    // 2. Initialize controller
    let params = BipedParams::default();
    let mut controller = BipedController::new(params.clone(), 6);

    println!("\nRobot Configuration:");
    println!("  Mass: {:.0} kg", params.mass);
    println!("  Leg length: {:.2} m", params.leg_length);
    println!("  Hip width: {:.2} m", params.hip_width);
    println!("  DCM omega: {:.2} rad/s", controller.dcm_controller.omega);
    println!("  Planned footsteps: {}", controller.footsteps.len());
    println!("  Step length: {:.2} m", controller.gait_planner.step_length);
    println!();

    // 3. Simulation parameters
    let dt = sim.get_timestep();
    let max_time = 5.0;
    let mut print_interval = 0.0;

    println!("Starting simulation at {:.0} Hz (dt = {:.4}s)\n", 1.0 / dt, dt);
    println!("{:<6} {:<14} {:<24} {:<18} {:<18} {:<6}",
        "Time", "Phase", "CoM [x, y, z]", "DCM [x, y]", "ZMP [x, y]", "Contact");
    println!("{}", "-".repeat(90));

    // 4. Simulation loop
    let mut prev_com_vel = na::Vector3::zeros();

    while controller.time < max_time && !controller.is_done() {
        // a. Get current state from MuJoCo
        let com = sim.get_center_of_mass();
        let com_vel = sim.get_com_velocity();
        let joint_pos = sim.get_joint_positions();

        // b. Run controller to get joint commands
        let joint_commands = controller.update(&com, &com_vel, dt);

        // c. Send commands to MuJoCo
        sim.set_joint_targets(&joint_commands);

        // d. Step physics
        sim.step();

        // e. Compute ZMP for monitoring (doesn't affect control)
        let com_accel = (com_vel - prev_com_vel) / dt;
        prev_com_vel = com_vel;

        let state = BipedState {
            com_position: com,
            com_velocity: com_vel,
            orientation: na::UnitQuaternion::identity(),
            angular_velocity: na::Vector3::zeros(),
            joint_positions: joint_pos,
            joint_velocities: sim.get_joint_velocities(),
        };

        let actual_zmp = controller.zmp_calc.calculate_zmp(&state, &com_accel);

        // f. Check contacts
        let left_contact = sim.is_foot_in_contact("left_foot");
        let right_contact = sim.is_foot_in_contact("right_foot");
        let contact_str = match (left_contact, right_contact) {
            (true, true) => "L+R",
            (true, false) => "L  ",
            (false, true) => "  R",
            (false, false) => "---",
        };

        // g. Compute DCM for display
        let com_2d = na::Vector2::new(com.x, com.y);
        let com_vel_2d = na::Vector2::new(com_vel.x, com_vel.y);
        let dcm = controller.get_actual_dcm(&com_2d, &com_vel_2d);

        // h. Print status periodically
        if controller.time >= print_interval {
            let phase_str = match controller.get_phase() {
                GaitPhase::DoubleSupport => "DblSupport",
                GaitPhase::LeftStance => "L-Stance",
                GaitPhase::RightStance => "R-Stance",
            };

            println!(
                "{:<6.2} {:<14} [{:>6.3}, {:>6.3}, {:>5.3}]   [{:>5.2}, {:>5.2}]   [{:>5.2}, {:>5.2}]       {}",
                controller.time,
                phase_str,
                com.x, com.y, com.z,
                dcm.x, dcm.y,
                actual_zmp.x, actual_zmp.y,
                contact_str,
            );
            print_interval += 0.2;
        }
    }

    println!("\n--- Simulation Complete ---");
    println!("Final CoM: [{:.3}, {:.3}, {:.3}]",
        sim.get_center_of_mass().x,
        sim.get_center_of_mass().y,
        sim.get_center_of_mass().z);
    println!("Final CoM velocity: [{:.3}, {:.3}, {:.3}]",
        sim.get_com_velocity().x,
        sim.get_com_velocity().y,
        sim.get_com_velocity().z);
    println!("Steps completed: {}/{}", controller.get_step_index(), controller.footsteps.len());

    Ok(())
}
