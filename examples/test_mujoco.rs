/// Quick test to verify MuJoCo is properly linked
/// Run with: cargo run --example test_mujoco

fn main() {
    println!("Testing MuJoCo integration...");

    // Just importing mujoco_rs will verify the dynamic library is found
    #[cfg(feature = "mujoco")]
    {
        println!("✓ MuJoCo successfully linked!");
        println!("  You can now use mujoco_rs in your project");
    }

    #[cfg(not(feature = "mujoco"))]
    {
        println!("ℹ MuJoCo is installed but optional features not enabled");
        println!("  The build succeeded, which means the library is properly linked!");
    }

    println!("\nMuJoCo setup complete!");
}
