use nalgebra as na;
use std::collections::HashMap;
use std::ffi::CString;
use thiserror::Error;
use mujoco_rs::mujoco_c::mjtObj;

#[derive(Error, Debug)]
pub enum MujocoError {
    #[error("Failed to load model: {0}")]
    ModelLoadError(String),
    #[error("Body not found: {0}")]
    BodyNotFound(String),
    #[error("Joint not found: {0}")]
    JointNotFound(String),
    #[error("Actuator not found: {0}")]
    ActuatorNotFound(String),
    #[error("Geom not found: {0}")]
    GeomNotFound(String),
}

pub type Result<T> = std::result::Result<T, MujocoError>;

/// Wrapper around MuJoCo model and simulation state
pub struct MujocoSimulator {
    model: *mut mujoco_rs::mujoco_c::mjModel,
    data: *mut mujoco_rs::mujoco_c::mjData,

    // Name -> index mappings for fast lookup
    joint_indices: HashMap<String, usize>,
    actuator_indices: HashMap<String, usize>,
    body_indices: HashMap<String, usize>,
    geom_indices: HashMap<String, usize>,

    // Simulation config
    timestep: f64,
}

impl MujocoSimulator {
    /// Load model from XML file
    pub fn new(model_path: &str) -> Result<Self> {
        unsafe {
            // Load model from XML
            let path_cstr = CString::new(model_path)
                .map_err(|e| MujocoError::ModelLoadError(format!("Invalid path: {}", e)))?;

            let mut error = [0i8; 1000];
            let model = mujoco_rs::mujoco_c::mj_loadXML(
                path_cstr.as_ptr(),
                std::ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            );

            if model.is_null() {
                let error_bytes: Vec<u8> = error
                    .iter()
                    .take_while(|&&c| c != 0)
                    .map(|&c| c as u8)
                    .collect();
                let error_str = String::from_utf8_lossy(&error_bytes);
                return Err(MujocoError::ModelLoadError(error_str.to_string()));
            }

            // Create data structure
            let data = mujoco_rs::mujoco_c::mj_makeData(model);
            if data.is_null() {
                mujoco_rs::mujoco_c::mj_deleteModel(model);
                return Err(MujocoError::ModelLoadError(
                    "Failed to create data".to_string(),
                ));
            }

            let timestep = (*model).opt.timestep;

            // Build lookup tables
            let mut joint_indices = HashMap::new();
            let njnt = (*model).njnt as usize;
            for i in 0..njnt {
                let name_ptr = mujoco_rs::mujoco_c::mj_id2name(
                    model,
                    mjtObj::mjOBJ_JOINT as i32,
                    i as i32,
                );
                if !name_ptr.is_null() {
                    let name = std::ffi::CStr::from_ptr(name_ptr)
                        .to_string_lossy()
                        .into_owned();
                    joint_indices.insert(name, i);
                }
            }

            let mut actuator_indices = HashMap::new();
            let nu = (*model).nu as usize;
            for i in 0..nu {
                let name_ptr = mujoco_rs::mujoco_c::mj_id2name(
                    model,
                    mjtObj::mjOBJ_ACTUATOR as i32,
                    i as i32,
                );
                if !name_ptr.is_null() {
                    let name = std::ffi::CStr::from_ptr(name_ptr)
                        .to_string_lossy()
                        .into_owned();
                    actuator_indices.insert(name, i);
                }
            }

            let mut body_indices = HashMap::new();
            let nbody = (*model).nbody as usize;
            for i in 0..nbody {
                let name_ptr = mujoco_rs::mujoco_c::mj_id2name(
                    model,
                    mjtObj::mjOBJ_BODY as i32,
                    i as i32,
                );
                if !name_ptr.is_null() {
                    let name = std::ffi::CStr::from_ptr(name_ptr)
                        .to_string_lossy()
                        .into_owned();
                    body_indices.insert(name, i);
                }
            }

            let mut geom_indices = HashMap::new();
            let ngeom = (*model).ngeom as usize;
            for i in 0..ngeom {
                let name_ptr = mujoco_rs::mujoco_c::mj_id2name(
                    model,
                    mjtObj::mjOBJ_GEOM as i32,
                    i as i32,
                );
                if !name_ptr.is_null() {
                    let name = std::ffi::CStr::from_ptr(name_ptr)
                        .to_string_lossy()
                        .into_owned();
                    geom_indices.insert(name, i);
                }
            }

            Ok(Self {
                model,
                data,
                joint_indices,
                actuator_indices,
                body_indices,
                geom_indices,
                timestep,
            })
        }
    }

    /// Step the physics forward by one timestep
    pub fn step(&mut self) {
        unsafe {
            mujoco_rs::mujoco_c::mj_step(self.model, self.data);
        }
    }

    /// Set joint position targets for position-controlled actuators
    pub fn set_joint_targets(&mut self, joint_angles: &[f64]) {
        assert_eq!(joint_angles.len(), 12, "Expected 12 joint angles");

        let actuator_names = [
            "left_hip_yaw_act",
            "left_hip_roll_act",
            "left_hip_pitch_act",
            "left_knee_pitch_act",
            "left_ankle_pitch_act",
            "left_ankle_roll_act",
            "right_hip_yaw_act",
            "right_hip_roll_act",
            "right_hip_pitch_act",
            "right_knee_pitch_act",
            "right_ankle_pitch_act",
            "right_ankle_roll_act",
        ];

        unsafe {
            let ctrl_slice =
                std::slice::from_raw_parts_mut((*self.data).ctrl, (*self.model).nu as usize);

            for (i, name) in actuator_names.iter().enumerate() {
                if let Some(&actuator_id) = self.actuator_indices.get(*name) {
                    ctrl_slice[actuator_id] = joint_angles[i];
                }
            }
        }
    }

    /// Get current joint positions
    pub fn get_joint_positions(&self) -> Vec<f64> {
        let joint_names = [
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee_pitch",
            "left_ankle_pitch",
            "left_ankle_roll",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee_pitch",
            "right_ankle_pitch",
            "right_ankle_roll",
        ];

        unsafe {
            let qpos_slice =
                std::slice::from_raw_parts((*self.data).qpos, (*self.model).nq as usize);
            let jnt_qposadr_slice =
                std::slice::from_raw_parts((*self.model).jnt_qposadr, (*self.model).njnt as usize);

            joint_names
                .iter()
                .map(|name| {
                    if let Some(&joint_id) = self.joint_indices.get(*name) {
                        let qpos_adr = jnt_qposadr_slice[joint_id] as usize;
                        qpos_slice[qpos_adr]
                    } else {
                        0.0
                    }
                })
                .collect()
        }
    }

    /// Get current joint velocities
    pub fn get_joint_velocities(&self) -> Vec<f64> {
        let joint_names = [
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee_pitch",
            "left_ankle_pitch",
            "left_ankle_roll",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee_pitch",
            "right_ankle_pitch",
            "right_ankle_roll",
        ];

        unsafe {
            let qvel_slice =
                std::slice::from_raw_parts((*self.data).qvel, (*self.model).nv as usize);
            let jnt_dofadr_slice =
                std::slice::from_raw_parts((*self.model).jnt_dofadr, (*self.model).njnt as usize);

            joint_names
                .iter()
                .map(|name| {
                    if let Some(&joint_id) = self.joint_indices.get(*name) {
                        let qvel_adr = jnt_dofadr_slice[joint_id] as usize;
                        qvel_slice[qvel_adr]
                    } else {
                        0.0
                    }
                })
                .collect()
        }
    }

    /// Get body position and orientation
    pub fn get_body_pose(
        &self,
        body_name: &str,
    ) -> Result<(na::Vector3<f64>, na::UnitQuaternion<f64>)> {
        let body_id = self
            .body_indices
            .get(body_name)
            .ok_or_else(|| MujocoError::BodyNotFound(body_name.to_string()))?;

        unsafe {
            let xpos_slice =
                std::slice::from_raw_parts((*self.data).xpos, (*self.model).nbody as usize * 3);
            let xquat_slice =
                std::slice::from_raw_parts((*self.data).xquat, (*self.model).nbody as usize * 4);

            let pos_start = *body_id * 3;
            let position = na::Vector3::new(
                xpos_slice[pos_start],
                xpos_slice[pos_start + 1],
                xpos_slice[pos_start + 2],
            );

            let quat_start = *body_id * 4;
            let orientation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                xquat_slice[quat_start],
                xquat_slice[quat_start + 1],
                xquat_slice[quat_start + 2],
                xquat_slice[quat_start + 3],
            ));

            Ok((position, orientation))
        }
    }

    /// Get body velocity (linear and angular)
    pub fn get_body_velocity(
        &self,
        body_name: &str,
    ) -> Result<(na::Vector3<f64>, na::Vector3<f64>)> {
        let body_id = self
            .body_indices
            .get(body_name)
            .ok_or_else(|| MujocoError::BodyNotFound(body_name.to_string()))?;

        unsafe {
            let cvel_slice =
                std::slice::from_raw_parts((*self.data).cvel, (*self.model).nbody as usize * 6);

            let vel_start = *body_id * 6;
            let angular_vel = na::Vector3::new(
                cvel_slice[vel_start],
                cvel_slice[vel_start + 1],
                cvel_slice[vel_start + 2],
            );

            let linear_vel = na::Vector3::new(
                cvel_slice[vel_start + 3],
                cvel_slice[vel_start + 4],
                cvel_slice[vel_start + 5],
            );

            Ok((linear_vel, angular_vel))
        }
    }

    /// Compute robot's center of mass
    pub fn get_center_of_mass(&self) -> na::Vector3<f64> {
        unsafe {
            if let Some(&torso_id) = self.body_indices.get("torso") {
                let subtree_com_slice = std::slice::from_raw_parts(
                    (*self.data).subtree_com,
                    (*self.model).nbody as usize * 3,
                );

                let com_start = torso_id * 3;
                na::Vector3::new(
                    subtree_com_slice[com_start],
                    subtree_com_slice[com_start + 1],
                    subtree_com_slice[com_start + 2],
                )
            } else {
                // Fallback: compute weighted average
                let xpos_slice =
                    std::slice::from_raw_parts((*self.data).xpos, (*self.model).nbody as usize * 3);
                let body_mass_slice =
                    std::slice::from_raw_parts((*self.model).body_mass, (*self.model).nbody as usize);

                let mut total_mass = 0.0;
                let mut com = na::Vector3::zeros();

                for (name, &body_id) in &self.body_indices {
                    if name != "world" {
                        let mass = body_mass_slice[body_id];
                        let pos_start = body_id * 3;
                        let pos = na::Vector3::new(
                            xpos_slice[pos_start],
                            xpos_slice[pos_start + 1],
                            xpos_slice[pos_start + 2],
                        );
                        com += mass * pos;
                        total_mass += mass;
                    }
                }

                if total_mass > 0.0 {
                    com / total_mass
                } else {
                    na::Vector3::zeros()
                }
            }
        }
    }

    /// Get CoM velocity
    pub fn get_com_velocity(&self) -> na::Vector3<f64> {
        unsafe {
            let cvel_slice =
                std::slice::from_raw_parts((*self.data).cvel, (*self.model).nbody as usize * 6);
            let body_mass_slice =
                std::slice::from_raw_parts((*self.model).body_mass, (*self.model).nbody as usize);

            let mut total_mass = 0.0;
            let mut com_vel = na::Vector3::zeros();

            for (name, &body_id) in &self.body_indices {
                if name != "world" {
                    let mass = body_mass_slice[body_id];
                    let vel_start = body_id * 6;

                    let vel = na::Vector3::new(
                        cvel_slice[vel_start + 3],
                        cvel_slice[vel_start + 4],
                        cvel_slice[vel_start + 5],
                    );

                    com_vel += mass * vel;
                    total_mass += mass;
                }
            }

            if total_mass > 0.0 {
                com_vel / total_mass
            } else {
                na::Vector3::zeros()
            }
        }
    }

    /// Check if a foot is in contact with ground
    pub fn is_foot_in_contact(&self, foot_name: &str) -> bool {
        if let Some(&geom_id) = self.geom_indices.get(foot_name) {
            unsafe {
                let ncon = (*self.data).ncon as usize;
                let contact_slice = std::slice::from_raw_parts((*self.data).contact, ncon);

                for i in 0..ncon {
                    let contact = &contact_slice[i];
                    if contact.geom1 == geom_id as i32 || contact.geom2 == geom_id as i32 {
                        // Check if normal force is above threshold
                        let force_slice = std::slice::from_raw_parts(contact.frame.as_ptr(), 9);
                        if force_slice[0] > 0.1 {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Get total ground reaction force
    pub fn get_ground_reaction_force(&self) -> na::Vector3<f64> {
        let mut total_force = na::Vector3::zeros();

        let foot_geoms = ["left_foot", "right_foot"];

        unsafe {
            let ncon = (*self.data).ncon as usize;
            let contact_slice = std::slice::from_raw_parts((*self.data).contact, ncon);

            for i in 0..ncon {
                let contact = &contact_slice[i];

                let is_foot_contact = foot_geoms.iter().any(|foot_name| {
                    if let Some(&geom_id) = self.geom_indices.get(*foot_name) {
                        contact.geom1 == geom_id as i32 || contact.geom2 == geom_id as i32
                    } else {
                        false
                    }
                });

                if is_foot_contact {
                    let frame_slice = std::slice::from_raw_parts(contact.frame.as_ptr(), 9);
                    let normal_force = frame_slice[0];
                    let normal = na::Vector3::new(frame_slice[2], frame_slice[5], frame_slice[8]);
                    total_force += normal_force * normal;
                }
            }
        }

        total_force
    }

    pub fn get_timestep(&self) -> f64 {
        self.timestep
    }
}

impl Drop for MujocoSimulator {
    fn drop(&mut self) {
        unsafe {
            if !self.data.is_null() {
                mujoco_rs::mujoco_c::mj_deleteData(self.data);
            }
            if !self.model.is_null() {
                mujoco_rs::mujoco_c::mj_deleteModel(self.model);
            }
        }
    }
}

// Mark as Send/Sync with caution - MuJoCo doesn't have thread safety guarantees
unsafe impl Send for MujocoSimulator {}
unsafe impl Sync for MujocoSimulator {}
