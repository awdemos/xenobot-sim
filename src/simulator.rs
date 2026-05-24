use crate::types::*;
use oxiphysics_core::math::Vec3 as OxVec3;
use oxiphysics_softbody::{
    SoftBody, SoftParticle, XpbdSolver,
    DistanceConstraint,
    constraint::SoftConstraint,
    xpbd::XpbdSelfCollision,
};
use oxiphysics_io::vtk_writer::ParticleVtkExporter;

pub struct SimulationState {
    pub body: SoftBody,
    pub constraints: Vec<Box<dyn SoftConstraint>>,
    pub material_map: Vec<Material>,
    pub solver: XpbdSolver,
    pub time: Real,
    pub self_collision: Option<XpbdSelfCollision>,
}

pub struct SimulatorConfig {
    pub dt: Real,
    pub substeps: usize,
    pub iterations: usize,
    pub gravity: Vec3,
    pub enable_self_collision: bool,
    pub collision_radius: Real,
    pub collision_compliance: Real,
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            dt: 0.00044,
            substeps: 1,
            iterations: 5,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            enable_self_collision: false,
            collision_radius: 0.00005,
            collision_compliance: 1e-6,
        }
    }
}

pub fn morphology_to_simulation(
    morphology: &VoxelMorphology,
    config: &SimulatorConfig,
) -> SimulationState {
    let mut soft_body = SoftBody::new();
    let mut constraints: Vec<Box<dyn SoftConstraint>> = Vec::new();
    let mut material_map: Vec<Material> = Vec::new();
    let mut voxel_to_particle: Vec<Option<usize>> = vec![None; morphology.voxels.len()];

    for z in 0..morphology.dims[2] {
        for y in 0..morphology.dims[1] {
            for x in 0..morphology.dims[0] {
                if let Some(material) = morphology.get(x, y, z) {
                    let pos = OxVec3::new(
                        x as Real * morphology.voxel_size,
                        y as Real * morphology.voxel_size,
                        z as Real * morphology.voxel_size,
                    );
                    let p = SoftParticle::new(pos, material.density * morphology.voxel_size.powi(3));
                    let idx = soft_body.particles.len();
                    soft_body.particles.push(p);
                    material_map.push(*material);
                    voxel_to_particle[morphology.index(x, y, z)] = Some(idx);
                }
            }
        }
    }

    let step = morphology.voxel_size;
    for z in 0..morphology.dims[2] {
        for y in 0..morphology.dims[1] {
            for x in 0..morphology.dims[0] {
                if let Some(i) = voxel_to_particle[morphology.index(x, y, z)] {
                    let neighbors = [
                        (x + 1, y, z),
                        (x, y + 1, z),
                        (x, y, z + 1),
                    ];
                    for (nx, ny, nz) in neighbors {
                        if nx < morphology.dims[0] && ny < morphology.dims[1] && nz < morphology.dims[2] {
                            if let Some(j) = voxel_to_particle[morphology.index(nx, ny, nz)] {
                                let compliance = 1.0 / material_map[i].stiffness;
                                constraints.push(Box::new(DistanceConstraint::new(i, j, step, compliance)));
                            }
                        }
                    }
                }
            }
        }
    }

    let self_collision = if config.enable_self_collision && !soft_body.particles.is_empty() {
        Some(XpbdSelfCollision::new_uniform(
            soft_body.particles.len(),
            config.collision_radius,
            config.collision_compliance,
        ))
    } else {
        None
    };

    let mut solver = XpbdSolver::new(config.substeps);
    solver.num_iterations = config.iterations;
    soft_body.damping = material_map.first().map(|m| m.damping).unwrap_or(0.01);

    SimulationState {
        body: soft_body,
        constraints,
        material_map,
        solver,
        time: 0.0,
        self_collision,
    }
}

pub fn step_simulation(state: &mut SimulationState, config: &SimulatorConfig, mut fluid: Option<&mut crate::fluid_coupling::FluidEnvironment>) {
    state.body.clear_forces();

    for p in &mut state.body.particles {
        if !p.is_static() {
            let mass = 1.0 / p.inverse_mass;
            p.external_force += OxVec3::new(
                config.gravity.x * mass,
                config.gravity.y * mass,
                config.gravity.z * mass,
            );
        }
    }

    for (i, p) in state.body.particles.iter_mut().enumerate() {
        if p.is_static() {
            continue;
        }
        let material = state.material_map[i];
        if material.material_type == MaterialType::CardiacMuscle && material.active_strength > 0.0 {
            let t = state.time + material.activation_phase;
            let period = material.activation_period;
            let phase = (t % period) / period;
            let duty = material.activation_duty_cycle;
            let activation = if phase < duty {
                let x = phase / duty;
                3.0 * x * x - 2.0 * x * x * x
            } else {
                0.0
            };
            let mass = 1.0 / p.inverse_mass;
            let force = OxVec3::new(-activation * material.active_strength * mass, 0.0, 0.0);
            p.external_force += force;
        }
    }

    if let Some(ref mut fluid_env) = fluid {
        fluid_env.mark_solids(state);
        fluid_env.apply_body_forces(state);
        fluid_env.step_lbm(fluid_env.config.lbm_substeps);
        fluid_env.apply_fluid_forces(state);
    }

    state.solver.solve(&mut state.body, &mut state.constraints, config.dt);

    if let Some(ref sc) = state.self_collision {
        let positions: Vec<OxVec3> = state.body.particles.iter().map(|p| p.position).collect();
        let pairs = sc.detect_pairs(&positions);
        let mut positions_mut: Vec<OxVec3> = positions.clone();
        let inv_masses: Vec<f64> = state.body.particles.iter().map(|p| p.inverse_mass).collect();
        for pair in &pairs {
            sc.resolve_pair(pair, &mut positions_mut, &inv_masses, config.dt);
        }
        for (i, p) in state.body.particles.iter_mut().enumerate() {
            p.position = positions_mut[i];
        }
    }

    state.time += config.dt;
}

pub fn run_simulation(
    morphology: &VoxelMorphology,
    config: &SimulatorConfig,
    duration: Real,
) -> SimulationState {
    let mut state = morphology_to_simulation(morphology, config);
    let steps = (duration / config.dt) as usize;
    for _ in 0..steps {
        step_simulation(&mut state, config, None);
    }
    state
}

pub fn run_simulation_with_fluid(
    morphology: &VoxelMorphology,
    config: &SimulatorConfig,
    duration: Real,
    fluid: &mut crate::fluid_coupling::FluidEnvironment,
) -> SimulationState {
    let mut state = morphology_to_simulation(morphology, config);
    let steps = (duration / config.dt) as usize;
    for _ in 0..steps {
        step_simulation(&mut state, config, Some(fluid));
    }
    state
}

pub fn center_of_mass(state: &SimulationState) -> Vec3 {
    let mut com = Vec3::zeros();
    let mut total_mass = 0.0;
    for p in &state.body.particles {
        let mass = if p.inverse_mass > 0.0 { 1.0 / p.inverse_mass } else { 0.0 };
        com.x += p.position.x * mass;
        com.y += p.position.y * mass;
        com.z += p.position.z * mass;
        total_mass += mass;
    }
    if total_mass > 0.0 {
        com /= total_mass;
    }
    com
}

pub fn bounding_box(state: &SimulationState) -> (Vec3, Vec3) {
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &state.body.particles {
        min.x = min.x.min(p.position.x);
        min.y = min.y.min(p.position.y);
        min.z = min.z.min(p.position.z);
        max.x = max.x.max(p.position.x);
        max.y = max.y.max(p.position.y);
        max.z = max.z.max(p.position.z);
    }
    (min, max)
}

pub fn export_vtk(state: &SimulationState, path: &str) -> std::io::Result<()> {
    let positions: Vec<[f64; 3]> = state.body.particles.iter()
        .map(|p| [p.position.x, p.position.y, p.position.z])
        .collect();
    let velocities: Vec<[f64; 3]> = state.body.particles.iter()
        .map(|p| [p.velocity.x, p.velocity.y, p.velocity.z])
        .collect();
    let material_types: Vec<f64> = state.material_map.iter()
        .map(|m| match m.material_type {
            MaterialType::Passive => 0.0,
            MaterialType::CardiacMuscle => 1.0,
            MaterialType::Skin => 2.0,
        })
        .collect();

    let vtk = ParticleVtkExporter::export_particles(
        &positions,
        Some(&velocities),
        Some(("material_type", &material_types)),
    );

    std::fs::write(path, vtk)
}

pub fn export_csv_trajectory(trajectory: &[(Real, Vec3)], path: &str) -> std::io::Result<()> {
    let mut csv = String::from("time,com_x,com_y,com_z\n");
    for (time, com) in trajectory {
        csv.push_str(&format!("{},{},{},{}\n", time, com.x, com.y, com.z));
    }
    std::fs::write(path, csv)
}
