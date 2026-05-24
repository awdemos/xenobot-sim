use crate::experiment::*;
use crate::simulator::*;
use crate::types::*;
use crate::fluid_coupling::FluidEnvironment;
use rayon::prelude::*;

pub fn run_single_experiment(experiment: &Experiment) -> ExperimentResult {
    let sim_config = SimulatorConfig {
        dt: experiment.config.dt,
        substeps: experiment.config.substeps,
        iterations: experiment.config.iterations,
        gravity: Vec3::new(
            experiment.config.gravity[0],
            experiment.config.gravity[1],
            experiment.config.gravity[2],
        ),
        enable_self_collision: experiment.config.enable_self_collision,
        collision_radius: experiment.config.collision_radius,
        collision_compliance: experiment.config.collision_compliance,
    };

    let mut state = morphology_to_simulation(&experiment.body.morphology, &sim_config);
    let steps = (experiment.config.duration / experiment.config.dt) as usize;
    let record_interval = experiment.config.record_interval;

    let mut trajectory = Vec::new();
    let initial_com = center_of_mass(&state);

    let mut fluid = if experiment.config.fluid.enabled {
        Some(FluidEnvironment::around_morphology(&experiment.body.morphology, experiment.config.fluid.clone()))
    } else {
        None
    };

    for step in 0..steps {
        step_simulation(&mut state, &sim_config, fluid.as_mut());

        if step % record_interval == 0 {
            let com = center_of_mass(&state);
            let (bb_min, bb_max) = bounding_box(&state);
            trajectory.push(TrajectoryPoint {
                time: state.time,
                center_of_mass: [com.x, com.y, com.z],
                bounding_box_min: [bb_min.x, bb_min.y, bb_min.z],
                bounding_box_max: [bb_max.x, bb_max.y, bb_max.z],
            });
        }
    }

    let final_com = center_of_mass(&state);
    let fitness = compute_fitness(&experiment.config.fitness_metric, initial_com, final_com, &trajectory);

    ExperimentResult {
        experiment_name: experiment.config.name.clone(),
        fitness,
        trajectory,
        final_state: None,
    }
}

pub fn run_batch_sequential(experiments: &[Experiment]) -> Vec<ExperimentResult> {
    experiments.iter().map(|exp| run_single_experiment(exp)).collect()
}

pub fn run_batch_parallel(experiments: &[Experiment]) -> Vec<ExperimentResult> {
    experiments.par_iter().map(|exp| run_single_experiment(exp)).collect()
}

fn compute_fitness(
    metric: &FitnessMetric,
    initial_com: Vec3,
    final_com: Vec3,
    trajectory: &[TrajectoryPoint],
) -> Real {
    match metric {
        FitnessMetric::DistanceTraveled => {
            let dx = final_com.x - initial_com.x;
            let dy = final_com.y - initial_com.y;
            let dz = final_com.z - initial_com.z;
            (dx * dx + dy * dy + dz * dz).sqrt()
        }
        FitnessMetric::Velocity => {
            if trajectory.len() >= 2 {
                let dt = trajectory[trajectory.len() - 1].time - trajectory[0].time;
                let dx = final_com.x - initial_com.x;
                let dy = final_com.y - initial_com.y;
                let dz = final_com.z - initial_com.z;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                if dt > 0.0 { dist / dt } else { 0.0 }
            } else {
                0.0
            }
        }
        FitnessMetric::WorkDone => {
            let mut work = 0.0;
            for i in 1..trajectory.len() {
                let prev = &trajectory[i - 1];
                let curr = &trajectory[i];
                let dx = curr.center_of_mass[0] - prev.center_of_mass[0];
                let dy = curr.center_of_mass[1] - prev.center_of_mass[1];
                let dz = curr.center_of_mass[2] - prev.center_of_mass[2];
                let dt = curr.time - prev.time;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                work += dist * dist / dt.max(1e-10);
            }
            work
        }
        FitnessMetric::DisplacementX => (final_com.x - initial_com.x).abs(),
        FitnessMetric::DisplacementY => (final_com.y - initial_com.y).abs(),
        FitnessMetric::DisplacementZ => (final_com.z - initial_com.z).abs(),
        FitnessMetric::MaxHeight => {
            trajectory.iter().map(|t| t.center_of_mass[1]).fold(0.0_f64, f64::max)
        }
    }
}
