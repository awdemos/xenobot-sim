use crate::types::*;
use serde::{Deserialize, Serialize};

use crate::fluid_coupling::FluidConfig;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentConfig {
    pub name: String,
    pub duration: Real,
    pub dt: Real,
    pub substeps: usize,
    pub iterations: usize,
    pub gravity: [Real; 3],
    pub record_interval: usize,
    pub fitness_metric: FitnessMetric,
    pub enable_self_collision: bool,
    pub collision_radius: Real,
    pub collision_compliance: Real,
    pub fluid: FluidConfig,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum FitnessMetric {
    DistanceTraveled,
    Velocity,
    WorkDone,
    DisplacementX,
    DisplacementY,
    DisplacementZ,
    MaxHeight,
}

impl Default for ExperimentConfig {
    fn default() -> Self {
        Self {
            name: "unnamed".to_string(),
            duration: 10.0,
            dt: 0.00044,
            substeps: 1,
            iterations: 5,
            gravity: [0.0, -9.81, 0.0],
            record_interval: 100,
            fitness_metric: FitnessMetric::DistanceTraveled,
            enable_self_collision: false,
            collision_radius: 0.00005,
            collision_compliance: 1e-6,
            fluid: FluidConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Experiment {
    pub config: ExperimentConfig,
    pub body: XenobotBody,
}

impl Experiment {
    pub fn new(name: &str, body: XenobotBody) -> Self {
        Self {
            config: ExperimentConfig {
                name: name.to_string(),
                ..Default::default()
            },
            body,
        }
    }

    pub fn with_duration(mut self, duration: Real) -> Self {
        self.config.duration = duration;
        self
    }

    pub fn with_dt(mut self, dt: Real) -> Self {
        self.config.dt = dt;
        self
    }

    pub fn with_fitness(mut self, metric: FitnessMetric) -> Self {
        self.config.fitness_metric = metric;
        self
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub time: Real,
    pub center_of_mass: [Real; 3],
    pub bounding_box_min: [Real; 3],
    pub bounding_box_max: [Real; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentResult {
    pub experiment_name: String,
    pub fitness: Real,
    pub trajectory: Vec<TrajectoryPoint>,
    pub final_state: Option<String>,
}
