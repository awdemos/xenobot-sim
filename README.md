# Xenobot Simulator

A high-performance simulation engine for designing and evolving synthetic biological machines (xenobots) using voxel-based soft-body physics.

## Features

- **Voxel-based soft-body physics** via OxiPhysics XPBD solver
- **Cardiac muscle contraction** model with rhythmic activation
- **LBM fluid coupling** for low Reynolds number environments
- **Self-collision detection** for realistic body interactions
- **Structural evolutionary operators**: grow/shrink regions, material swap, symmetry enforcement, component crossover
- **Multi-objective fitness**: distance, velocity, work, complexity penalty
- **Novelty search** for diversity maintenance
- **Parallel batch execution** via Rayon
- **VTK/CSV export** for visualization and analysis
- **CLI** with run, batch, evolve, validate, and example commands

## Build

```bash
cd xenobot-sim
cargo build --release
```

The binary is at `./target/release/xenobot-sim`.

## CLI Quickstart

### Run a single experiment

```bash
xenobot-sim run --config experiment.json --output result.json --vtk output.vtk
```

### Run a batch of experiments in parallel

```bash
xenobot-sim batch --configs exp1.json exp2.json exp3.json --output results.json
```

### Evolve a population

```bash
xenobot-sim evolve --population 100 --generations 20 --output best_body.json
```

### Validate against known morphologies

```bash
xenobot-sim validate --target all --duration 5.0 --vtk-dir ./vtk_outputs
```

### Generate an example experiment config

```bash
xenobot-sim example --output my_experiment.json
```

## Agent API Examples

The simulator is designed to be used as a library by agent/AI systems. All core functionality is exposed via the public API.

### Example 1: Create a Morphology Programmatically

```rust
use xenobot_sim::types::*;

fn create_crawler() -> XenobotBody {
    let dims = [16, 10, 6];
    let voxel_size = 0.0001; // 100 microns
    let mut morphology = VoxelMorphology::new(dims, voxel_size);

    // Create a bilateral crawler: cardiac muscle on left, skin on right
    for z in 1..dims[2]-1 {
        for y in 1..dims[1]-1 {
            for x in 1..dims[0]-1 {
                let dx = x as f64 - dims[0] as f64 / 2.0;
                let dy = y as f64 - dims[1] as f64 / 2.0;
                let dist = (dx*dx + dy*dy).sqrt();
                
                if dist < 4.0 {
                    if x < dims[0] / 2 {
                        morphology.set(x, y, z, Material::cardiac_muscle());
                    } else {
                        morphology.set(x, y, z, Material::skin());
                    }
                }
            }
        }
    }

    XenobotBody::new("crawler_v1", morphology)
}
```

### Example 2: Run a Single Simulation

```rust
use xenobot_sim::simulator::*;
use xenobot_sim::types::*;

fn simulate_body(body: &XenobotBody, duration_seconds: f64) -> SimulationState {
    let config = SimulatorConfig {
        dt: 0.00044,              // 0.44 ms timestep (cardiac dynamics)
        substeps: 1,
        iterations: 5,
        gravity: Vec3::new(0.0, -9.81, 0.0),
        enable_self_collision: true,
        collision_radius: 0.00005,
        collision_compliance: 1e-6,
    };

    run_simulation(&body.morphology, &config, duration_seconds)
}

fn analyze_result(state: &SimulationState) {
    let com = center_of_mass(state);
    let (min, max) = bounding_box(state);
    
    println!("Final center of mass: ({:.6}, {:.6}, {:.6})", com.x, com.y, com.z);
    println!("Bounding box: [{:.6}, {:.6}, {:.6}] -> [{:.6}, {:.6}, {:.6}]",
        min.x, min.y, min.z, max.x, max.y, max.z);
}
```

### Example 3: Export Results

```rust
use xenobot_sim::simulator::*;

fn export_results(state: &SimulationState, vtk_path: &str, csv_path: &str) {
    export_vtk(state, vtk_path).expect("VTK export failed");
    
    let trajectory: Vec<(f64, Vec3)> = vec![
        (0.0, Vec3::new(0.0, 0.0, 0.0)),
        // ... trajectory points
    ];
    export_csv_trajectory(&trajectory, csv_path).expect("CSV export failed");
}
```

### Example 4: Define an Experiment Configuration

```rust
use xenobot_sim::experiment::*;
use xenobot_sim::types::*;

fn create_experiment(body: XenobotBody) -> Experiment {
    Experiment::new("cardiac_crawler", body)
        .with_duration(10.0)
        .with_dt(0.00044)
        .with_fitness(FitnessMetric::DistanceTraveled)
}

// Or construct config directly for finer control
fn create_advanced_config() -> ExperimentConfig {
    ExperimentConfig {
        name: "advanced_test".to_string(),
        duration: 5.0,
        dt: 0.00044,
        substeps: 1,
        iterations: 5,
        gravity: [0.0, -9.81, 0.0],
        record_interval: 100,
        fitness_metric: FitnessMetric::WorkDone,
        enable_self_collision: true,
        collision_radius: 0.00005,
        collision_compliance: 1e-6,
        fluid: FluidConfig {
            enabled: false,
            domain_padding: 2.0,
            lattice_spacing: 0.0001,
            kinematic_viscosity: 1e-6,
            fluid_density: 1000.0,
            two_way_coupling: true,
            lbm_substeps: 10,
        },
    }
}
```

### Example 5: Run a Batch of Experiments

```rust
use xenobot_sim::batch::*;
use xenobot_sim::experiment::*;

fn run_parameter_sweep(body: &XenobotBody) -> Vec<ExperimentResult> {
    let mut experiments = Vec::new();
    
    // Sweep over different active strengths
    for strength in [10.0, 25.0, 50.0, 75.0, 100.0] {
        let mut mat = Material::cardiac_muscle();
        mat.active_strength = strength;
        
        let mut test_body = body.clone();
        // Modify all cardiac muscle voxels
        for voxel in test_body.morphology.voxels.iter_mut() {
            if let Some(ref mut m) = voxel {
                if m.material_type == MaterialType::CardiacMuscle {
                    m.active_strength = strength;
                }
            }
        }
        
        experiments.push(Experiment::new(
            &format!("strength_{}", strength),
            test_body
        ).with_duration(5.0));
    }
    
    run_batch_parallel(&experiments)
}
```

### Example 6: Evolutionary Search

```rust
use xenobot_sim::evolution::*;
use xenobot_sim::experiment::*;
use xenobot_sim::types::*;

fn evolve_crawlers(seed_body: XenobotBody) {
    let evolution_config = EvolutionConfig {
        population_size: 100,
        generations: 20,
        mutation_rate: 0.3,
        mutation_strength: 0.2,
        crossover_rate: 0.7,
        elitism_count: 2,
        tournament_size: 3,
        novelty_weight: 0.5,
        archive_size: 100,
        k_nearest: 15,
    };
    
    let experiment_template = ExperimentConfig {
        duration: 5.0,
        dt: 0.00044,
        fitness_metric: FitnessMetric::DistanceTraveled,
        ..Default::default()
    };
    
    let mut search = EvolutionarySearch::new(evolution_config, experiment_template);
    let generations = search.run(|| seed_body.clone());
    
    for gen in &generations {
        println!(
            "Gen {}: best={:.4}, avg={:.4}, pareto_front={}",
            gen.number, gen.best_fitness, gen.avg_fitness, gen.pareto_front.len()
        );
    }
    
    // Save best individual
    if let Some(last_gen) = generations.last() {
        if let Some(best) = last_gen.individuals.first() {
            let json = serde_json::to_string_pretty(&best.body).unwrap();
            std::fs::write("best_body.json", json).unwrap();
        }
    }
}
```

### Example 7: Custom Fitness Function

```rust
use xenobot_sim::batch::*;
use xenobot_sim::experiment::*;
use xenobot_sim::types::*;

fn evaluate_with_custom_fitness(body: &XenobotBody) -> f64 {
    let experiment = Experiment::new("custom", body.clone())
        .with_duration(10.0)
        .with_fitness(FitnessMetric::DistanceTraveled);
    
    let result = run_single_experiment(&experiment);
    
    // Primary: distance traveled
    let distance = result.fitness;
    
    // Secondary: energy efficiency (work done per voxel)
    let voxel_count = body.morphology.occupied_count() as f64;
    let work: f64 = result.trajectory.windows(2)
        .map(|w| {
            let dx = w[1].center_of_mass[0] - w[0].center_of_mass[0];
            let dy = w[1].center_of_mass[1] - w[0].center_of_mass[1];
            let dz = w[1].center_of_mass[2] - w[0].center_of_mass[2];
            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
            let dt = w[1].time - w[0].time;
            dist * dist / dt.max(1e-10)
        })
        .sum();
    
    let efficiency = work / voxel_count.max(1.0);
    
    // Combined score
    distance * 0.7 + efficiency * 0.3
}
```

### Example 8: Fluid-Coupled Simulation

```rust
use xenobot_sim::simulator::*;
use xenobot_sim::fluid_coupling::*;
use xenobot_sim::types::*;

fn simulate_with_fluid(body: &XenobotBody) -> SimulationState {
    let sim_config = SimulatorConfig::default();
    let fluid_config = FluidConfig {
        enabled: true,
        domain_padding: 2.0,
        lattice_spacing: 0.0002,  // Coarser = faster
        kinematic_viscosity: 1e-6,
        fluid_density: 1000.0,
        two_way_coupling: true,
        lbm_substeps: 1,          // Reduce for speed
    };
    
    let mut fluid = FluidEnvironment::around_morphology(
        &body.morphology,
        fluid_config
    );
    
    run_simulation_with_fluid(
        &body.morphology,
        &sim_config,
        5.0,  // duration
        &mut fluid
    )
}
```

### Example 9: Validation Pipeline

```rust
use xenobot_sim::validation::*;
use xenobot_sim::simulator::*;
use xenobot_sim::types::*;

fn run_all_validations() {
    let bodies = vec![
        xenobot_v1(),
        xenobot_v2(),
        validation_beam(),
        validation_sphere(),
        validation_bilateral(),
    ];
    
    let config = SimulatorConfig {
        dt: 0.00044,
        substeps: 1,
        iterations: 5,
        gravity: Vec3::new(0.0, -9.81, 0.0),
        enable_self_collision: false,
        ..Default::default()
    };
    
    for body in &bodies {
        println!("\nValidating: {} ({} voxels)", body.name, body.morphology.occupied_count());
        let state = run_simulation(&body.morphology, &config, 5.0);
        let com = center_of_mass(&state);
        println!("  Final CoM: ({:.6}, {:.6}, {:.6})", com.x, com.y, com.z);
    }
}
```

### Example 10: Agent Loop — Design → Simulate → Score → Iterate

```rust
use xenobot_sim::types::*;
use xenobot_sim::simulator::*;
use xenobot_sim::batch::*;
use xenobot_sim::experiment::*;

struct XenobotAgent {
    best_fitness: f64,
    best_body: Option<XenobotBody>,
    iteration: usize,
}

impl XenobotAgent {
    fn new() -> Self {
        Self {
            best_fitness: 0.0,
            best_body: None,
            iteration: 0,
        }
    }
    
    fn design_body(&self) -> XenobotBody {
        // Your agent's design logic here
        // Could be: learned policy, grammar, evolutionary, etc.
        let dims = [12, 8, 6];
        let voxel_size = 0.0001;
        let mut morphology = VoxelMorphology::new(dims, voxel_size);
        
        // Simple random design
        for z in 1..dims[2]-1 {
            for y in 1..dims[1]-1 {
                for x in 1..dims[0]-1 {
                    if rand::random::<f64>() < 0.3 {
                        let mat = if rand::random::<bool>() {
                            Material::cardiac_muscle()
                        } else {
                            Material::skin()
                        };
                        morphology.set(x, y, z, mat);
                    }
                }
            }
        }
        
        XenobotBody::new(&format!("agent_design_{}", self.iteration), morphology)
    }
    
    fn evaluate(&mut self, body: &XenobotBody) -> f64 {
        let experiment = Experiment::new("agent_eval", body.clone())
            .with_duration(5.0)
            .with_fitness(FitnessMetric::DistanceTraveled);
        
        let result = run_single_experiment(&experiment);
        result.fitness
    }
    
    fn step(&mut self) {
        let body = self.design_body();
        let fitness = self.evaluate(&body);
        
        if fitness > self.best_fitness {
            self.best_fitness = fitness;
            self.best_body = Some(body);
            println!("Iteration {}: New best fitness = {:.4}", self.iteration, fitness);
        }
        
        self.iteration += 1;
    }
}

fn main() {
    let mut agent = XenobotAgent::new();
    for _ in 0..100 {
        agent.step();
    }
    println!("Best fitness after 100 iterations: {:.4}", agent.best_fitness);
}
```

## Architecture

```
xenobot-sim/
├── src/
│   ├── main.rs           # CLI entry point
│   ├── lib.rs            # Public API exports
│   ├── types.rs          # Core types (Material, VoxelMorphology, XenobotBody)
│   ├── simulator.rs      # OXiPhysics XPBD integration, cardiac model
│   ├── fluid_coupling.rs # LBM fluid environment
│   ├── experiment.rs     # Experiment configuration DSL
│   ├── batch.rs          # Parallel batch execution
│   ├── evolution.rs      # Genetic algorithm with novelty search
│   └── validation.rs     # Pre-defined validation morphologies
```

## Performance

| Particles | Step Time | Real-time Factor |
|-----------|-----------|------------------|
| 1,000     | 0.006 ms  | 73×              |
| 10,000    | 0.063 ms  | 7×               |
| 32,000    | 0.369 ms  | 1.2×             |

Evolutionary search: 1000 pop × 50 gen ≈ 40 minutes (parallelized).

## License

MIT OR Apache-2.0
