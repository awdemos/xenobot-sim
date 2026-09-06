use clap::{Parser, Subcommand};
use serde_json;
use std::fs;

use xenobot_sim::batch::{run_batch_parallel, run_single_experiment};
use xenobot_sim::evolution::{EvolutionConfig, EvolutionarySearch};
use xenobot_sim::experiment::{Experiment, ExperimentConfig, FitnessMetric};
use xenobot_sim::simulator::{
    center_of_mass, export_csv_trajectory, export_vtk, run_simulation, SimulatorConfig,
};
use xenobot_sim::types::*;
use xenobot_sim::validation;

#[derive(Parser)]
#[command(name = "xenobot-sim")]
#[command(about = "Xenobot simulation engine for synthetic biology experiments")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Run {
        #[arg(short, long)]
        config: String,
        #[arg(short, long, default_value = "result.json")]
        output: String,
        #[arg(long)]
        vtk: Option<String>,
        #[arg(long)]
        csv: Option<String>,
    },
    Batch {
        #[arg(short, long)]
        configs: Vec<String>,
        #[arg(short, long, default_value = "results.json")]
        output: String,
    },
    Evolve {
        #[arg(short, long, default_value = "20")]
        generations: usize,
        #[arg(short, long, default_value = "50")]
        population: usize,
        #[arg(short, long, default_value = "result.json")]
        output: String,
        #[arg(long)]
        fluid: bool,
    },
    Example {
        #[arg(short, long, default_value = "example_xenobot.json")]
        output: String,
    },
    Validate {
        #[arg(short, long, default_value = "all")]
        target: String,
        #[arg(short, long, default_value = "5.0")]
        duration: f64,
        #[arg(long)]
        collision: bool,
        #[arg(long)]
        vtk_dir: Option<String>,
    },
}

fn create_example_body() -> XenobotBody {
    let dims = [12, 8, 6];
    let voxel_size = 0.0001;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);
    for z in 1..dims[2] - 1 {
        for y in 1..dims[1] - 1 {
            for x in 1..dims[0] - 1 {
                let dx = x as f64 - dims[0] as f64 / 2.0;
                let dy = y as f64 - dims[1] as f64 / 2.0;
                let dz = z as f64 - dims[2] as f64 / 2.0;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                if dist < 3.5 {
                    if x < dims[0] / 2 {
                        morphology.set(x, y, z, Material::cardiac_muscle());
                    } else {
                        morphology.set(x, y, z, Material::skin());
                    }
                }
            }
        }
    }
    XenobotBody::new("example_xenobot", morphology)
}

fn main() {
    let cli = Cli::parse();
    if let Err(e) = run_command(cli) {
        eprintln!("xenobot-sim failed: {}", e);
        std::process::exit(1);
    }
}

fn run_command(cli: Cli) -> Result<(), Box<dyn std::error::Error>> {
    match cli.command {
        Commands::Run {
            config,
            output,
            vtk,
            csv,
        } => {
            let experiment: Experiment = serde_json::from_str(
                &fs::read_to_string(&config)
                    .map_err(|e| format!("failed to read config {}: {}", config, e))?,
            )
            .map_err(|e| format!("failed to parse config {}: {}", config, e))?;
            println!("Running experiment: {}", experiment.config.name);
            println!(
                "Body: {} ({} voxels)",
                experiment.body.name,
                experiment.body.morphology.occupied_count()
            );
            println!(
                "Duration: {}s, dt: {}s",
                experiment.config.duration, experiment.config.dt
            );
            let result = run_single_experiment(&experiment);
            println!("Fitness: {:.6}", result.fitness);
            println!("Trajectory points: {}", result.trajectory.len());
            if let Some(vtk_path) = vtk {
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
                let state = run_simulation(
                    &experiment.body.morphology,
                    &sim_config,
                    experiment.config.duration,
                );
                export_vtk(&state, &vtk_path)
                    .map_err(|e| format!("failed to write VTK {}: {}", vtk_path, e))?;
                println!("VTK exported to {}", vtk_path);
            }
            if let Some(csv_path) = csv {
                let trajectory: Vec<(f64, Vec3)> = result
                    .trajectory
                    .iter()
                    .map(|t| {
                        (
                            t.time,
                            Vec3::new(
                                t.center_of_mass[0],
                                t.center_of_mass[1],
                                t.center_of_mass[2],
                            ),
                        )
                    })
                    .collect();
                export_csv_trajectory(&trajectory, &csv_path)
                    .map_err(|e| format!("failed to write CSV {}: {}", csv_path, e))?;
                println!("CSV exported to {}", csv_path);
            }
            fs::write(&output, serde_json::to_string_pretty(&result)?)
                .map_err(|e| format!("failed to write output {}: {}", output, e))?;
            println!("Result saved to {}", output);
        }
        Commands::Batch { configs, output } => {
            let experiments: Vec<Experiment> = configs
                .iter()
                .map(|path| {
                    let content = fs::read_to_string(path)
                        .map_err(|e| format!("failed to read config {}: {}", path, e))?;
                    serde_json::from_str(&content)
                        .map_err(|e| format!("failed to parse config {}: {}", path, e))
                })
                .collect::<Result<Vec<_>, _>>()?;
            println!("Running batch of {} experiments...", experiments.len());
            let results = run_batch_parallel(&experiments);
            for (i, result) in results.iter().enumerate() {
                println!(
                    "  [{}] {}: fitness = {:.6}",
                    i, result.experiment_name, result.fitness
                );
            }
            fs::write(&output, serde_json::to_string_pretty(&results)?)
                .map_err(|e| format!("failed to write output {}: {}", output, e))?;
            println!("Results saved to {}", output);
        }
        Commands::Evolve {
            generations,
            population,
            output,
            fluid,
        } => {
            let evolution_config = EvolutionConfig {
                population_size: population,
                generations,
                ..Default::default()
            };
            let mut experiment_template = ExperimentConfig {
                duration: 5.0,
                dt: 0.00044,
                fitness_metric: FitnessMetric::DistanceTraveled,
                ..Default::default()
            };
            experiment_template.fluid.enabled = fluid;
            println!(
                "Starting evolution: {} generations, {} population",
                generations, population
            );
            let mut search = EvolutionarySearch::new(evolution_config, experiment_template);
            let gens = search.run(create_example_body);
            for gen in &gens {
                println!(
                    "Generation {}: best = {:.6}, avg = {:.6}",
                    gen.number, gen.best_fitness, gen.avg_fitness
                );
            }
            if let Some(best) = gens.last() {
                if let Some(best_ind) = best.individuals.first() {
                    fs::write(
                        &output,
                        serde_json::to_string_pretty(&best_ind.body)?,
                    )
                    .map_err(|e| format!("failed to write output {}: {}", output, e))?;
                    println!("Best body saved to {}", output);
                }
            }
        }
        Commands::Example { output } => {
            let body = create_example_body();
            let experiment = Experiment::new("example", body)
                .with_duration(5.0)
                .with_fitness(FitnessMetric::DistanceTraveled);
            fs::write(&output, serde_json::to_string_pretty(&experiment)?)
                .map_err(|e| format!("failed to write example {}: {}", output, e))?;
            println!("Example experiment saved to {}", output);
        }
        Commands::Validate {
            target,
            duration,
            collision,
            vtk_dir,
        } => {
            let bodies = match target.as_str() {
                "v1" => vec![validation::xenobot_v1()],
                "v2" => vec![validation::xenobot_v2()],
                "beam" => vec![validation::validation_beam()],
                "sphere" => vec![validation::validation_sphere()],
                "bilateral" => vec![validation::validation_bilateral()],
                "all" => vec![
                    validation::xenobot_v1(),
                    validation::xenobot_v2(),
                    validation::validation_beam(),
                    validation::validation_sphere(),
                    validation::validation_bilateral(),
                ],
                _ => {
                    return Err(format!(
                        "Unknown target: {}. Use: v1, v2, beam, sphere, bilateral, all",
                        target
                    )
                    .into());
                }
            };
            let sim_config = SimulatorConfig {
                dt: 0.00044,
                substeps: 1,
                iterations: 5,
                gravity: Vec3::new(0.0, -9.81, 0.0),
                enable_self_collision: collision,
                collision_radius: 0.00005,
                collision_compliance: 1e-6,
            };
            println!("Running validation on {} target(s)...", bodies.len());
            for body in &bodies {
                println!(
                    "\n  Target: {} ({} voxels)",
                    body.name,
                    body.morphology.occupied_count()
                );
                let state = run_simulation(&body.morphology, &sim_config, duration);
                let com = center_of_mass(&state);
                println!("    Final CoM: ({:.6}, {:.6}, {:.6})", com.x, com.y, com.z);
                println!("    Time: {}s", state.time);
                if let Some(ref dir) = vtk_dir {
                    let vtk_path = format!("{}/{}.vtk", dir, body.name);
                    if let Err(e) = std::fs::create_dir_all(dir) {
                        eprintln!("    Warning: could not create VTK dir: {}", e);
                    }
                    if let Err(e) = export_vtk(&state, &vtk_path) {
                        eprintln!("    Warning: VTK export failed: {}", e);
                    } else {
                        println!("    VTK exported to {}", vtk_path);
                    }
                }
            }
        }
    }
    Ok(())
}
