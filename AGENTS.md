# Agent Notes: xenobot-sim

A Rust simulation engine for designing and evolving synthetic biological machines (xenobots) with voxel-based soft-body physics, cardiac-muscle contraction, and LBM fluid coupling.

## Repository Layout

- `src/main.rs` — CLI entrypoint (`run`, `batch`, `evolve`, `validate`, `example`).
- `src/lib.rs` — Public library API.
- `src/types.rs` — Voxel morphology, materials, body, and simulation-state types.
- `src/simulator.rs` — Core XPBD soft-body simulator and cardiac-muscle model.
- `src/evolution.rs` — Evolutionary operators and novelty search.
- `src/validation.rs` — Validation against known morphologies.
- `src/export.rs` — VTK/CSV output for visualization and analysis.
- `src/cli.rs` — Clap-based CLI definitions.
- `Cargo.toml` — Workspace manifest; depends on local OxiPhysics crates under `/tmp/oxiphysics`.

## Build

```bash
cargo build --release
```

The binary is at `./target/release/xenobot-sim`.

## Run Tests

```bash
cargo test
```

## CLI Usage

```bash
# Run a single experiment
xenobot-sim run --config experiment.json --output result.json --vtk output.vtk

# Run a batch of experiments in parallel
xenobot-sim batch --configs exp1.json exp2.json exp3.json --output results.json

# Evolve a population
xenobot-sim evolve --population 100 --generations 20 --output best_body.json

# Validate against known morphologies
xenobot-sim validate --target all --duration 5.0 --vtk-dir ./vtk_outputs

# Generate an example experiment config
xenobot-sim example --output my_experiment.json
```

## Library API

The crate exposes a public Rust API. See `src/lib.rs` and the examples in `README.md` for creating morphologies, running simulations, and exporting results.

## Key Conventions

- Voxel size is typically 0.0001 (100 µm); keep units consistent in config JSON.
- Materials (`Material::cardiac_muscle()`, `Material::skin()`, etc.) determine voxel behavior.
- Simulation configs use a fixed 0.44 ms timestep for cardiac dynamics.
- The OxiPhysics dependency paths in `Cargo.toml` point to `/tmp/oxiphysics`; update them or set up that workspace before building in a fresh environment.

## Common Issues

- **Build fails with missing `oxiphysics-*` crates**: clone/setup the OxiPhysics workspace at `/tmp/oxiphysics` or adjust the `path` dependencies in `Cargo.toml`.
- **CLI command not found after build**: use `./target/release/xenobot-sim` or add `target/release` to `PATH`.
- **VTK export empty**: verify `--vtk` is passed with a writable path and the simulation ran for at least one frame.

## License

MIT OR Apache-2.0. See `Cargo.toml` and `LICENSE`.
