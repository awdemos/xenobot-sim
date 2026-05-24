use crate::types::*;
use crate::simulator::*;
use oxiphysics_lbm::d3q19_full::D3q19Simulation;
use oxiphysics_core::math::Vec3 as OxVec3;

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct FluidConfig {
    pub enabled: bool,
    pub domain_padding: Real,
    pub lattice_spacing: Real,
    pub kinematic_viscosity: Real,
    pub fluid_density: Real,
    pub two_way_coupling: bool,
    pub lbm_substeps: usize,
}

impl Default for FluidConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            domain_padding: 2.0,
            lattice_spacing: 0.0001,
            kinematic_viscosity: 1e-6,
            fluid_density: 1000.0,
            two_way_coupling: true,
            lbm_substeps: 10,
        }
    }
}

pub struct FluidEnvironment {
    pub lbm: D3q19Simulation,
    pub config: FluidConfig,
    pub dx: Real,
    pub origin: Vec3,
}

impl FluidEnvironment {
    pub fn around_morphology(morphology: &VoxelMorphology, config: FluidConfig) -> Self {
        let bb_min = Vec3::zeros();
        let bb_max = Vec3::new(
            morphology.dims[0] as Real * morphology.voxel_size,
            morphology.dims[1] as Real * morphology.voxel_size,
            morphology.dims[2] as Real * morphology.voxel_size,
        );
        let size = bb_max - bb_min;
        let padded_size = size * (1.0 + 2.0 * config.domain_padding);
        
        let nx = (padded_size.x / config.lattice_spacing).ceil() as usize;
        let ny = (padded_size.y / config.lattice_spacing).ceil() as usize;
        let nz = (padded_size.z / config.lattice_spacing).ceil() as usize;
        
        let nx = nx.max(8);
        let ny = ny.max(8);
        let nz = nz.max(8);
        
        let dt_lbm = 1.0; // LBM timestep in lattice units
        let nu_lb = config.kinematic_viscosity * dt_lbm / (config.lattice_spacing * config.lattice_spacing);
        let tau = 3.0 * nu_lb + 0.5;
        
        let tau = tau.clamp(0.51, 1.9);
        
        let lbm = D3q19Simulation::new(nx, ny, nz, tau);
        
        let origin = Vec3::new(
            bb_min.x - config.domain_padding * size.x,
            bb_min.y - config.domain_padding * size.y,
            bb_min.z - config.domain_padding * size.z,
        );
        
        Self {
            lbm,
            config,
            dx: config.lattice_spacing,
            origin,
        }
    }
    
    pub fn mark_solids(&mut self, state: &SimulationState) {
        let n = self.lbm.nx * self.lbm.ny * self.lbm.nz;
        self.lbm.solid = vec![false; n];
        
        for p in &state.body.particles {
            let rel_pos = Vec3::new(p.position.x, p.position.y, p.position.z) - self.origin;
            let ix = (rel_pos.x / self.dx).round() as isize;
            let iy = (rel_pos.y / self.dx).round() as isize;
            let iz = (rel_pos.z / self.dx).round() as isize;
            
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let x = ix + dx;
                        let y = iy + dy;
                        let z = iz + dz;
                        
                        if x >= 0 && x < self.lbm.nx as isize
                            && y >= 0 && y < self.lbm.ny as isize
                            && z >= 0 && z < self.lbm.nz as isize
                        {
                            let idx = (x as usize) + (y as usize) * self.lbm.nx + (z as usize) * self.lbm.nx * self.lbm.ny;
                            self.lbm.solid[idx] = true;
                        }
                    }
                }
            }
        }
    }
    
    fn stokes_drag(&self, particle_vel: Vec3, fluid_vel: Vec3, radius: Real) -> Vec3 {
        let mu = self.config.kinematic_viscosity * self.config.fluid_density; // Dynamic viscosity
        let slip = fluid_vel - particle_vel;
        let factor = 6.0 * std::f64::consts::PI * mu * radius;
        slip * factor
    }
    
    pub fn fluid_velocity_at(&self, pos: Vec3) -> Vec3 {
        let rel_pos = pos - self.origin;
        let xf = (rel_pos.x / self.dx).clamp(0.0, (self.lbm.nx - 1) as Real - 1e-10);
        let yf = (rel_pos.y / self.dx).clamp(0.0, (self.lbm.ny - 1) as Real - 1e-10);
        let zf = (rel_pos.z / self.dx).clamp(0.0, (self.lbm.nz - 1) as Real - 1e-10);
        
        let x0 = xf.floor() as usize;
        let y0 = yf.floor() as usize;
        let z0 = zf.floor() as usize;
        let x1 = (x0 + 1).min(self.lbm.nx - 1);
        let y1 = (y0 + 1).min(self.lbm.ny - 1);
        let z1 = (z0 + 1).min(self.lbm.nz - 1);
        
        let tx = xf - x0 as Real;
        let ty = yf - y0 as Real;
        let tz = zf - z0 as Real;
        
        let idx = |x, y, z| x + y * self.lbm.nx + z * self.lbm.nx * self.lbm.ny;
        
        let v000 = self.lbm.macroscopic(idx(x0, y0, z0)).1;
        let v100 = self.lbm.macroscopic(idx(x1, y0, z0)).1;
        let v010 = self.lbm.macroscopic(idx(x0, y1, z0)).1;
        let v110 = self.lbm.macroscopic(idx(x1, y1, z0)).1;
        let v001 = self.lbm.macroscopic(idx(x0, y0, z1)).1;
        let v101 = self.lbm.macroscopic(idx(x1, y0, z1)).1;
        let v011 = self.lbm.macroscopic(idx(x0, y1, z1)).1;
        let v111 = self.lbm.macroscopic(idx(x1, y1, z1)).1;
        
        let ux = trilerp(v000[0], v100[0], v010[0], v110[0], v001[0], v101[0], v011[0], v111[0], tx, ty, tz);
        let uy = trilerp(v000[1], v100[1], v010[1], v110[1], v001[1], v101[1], v011[1], v111[1], tx, ty, tz);
        let uz = trilerp(v000[2], v100[2], v010[2], v110[2], v001[2], v101[2], v011[2], v111[2], tx, ty, tz);
        
        let factor = self.dx; // dt = 1 in lattice units
        Vec3::new(ux * factor, uy * factor, uz * factor)
    }
    
    pub fn apply_fluid_forces(&self, state: &mut SimulationState) {
        let particle_radius = self.dx * 0.5; // Approximate particle as half lattice spacing
        
        for p in &mut state.body.particles {
            let pos = Vec3::new(p.position.x, p.position.y, p.position.z);
            let fluid_vel = self.fluid_velocity_at(pos);
            let particle_vel = Vec3::new(p.velocity.x, p.velocity.y, p.velocity.z);
            let drag = self.stokes_drag(particle_vel, fluid_vel, particle_radius);
            
            p.external_force += OxVec3::new(drag.x, drag.y, drag.z);
        }
    }
    
    pub fn apply_body_forces(&mut self, state: &SimulationState) {
        if !self.config.two_way_coupling {
            return;
        }
        
        let mut fx_grid = vec![0.0f64; self.lbm.nx * self.lbm.ny * self.lbm.nz];
        let mut fy_grid = vec![0.0f64; self.lbm.nx * self.lbm.ny * self.lbm.nz];
        let mut fz_grid = vec![0.0f64; self.lbm.nx * self.lbm.ny * self.lbm.nz];
        
        for p in &state.body.particles {
            let rel_pos = Vec3::new(p.position.x, p.position.y, p.position.z) - self.origin;
            let ix = (rel_pos.x / self.dx).round() as usize;
            let iy = (rel_pos.y / self.dx).round() as usize;
            let iz = (rel_pos.z / self.dx).round() as usize;
            
            if ix < self.lbm.nx && iy < self.lbm.ny && iz < self.lbm.nz {
                let idx = ix + iy * self.lbm.nx + iz * self.lbm.nx * self.lbm.ny;
                let cell_volume = self.dx * self.dx * self.dx;
                fx_grid[idx] += p.external_force.x / cell_volume;
                fy_grid[idx] += p.external_force.y / cell_volume;
                fz_grid[idx] += p.external_force.z / cell_volume;
            }
        }
        
        let n = self.lbm.nx * self.lbm.ny * self.lbm.nz;
        let _prefactor = 1.0 - 1.0 / (2.0 * self.lbm.tau);
        
        for k in 0..n {
            if self.lbm.solid[k] {
                continue;
            }
            let fx = fx_grid[k];
            let fy = fy_grid[k];
            let fz = fz_grid[k];
            
            let fx_lb = fx * (self.dx * self.dx * self.dx); // Simplified scaling
            let fy_lb = fy * (self.dx * self.dx * self.dx);
            let fz_lb = fz * (self.dx * self.dx * self.dx);
            
            self.lbm.apply_body_force(fx_lb, fy_lb, fz_lb);
        }
    }
    
    pub fn step_lbm(&mut self, n_steps: usize) {
        for _ in 0..n_steps {
            self.lbm.step();
        }
    }
    
    pub fn total_kinetic_energy(&self) -> Real {
        self.lbm.total_kinetic_energy()
    }
    
    pub fn max_velocity(&self) -> Real {
        self.lbm.max_velocity() * self.dx // Convert to physical units
    }
}

fn trilerp(
    c000: Real, c100: Real, c010: Real, c110: Real,
    c001: Real, c101: Real, c011: Real, c111: Real,
    tx: Real, ty: Real, tz: Real,
) -> Real {
    let c00 = c000 * (1.0 - tx) + c100 * tx;
    let c10 = c010 * (1.0 - tx) + c110 * tx;
    let c01 = c001 * (1.0 - tx) + c101 * tx;
    let c11 = c011 * (1.0 - tx) + c111 * tx;
    
    let c0 = c00 * (1.0 - ty) + c10 * ty;
    let c1 = c01 * (1.0 - ty) + c11 * ty;
    
    c0 * (1.0 - tz) + c1 * tz
}
