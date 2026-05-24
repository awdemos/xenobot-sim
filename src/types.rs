use serde::{Deserialize, Serialize};

pub type Real = f64;
pub type Vec3 = nalgebra::Vector3<Real>;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MaterialType {
    Passive,
    CardiacMuscle,
    Skin,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub material_type: MaterialType,
    pub stiffness: Real,
    pub damping: Real,
    pub density: Real,
    pub active_strength: Real,
    pub activation_period: Real,
    pub activation_duty_cycle: Real,
    pub activation_phase: Real,
}

impl Material {
    pub fn passive() -> Self {
        Self {
            material_type: MaterialType::Passive,
            stiffness: 1000.0,
            damping: 0.01,
            density: 1000.0,
            active_strength: 0.0,
            activation_period: 1.0,
            activation_duty_cycle: 0.0,
            activation_phase: 0.0,
        }
    }

    pub fn cardiac_muscle() -> Self {
        Self {
            material_type: MaterialType::CardiacMuscle,
            stiffness: 1000.0,
            damping: 0.01,
            density: 1000.0,
            active_strength: 50.0,
            activation_period: 0.5,
            activation_duty_cycle: 0.3,
            activation_phase: 0.0,
        }
    }

    pub fn skin() -> Self {
        Self {
            material_type: MaterialType::Skin,
            stiffness: 500.0,
            damping: 0.02,
            density: 1100.0,
            active_strength: 0.0,
            activation_period: 1.0,
            activation_duty_cycle: 0.0,
            activation_phase: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoxelMorphology {
    pub dims: [usize; 3],
    pub voxel_size: Real,
    pub voxels: Vec<Option<Material>>,
}

impl VoxelMorphology {
    pub fn new(dims: [usize; 3], voxel_size: Real) -> Self {
        let n = dims[0] * dims[1] * dims[2];
        Self {
            dims,
            voxel_size,
            voxels: vec![None; n],
        }
    }

    pub fn index(&self, x: usize, y: usize, z: usize) -> usize {
        x + self.dims[0] * (y + self.dims[1] * z)
    }

    pub fn set(&mut self, x: usize, y: usize, z: usize, material: Material) {
        let idx = self.index(x, y, z);
        self.voxels[idx] = Some(material);
    }

    pub fn get(&self, x: usize, y: usize, z: usize) -> Option<&Material> {
        let idx = self.index(x, y, z);
        self.voxels[idx].as_ref()
    }

    pub fn occupied_count(&self) -> usize {
        self.voxels.iter().filter(|v| v.is_some()).count()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XenobotBody {
    pub name: String,
    pub morphology: VoxelMorphology,
}

impl XenobotBody {
    pub fn new(name: &str, morphology: VoxelMorphology) -> Self {
        Self {
            name: name.to_string(),
            morphology,
        }
    }
}
