use crate::types::*;

/// Xenobot V1.0: Approximate reconstruction of the original Kriegman et al. 2020 design.
/// Spherical body with posterior-heavy cardiac muscle, anterior skin.
/// Scale: ~1mm diameter, 50µm voxels.
pub fn xenobot_v1() -> XenobotBody {
    let dims = [20, 20, 20];
    let voxel_size = 0.00005;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);
    let center = [
        dims[0] as f64 / 2.0,
        dims[1] as f64 / 2.0,
        dims[2] as f64 / 2.0,
    ];
    let radius = 8.0;

    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                let dx = x as f64 - center[0];
                let dy = y as f64 - center[1];
                let dz = z as f64 - center[2];
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                if dist < radius {
                    // Posterior half (x < center) = cardiac muscle
                    // Anterior half (x >= center) = skin
                    if dx < 0.0 {
                        morphology.set(x, y, z, Material::cardiac_muscle());
                    } else {
                        morphology.set(x, y, z, Material::skin());
                    }
                }
            }
        }
    }

    XenobotBody::new("xenobot_v1.0", morphology)
}

/// Xenobot V2.0: Ciliated design with hair-like projections (simplified as protrusions).
/// Smaller body with more surface area for ciliary action.
pub fn xenobot_v2() -> XenobotBody {
    let dims = [16, 16, 16];
    let voxel_size = 0.00005;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);
    let center = [
        dims[0] as f64 / 2.0,
        dims[1] as f64 / 2.0,
        dims[2] as f64 / 2.0,
    ];
    let radius = 5.0;

    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                let dx = x as f64 - center[0];
                let dy = y as f64 - center[1];
                let dz = z as f64 - center[2];
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                if dist < radius {
                    morphology.set(x, y, z, Material::skin());
                }
                if dist >= radius && dist < radius + 2.0 && y as f64 > center[1] {
                    morphology.set(x, y, z, Material::cardiac_muscle());
                }
            }
        }
    }

    XenobotBody::new("xenobot_v2.0", morphology)
}

/// Simplified validation target: a cardiac muscle beam that should contract and curl.
/// Expected behavior: beam shortens and curves, center of mass moves.
pub fn validation_beam() -> XenobotBody {
    let dims = [20, 4, 4];
    let voxel_size = 0.0001;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);

    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                morphology.set(x, y, z, Material::cardiac_muscle());
            }
        }
    }

    XenobotBody::new("validation_beam", morphology)
}

/// Passive sphere: should fall under gravity without active motion.
/// Useful for validating gravity and contact.
pub fn validation_sphere() -> XenobotBody {
    let dims = [12, 12, 12];
    let voxel_size = 0.0001;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);
    let center = [6.0, 6.0, 6.0];
    let radius = 4.5;

    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                let dx = x as f64 - center[0];
                let dy = y as f64 - center[1];
                let dz = z as f64 - center[2];
                if (dx * dx + dy * dy + dz * dz).sqrt() < radius {
                    morphology.set(x, y, z, Material::passive());
                }
            }
        }
    }

    XenobotBody::new("validation_sphere", morphology)
}

/// Bilateral symmetric body: two cardiac muscle lobes connected by skin bridge.
/// Expected behavior: contractile lobes pull toward each other.
pub fn validation_bilateral() -> XenobotBody {
    let dims = [24, 8, 8];
    let voxel_size = 0.0001;
    let mut morphology = VoxelMorphology::new(dims, voxel_size);

    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                let dx_left = x as f64 - 5.0;
                let dx_right = x as f64 - 18.0;
                let dy = y as f64 - dims[1] as f64 / 2.0;
                let dz = z as f64 - dims[2] as f64 / 2.0;
                
                let dist_left = (dx_left * dx_left + dy * dy + dz * dz).sqrt();
                let dist_right = (dx_right * dx_right + dy * dy + dz * dz).sqrt();
                
                if dist_left < 3.5 {
                    morphology.set(x, y, z, Material::cardiac_muscle());
                } else if dist_right < 3.5 {
                    morphology.set(x, y, z, Material::cardiac_muscle());
                } else if x >= 8 && x <= 15 && y >= 2 && y <= 5 && z >= 2 && z <= 5 {
                    morphology.set(x, y, z, Material::skin());
                }
            }
        }
    }

    XenobotBody::new("validation_bilateral", morphology)
}
