use crate::batch::*;
use crate::experiment::*;
use crate::types::*;
use rand::Rng;
use std::collections::{HashSet, VecDeque};

#[derive(Debug, Clone)]
pub struct EvolutionConfig {
    pub population_size: usize,
    pub generations: usize,
    pub mutation_rate: f64,
    pub mutation_strength: f64,
    pub crossover_rate: f64,
    pub elitism_count: usize,
    pub tournament_size: usize,
    pub novelty_weight: f64,
    pub archive_size: usize,
    pub k_nearest: usize,
}

impl Default for EvolutionConfig {
    fn default() -> Self {
        Self {
            population_size: 50,
            generations: 20,
            mutation_rate: 0.3,
            mutation_strength: 0.2,
            crossover_rate: 0.7,
            elitism_count: 2,
            tournament_size: 3,
            novelty_weight: 0.5,
            archive_size: 100,
            k_nearest: 15,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MultiObjectiveFitness {
    pub distance: f64,
    pub velocity: f64,
    pub work_done: f64,
    pub robustness: f64,
    pub complexity: f64,
    pub novelty: f64,
}

impl MultiObjectiveFitness {
    pub fn scalar(&self, weights: &[f64; 5]) -> f64 {
        weights[0] * self.distance
            + weights[1] * self.velocity
            + weights[2] * self.work_done
            + weights[3] * self.robustness
            - weights[4] * self.complexity
    }

    pub fn dominates(&self, other: &MultiObjectiveFitness) -> bool {
        (self.distance >= other.distance
            && self.velocity >= other.velocity
            && self.work_done >= other.work_done
            && self.robustness >= other.robustness
            && self.complexity <= other.complexity)
            && (self.distance > other.distance
                || self.velocity > other.velocity
                || self.work_done > other.work_done
                || self.robustness > other.robustness
                || self.complexity < other.complexity)
    }
}

#[derive(Clone)]
pub struct Individual {
    pub body: XenobotBody,
    pub fitness: Option<f64>,
    pub mo_fitness: Option<MultiObjectiveFitness>,
    pub behavior_vector: Vec<f64>,
}

#[derive(Clone)]
pub struct Generation {
    pub number: usize,
    pub individuals: Vec<Individual>,
    pub best_fitness: f64,
    pub avg_fitness: f64,
    pub pareto_front: Vec<usize>,
    pub archive: Vec<Vec<f64>>,
}

pub struct EvolutionarySearch {
    pub config: EvolutionConfig,
    pub experiment_template: ExperimentConfig,
    pub generations: Vec<Generation>,
    pub mo_weights: [f64; 5],
}

impl EvolutionarySearch {
    pub fn new(evolution_config: EvolutionConfig, experiment_template: ExperimentConfig) -> Self {
        Self {
            config: evolution_config,
            experiment_template,
            generations: Vec::new(),
            mo_weights: [1.0, 0.5, 0.3, 0.5, 0.1],
        }
    }

    pub fn initialize_population<F>(&self, seed_fn: F) -> Vec<Individual>
    where
        F: Fn() -> XenobotBody,
    {
        (0..self.config.population_size)
            .map(|_| Individual {
                body: seed_fn(),
                fitness: None,
                mo_fitness: None,
                behavior_vector: Vec::new(),
            })
            .collect()
    }

    pub fn evaluate_population(&self, population: &mut [Individual]) {
        let experiments: Vec<Experiment> = population
            .iter()
            .enumerate()
            .map(|(i, ind)| Experiment {
                config: ExperimentConfig {
                    name: format!("gen_{}_ind_{}", self.generations.len(), i),
                    ..self.experiment_template.clone()
                },
                body: ind.body.clone(),
            })
            .collect();

        let results = run_batch_parallel(&experiments);

        for (i, result) in results.iter().enumerate() {
            population[i].fitness = Some(result.fitness);
            
            let mut behavior = Vec::new();
            for tp in &result.trajectory {
                behavior.push(tp.center_of_mass[0]);
                behavior.push(tp.center_of_mass[1]);
                behavior.push(tp.center_of_mass[2]);
            }
            population[i].behavior_vector = behavior;
            
            let traj = &result.trajectory;
            let complexity = population[i].body.morphology.occupied_count() as f64 / 100.0;
            
            let (velocity, work_done) = if traj.len() >= 2 {
                let dt = traj[traj.len() - 1].time - traj[0].time;
                let dx = traj[traj.len() - 1].center_of_mass[0] - traj[0].center_of_mass[0];
                let dy = traj[traj.len() - 1].center_of_mass[1] - traj[0].center_of_mass[1];
                let dz = traj[traj.len() - 1].center_of_mass[2] - traj[0].center_of_mass[2];
                let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                let vel = if dt > 0.0 { dist / dt } else { 0.0 };
                
                let mut work = 0.0;
                for j in 1..traj.len() {
                    let prev = &traj[j - 1];
                    let curr = &traj[j];
                    let ddx = curr.center_of_mass[0] - prev.center_of_mass[0];
                    let ddy = curr.center_of_mass[1] - prev.center_of_mass[1];
                    let ddz = curr.center_of_mass[2] - prev.center_of_mass[2];
                    let ddt = curr.time - prev.time;
                    let ddist = (ddx*ddx + ddy*ddy + ddz*ddz).sqrt();
                    work += ddist * ddist / ddt.max(1e-10);
                }
                (vel, work)
            } else {
                (0.0, 0.0)
            };
            
            population[i].mo_fitness = Some(MultiObjectiveFitness {
                distance: result.fitness,
                velocity,
                work_done,
                robustness: 0.0,
                complexity,
                novelty: 0.0,
            });
        }
    }

    fn compute_novelty(&self, individual: &mut Individual, archive: &[Vec<f64>]) {
        if individual.behavior_vector.is_empty() {
            return;
        }
        
        let k = self.config.k_nearest.min(archive.len());
        if k == 0 {
            if let Some(ref mut mo) = individual.mo_fitness {
                mo.novelty = 1.0;
            }
            return;
        }
        
        let mut distances: Vec<f64> = archive
            .iter()
            .map(|bv| {
                let min_len = bv.len().min(individual.behavior_vector.len());
                if min_len == 0 {
                    return 1.0;
                }
                let mut dist = 0.0;
                for i in 0..min_len {
                    let d = bv[i] - individual.behavior_vector[i];
                    dist += d * d;
                }
                dist.sqrt() / min_len as f64
            })
            .collect();
        
        distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let avg_dist: f64 = distances.iter().take(k).sum::<f64>() / k as f64;
        
        if let Some(ref mut mo) = individual.mo_fitness {
            mo.novelty = avg_dist;
        }
    }

    pub fn select_parent<'a, R: Rng>(&self, population: &'a [Individual], rng: &mut R) -> &'a Individual {
        let mut best = &population[rng.gen_range(0..population.len())];
        for _ in 1..self.config.tournament_size {
            let candidate = &population[rng.gen_range(0..population.len())];
            let best_score = best.mo_fitness.as_ref()
                .map(|mo| mo.scalar(&self.mo_weights) + self.config.novelty_weight * mo.novelty)
                .unwrap_or(0.0);
            let cand_score = candidate.mo_fitness.as_ref()
                .map(|mo| mo.scalar(&self.mo_weights) + self.config.novelty_weight * mo.novelty)
                .unwrap_or(0.0);
            if cand_score > best_score {
                best = candidate;
            }
        }
        best
    }

    pub fn crossover(&self, parent1: &XenobotBody, parent2: &XenobotBody) -> XenobotBody {
        let mut child = parent1.clone();
        if parent1.morphology.dims != parent2.morphology.dims {
            return child;
        }
        let mut rng = rand::thread_rng();
        
        let comps1 = self.find_connected_components(&parent1.morphology);
        let comps2 = self.find_connected_components(&parent2.morphology);
        
        if !comps1.is_empty() && !comps2.is_empty() && rng.gen::<f64>() < 0.5 {
            let comp_idx = rng.gen_range(0..comps2.len());
            let comp2 = &comps2[comp_idx];
            
            let offset_x = rng.gen_range(0..child.morphology.dims[0].saturating_sub(1));
            let offset_y = rng.gen_range(0..child.morphology.dims[1].saturating_sub(1));
            let offset_z = rng.gen_range(0..child.morphology.dims[2].saturating_sub(1));
            
            for &(x, y, z, mat) in comp2 {
                let nx = (x as isize + offset_x as isize).clamp(0, child.morphology.dims[0] as isize - 1) as usize;
                let ny = (y as isize + offset_y as isize).clamp(0, child.morphology.dims[1] as isize - 1) as usize;
                let nz = (z as isize + offset_z as isize).clamp(0, child.morphology.dims[2] as isize - 1) as usize;
                child.morphology.set(nx, ny, nz, mat);
            }
        } else {
            let cx = rng.gen_range(0..child.morphology.dims[0]);
            let cy = rng.gen_range(0..child.morphology.dims[1]);
            let cz = rng.gen_range(0..child.morphology.dims[2]);
            
            for z in 0..child.morphology.dims[2] {
                for y in 0..child.morphology.dims[1] {
                    for x in 0..child.morphology.dims[0] {
                        let dx = x as f64 - cx as f64;
                        let dy = y as f64 - cy as f64;
                        let dz = z as f64 - cz as f64;
                        let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                        let p = (dist / 5.0).clamp(0.0, 1.0);
                        if rng.gen::<f64>() > p {
                            let idx = child.morphology.index(x, y, z);
                            child.morphology.voxels[idx] = parent2.morphology.voxels[idx].clone();
                        }
                    }
                }
            }
        }
        
        child
    }

    fn find_connected_components(&self, morphology: &VoxelMorphology) -> Vec<Vec<(usize, usize, usize, Material)>> {
        let mut visited = vec![false; morphology.voxels.len()];
        let mut components = Vec::new();
        
        for z in 0..morphology.dims[2] {
            for y in 0..morphology.dims[1] {
                for x in 0..morphology.dims[0] {
                    let idx = morphology.index(x, y, z);
                    if visited[idx] || morphology.voxels[idx].is_none() {
                        continue;
                    }
                    
                    let mut component = Vec::new();
                    let mut queue = VecDeque::new();
                    queue.push_back((x, y, z));
                    visited[idx] = true;
                    
                    while let Some((cx, cy, cz)) = queue.pop_front() {
                        let cidx = morphology.index(cx, cy, cz);
                        if let Some(mat) = morphology.voxels[cidx] {
                            component.push((cx, cy, cz, mat));
                        }
                        
                        for dz in -1..=1 {
                            for dy in -1..=1 {
                                for dx in -1..=1 {
                                    if dx == 0 && dy == 0 && dz == 0 {
                                        continue;
                                    }
                                    let nx = cx as isize + dx;
                                    let ny = cy as isize + dy;
                                    let nz = cz as isize + dz;
                                    if nx >= 0 && nx < morphology.dims[0] as isize
                                        && ny >= 0 && ny < morphology.dims[1] as isize
                                        && nz >= 0 && nz < morphology.dims[2] as isize
                                    {
                                        let nidx = morphology.index(nx as usize, ny as usize, nz as usize);
                                        if !visited[nidx] && morphology.voxels[nidx].is_some() {
                                            visited[nidx] = true;
                                            queue.push_back((nx as usize, ny as usize, nz as usize));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    if !component.is_empty() {
                        components.push(component);
                    }
                }
            }
        }
        
        components
    }

    pub fn mutate(&self, individual: &mut Individual) {
        let mut rng = rand::thread_rng();
        let operators = ["grow", "shrink", "swap", "symmetry", "single"];
        let op = operators[rng.gen_range(0..operators.len())];
        
        match op {
            "grow" => self.grow_region(&mut individual.body.morphology, &mut rng),
            "shrink" => self.shrink_region(&mut individual.body.morphology, &mut rng),
            "swap" => self.swap_material_block(&mut individual.body.morphology, &mut rng),
            "symmetry" => self.enforce_symmetry(&mut individual.body.morphology),
            "single" => self.single_voxel_mutate(&mut individual.body.morphology, &mut rng),
            _ => {}
        }
        
        individual.fitness = None;
        individual.mo_fitness = None;
        individual.behavior_vector.clear();
    }

    fn grow_region<R: Rng>(&self, morphology: &mut VoxelMorphology, rng: &mut R) {
        let mut candidates = Vec::new();
        for z in 0..morphology.dims[2] {
            for y in 0..morphology.dims[1] {
                for x in 0..morphology.dims[0] {
                    let idx = morphology.index(x, y, z);
                    if morphology.voxels[idx].is_none() {
                        let mut adjacent = false;
                        for dz in -1..=1 {
                            for dy in -1..=1 {
                                for dx in -1..=1 {
                                    if dx == 0 && dy == 0 && dz == 0 { continue; }
                                    let nx = x as isize + dx;
                                    let ny = y as isize + dy;
                                    let nz = z as isize + dz;
                                    if nx >= 0 && nx < morphology.dims[0] as isize
                                        && ny >= 0 && ny < morphology.dims[1] as isize
                                        && nz >= 0 && nz < morphology.dims[2] as isize
                                    {
                                        let nidx = morphology.index(nx as usize, ny as usize, nz as usize);
                                        if morphology.voxels[nidx].is_some() {
                                            adjacent = true;
                                            break;
                                        }
                                    }
                                }
                                if adjacent { break; }
                            }
                            if adjacent { break; }
                        }
                        if adjacent {
                            candidates.push((x, y, z));
                        }
                    }
                }
            }
        }
        
        if candidates.is_empty() { return; }
        
        let start = candidates[rng.gen_range(0..candidates.len())];
        let size = rng.gen_range(1..=5);
        let material = if rng.gen::<f64>() < 0.5 {
            Material::cardiac_muscle()
        } else {
            Material::skin()
        };
        
        let mut queue = VecDeque::new();
        let mut grown = HashSet::new();
        queue.push_back(start);
        grown.insert(start);
        
        while let Some((cx, cy, cz)) = queue.pop_front() {
            if grown.len() >= size { break; }
            morphology.set(cx, cy, cz, material);
            
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if dx == 0 && dy == 0 && dz == 0 { continue; }
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        let nz = cz as isize + dz;
                        if nx >= 0 && nx < morphology.dims[0] as isize
                            && ny >= 0 && ny < morphology.dims[1] as isize
                            && nz >= 0 && nz < morphology.dims[2] as isize
                        {
                            let npos = (nx as usize, ny as usize, nz as usize);
                            if !grown.contains(&npos) {
                                grown.insert(npos);
                                queue.push_back(npos);
                            }
                        }
                    }
                }
            }
        }
    }

    fn shrink_region<R: Rng>(&self, morphology: &mut VoxelMorphology, rng: &mut R) {
        
        let mut candidates = Vec::new();
        for z in 0..morphology.dims[2] {
            for y in 0..morphology.dims[1] {
                for x in 0..morphology.dims[0] {
                    let idx = morphology.index(x, y, z);
                    if morphology.voxels[idx].is_some() {
                        
                        let mut surface = false;
                        for dz in -1..=1 {
                            for dy in -1..=1 {
                                for dx in -1..=1 {
                                    if dx == 0 && dy == 0 && dz == 0 { continue; }
                                    let nx = x as isize + dx;
                                    let ny = y as isize + dy;
                                    let nz = z as isize + dz;
                                    if nx >= 0 && nx < morphology.dims[0] as isize
                                        && ny >= 0 && ny < morphology.dims[1] as isize
                                        && nz >= 0 && nz < morphology.dims[2] as isize
                                    {
                                        let nidx = morphology.index(nx as usize, ny as usize, nz as usize);
                                        if morphology.voxels[nidx].is_none() {
                                            surface = true;
                                            break;
                                        }
                                    } else {
                                        surface = true;
                                        break;
                                    }
                                }
                                if surface { break; }
                            }
                            if surface { break; }
                        }
                        if surface {
                            candidates.push((x, y, z));
                        }
                    }
                }
            }
        }
        
        if candidates.is_empty() { return; }
        
        let start = candidates[rng.gen_range(0..candidates.len())];
        let size = rng.gen_range(1..=5);
        let mut queue = VecDeque::new();
        let mut removed = HashSet::new();
        queue.push_back(start);
        removed.insert(start);
        
        while let Some((cx, cy, cz)) = queue.pop_front() {
            if removed.len() >= size { break; }
            let idx = morphology.index(cx, cy, cz);
            morphology.voxels[idx] = None;
            
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if dx == 0 && dy == 0 && dz == 0 { continue; }
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        let nz = cz as isize + dz;
                        if nx >= 0 && nx < morphology.dims[0] as isize
                            && ny >= 0 && ny < morphology.dims[1] as isize
                            && nz >= 0 && nz < morphology.dims[2] as isize
                        {
                            let nidx = morphology.index(nx as usize, ny as usize, nz as usize);
                            if morphology.voxels[nidx].is_some() {
                                let npos = (nx as usize, ny as usize, nz as usize);
                                if !removed.contains(&npos) {
                                    removed.insert(npos);
                                    queue.push_back(npos);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fn swap_material_block<R: Rng>(&self, morphology: &mut VoxelMorphology, rng: &mut R) {
        let materials = [Material::passive(), Material::cardiac_muscle(), Material::skin()];
        let new_mat = materials[rng.gen_range(0..materials.len())];
        
        
        let mut occupied = Vec::new();
        for z in 0..morphology.dims[2] {
            for y in 0..morphology.dims[1] {
                for x in 0..morphology.dims[0] {
                    let idx = morphology.index(x, y, z);
                    if morphology.voxels[idx].is_some() {
                        occupied.push((x, y, z));
                    }
                }
            }
        }
        
        if occupied.is_empty() { return; }
        
        let start = occupied[rng.gen_range(0..occupied.len())];
        let size = rng.gen_range(3..=15);
        let mut queue = VecDeque::new();
        let mut changed = HashSet::new();
        queue.push_back(start);
        changed.insert(start);
        
        while let Some((cx, cy, cz)) = queue.pop_front() {
            if changed.len() >= size { break; }
            morphology.set(cx, cy, cz, new_mat);
            
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if dx == 0 && dy == 0 && dz == 0 { continue; }
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        let nz = cz as isize + dz;
                        if nx >= 0 && nx < morphology.dims[0] as isize
                            && ny >= 0 && ny < morphology.dims[1] as isize
                            && nz >= 0 && nz < morphology.dims[2] as isize
                        {
                            let nidx = morphology.index(nx as usize, ny as usize, nz as usize);
                            if morphology.voxels[nidx].is_some() {
                                let npos = (nx as usize, ny as usize, nz as usize);
                                if !changed.contains(&npos) {
                                    changed.insert(npos);
                                    queue.push_back(npos);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fn enforce_symmetry(&self, morphology: &mut VoxelMorphology) {
        let mid = morphology.dims[0] / 2;
        for z in 0..morphology.dims[2] {
            for y in 0..morphology.dims[1] {
                for x in 0..mid {
                    let mirror_x = morphology.dims[0] - 1 - x;
                    let idx = morphology.index(x, y, z);
                    let mirror_idx = morphology.index(mirror_x, y, z);
                    morphology.voxels[mirror_idx] = morphology.voxels[idx].clone();
                }
            }
        }
    }

    fn single_voxel_mutate<R: Rng>(&self, morphology: &mut VoxelMorphology, rng: &mut R) {
        let idx = rng.gen_range(0..morphology.voxels.len());
        morphology.voxels[idx] = match morphology.voxels[idx] {
            None => Some(Material::passive()),
            Some(_) => None,
        };
    }

    fn compute_pareto_front(&self, population: &[Individual]) -> Vec<usize> {
        let mut front = Vec::new();
        for (i, ind) in population.iter().enumerate() {
            let mo_i = match &ind.mo_fitness {
                Some(mo) => mo,
                None => continue,
            };
            let mut dominated = false;
            for (j, other) in population.iter().enumerate() {
                if i == j { continue; }
                let mo_j = match &other.mo_fitness {
                    Some(mo) => mo,
                    None => continue,
                };
                if mo_j.dominates(mo_i) {
                    dominated = true;
                    break;
                }
            }
            if !dominated {
                front.push(i);
            }
        }
        front
    }

    pub fn run_generation<R: Rng>(&mut self, population: &mut Vec<Individual>, rng: &mut R) -> Generation {
        self.evaluate_population(population);
        
        
        let archive = if let Some(last_gen) = self.generations.last() {
            last_gen.archive.clone()
        } else {
            Vec::new()
        };
        
        for ind in population.iter_mut() {
            self.compute_novelty(ind, &archive);
        }
        
        
        let mut new_archive = archive;
        for ind in population.iter() {
            if !ind.behavior_vector.is_empty() {
                new_archive.push(ind.behavior_vector.clone());
            }
        }
        while new_archive.len() > self.config.archive_size {
            new_archive.remove(0);
        }
        
        
        population.sort_by(|a, b| {
            let score_a = a.mo_fitness.as_ref()
                .map(|mo| mo.scalar(&self.mo_weights) + self.config.novelty_weight * mo.novelty)
                .unwrap_or(0.0);
            let score_b = b.mo_fitness.as_ref()
                .map(|mo| mo.scalar(&self.mo_weights) + self.config.novelty_weight * mo.novelty)
                .unwrap_or(0.0);
            score_b.partial_cmp(&score_a).unwrap()
        });
        
        let best_fitness = population[0].fitness.unwrap_or(0.0);
        let avg_fitness = population.iter().map(|i| i.fitness.unwrap_or(0.0)).sum::<f64>() / population.len() as f64;
        let pareto_front = self.compute_pareto_front(population);
        
        let mut new_population = Vec::new();
        
        
        for i in 0..self.config.elitism_count.min(population.len()) {
            new_population.push(Individual {
                body: population[i].body.clone(),
                fitness: population[i].fitness,
                mo_fitness: population[i].mo_fitness.clone(),
                behavior_vector: population[i].behavior_vector.clone(),
            });
        }
        
        
        let novelty_count = (self.config.population_size as f64 * 0.2) as usize;
        let mut novelty_candidates: Vec<_> = population.iter().enumerate().collect();
        novelty_candidates.sort_by(|(_, a), (_, b)| {
            let n_a = a.mo_fitness.as_ref().map(|mo| mo.novelty).unwrap_or(0.0);
            let n_b = b.mo_fitness.as_ref().map(|mo| mo.novelty).unwrap_or(0.0);
            n_b.partial_cmp(&n_a).unwrap()
        });
        for (i, _) in novelty_candidates.iter().take(novelty_count) {
            if new_population.len() >= self.config.population_size { break; }
            new_population.push(Individual {
                body: population[*i].body.clone(),
                fitness: population[*i].fitness,
                mo_fitness: population[*i].mo_fitness.clone(),
                behavior_vector: population[*i].behavior_vector.clone(),
            });
        }
        
        while new_population.len() < self.config.population_size {
            let parent1 = self.select_parent(population, rng);
            let child = if rand::thread_rng().gen::<f64>() < self.config.crossover_rate {
                let parent2 = self.select_parent(population, rng);
                self.crossover(&parent1.body, &parent2.body)
            } else {
                parent1.body.clone()
            };
            
            let mut child_individual = Individual {
                body: child,
                fitness: None,
                mo_fitness: None,
                behavior_vector: Vec::new(),
            };
            self.mutate(&mut child_individual);
            new_population.push(child_individual);
        }
        
        let gen_number = self.generations.len();
        let gen = Generation {
            number: gen_number,
            individuals: population.clone(),
            best_fitness,
            avg_fitness,
            pareto_front: pareto_front.clone(),
            archive: new_archive.clone(),
        };
        self.generations.push(gen.clone());
        
        *population = new_population;
        
        gen
    }

    pub fn run<F>(&mut self, seed_fn: F) -> Vec<Generation>
    where
        F: Fn() -> XenobotBody,
    {
        let mut population = self.initialize_population(seed_fn);
        let mut rng = rand::thread_rng();
        
        for _ in 0..self.config.generations {
            self.run_generation(&mut population, &mut rng);
        }
        
        self.generations.clone()
    }
}
