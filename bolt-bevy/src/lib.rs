use bevy::prelude::*;
use joltc_sys::*;
use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter, PhysicsSystem,
};

pub struct SimpleBroadPhaseLayer;
impl BroadPhaseLayerInterface for SimpleBroadPhaseLayer {
    fn get_num_broad_phase_layers(&self) -> u32 {
        1
    }
    fn get_broad_phase_layer(&self, _layer: ObjectLayer) -> BroadPhaseLayer {
        BroadPhaseLayer::new(0)
    }
}

pub struct SimpleObjectVsBroadPhaseLayerFilter;
impl ObjectVsBroadPhaseLayerFilter for SimpleObjectVsBroadPhaseLayerFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: BroadPhaseLayer) -> bool {
        true
    }
}

pub struct SimpleObjectLayerPairFilter;
impl ObjectLayerPairFilter for SimpleObjectLayerPairFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: ObjectLayer) -> bool {
        true
    }
}

#[derive(Resource)]
pub struct PhysicsWorld {
    pub physics_system: PhysicsSystem,
    pub temp_allocator: *mut JPC_TempAllocatorImpl,
    pub job_system: *mut JPC_JobSystemThreadPool,
}
impl PhysicsWorld {
    pub fn new() -> Self {
        println!("1. Initializing Jolt Core");
        unsafe {
            JPC_RegisterDefaultAllocator();
            JPC_FactoryInit();
            JPC_RegisterTypes();
        }

        println!("2. Creating PhysicsSystem");
        let mut physics_system = PhysicsSystem::new();

        println!("3. Calling PhysicsSystem.init");
        physics_system.init(
            10240,
            0,
            65536,
            10240,
            SimpleBroadPhaseLayer,
            SimpleObjectVsBroadPhaseLayerFilter,
            SimpleObjectLayerPairFilter,
        );

        println!("4. Creating TempAllocator");
        let temp_allocator = unsafe { JPC_TempAllocatorImpl_new(10 * 1024 * 1024) };

        println!("5. Creating JobSystem");
        let job_system = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                2,
            )
        };
        // 4. Return the struct
        Self {
            physics_system,
            temp_allocator,
            job_system,
        }
    }
}

// We tell Rust that it is safe to send this struct and share references to it
// across threads. This is safe because Jolt's PhysicsSystem is designed for
// multi-threaded access, and Bevy's ResMut ensures we don't mutate it from
// multiple threads at the exact same time.
unsafe impl Send for PhysicsWorld {}
unsafe impl Sync for PhysicsWorld {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_world_physics() {
        // 1. Create the physics world
        let world = PhysicsWorld::new();

        // 2. Step the simulation forward by one frame
        let delta_time = 1.0 / 60.0; // 60 FPS
        let collision_steps = 1;

        println!("6. Stepping simulation");
        unsafe {
            world.physics_system.update(
                delta_time,
                collision_steps,
                world.temp_allocator,
                world.job_system,
            );
        }
        println!("7. Done!");
    }
}
