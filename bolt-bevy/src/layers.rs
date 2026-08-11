/// A simple broad phase layer implementation that puts all objects in a single layer.
use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter,
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

/// A default filter that allows all objects to collide with the broad phase.
pub struct SimpleObjectVsBroadPhaseLayerFilter;
impl ObjectVsBroadPhaseLayerFilter for SimpleObjectVsBroadPhaseLayerFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: BroadPhaseLayer) -> bool {
        true
    }
}

/// A default filter that allows all objects to collide with each other.
pub struct SimpleObjectLayerPairFilter;
impl ObjectLayerPairFilter for SimpleObjectLayerPairFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: ObjectLayer) -> bool {
        true
    }
}
