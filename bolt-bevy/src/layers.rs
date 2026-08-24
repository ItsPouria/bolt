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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_layers() {
        // Test the broad phase
        let broad_phase = SimpleBroadPhaseLayer;
        assert_eq!(broad_phase.get_num_broad_phase_layers(), 1);
        assert_eq!(
            broad_phase.get_broad_phase_layer(ObjectLayer::new(0)).raw(),
            0
        );

        // Test the object vs broad phase filter
        let broad_filter = SimpleObjectVsBroadPhaseLayerFilter;
        assert!(broad_filter.should_collide(ObjectLayer::new(0), BroadPhaseLayer::new(0)));

        // Test the missing Object Layer Pair filter!
        let pair_filter = SimpleObjectLayerPairFilter;
        assert!(pair_filter.should_collide(ObjectLayer::new(0), ObjectLayer::new(1)));
    }
}
