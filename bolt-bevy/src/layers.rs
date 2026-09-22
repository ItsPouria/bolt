/// A simple broad phase layer implementation that puts all objects in a single layer.
use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter,
};

/// Predefined object layers for Bolt.
pub const OBJECT_LAYER_STATIC: ObjectLayer = ObjectLayer::new(0);
pub const OBJECT_LAYER_DYNAMIC: ObjectLayer = ObjectLayer::new(1);
pub const NUM_OBJECT_LAYERS: u32 = 2;

/// Broad phase spatial trees.
pub const BROAD_PHASE_LAYER_STATIC: BroadPhaseLayer = BroadPhaseLayer::new(0);
pub const BROAD_PHASE_LAYER_DYNAMIC: BroadPhaseLayer = BroadPhaseLayer::new(1);
pub const NUM_BROAD_PHASE_LAYERS: u32 = 2;

/// Maps object layers to their broad phase acceleration tree.
pub struct SimpleBroadPhaseLayer;

impl BroadPhaseLayerInterface for SimpleBroadPhaseLayer {
    fn get_num_broad_phase_layers(&self) -> u32 {
        NUM_BROAD_PHASE_LAYERS
    }

    fn get_broad_phase_layer(&self, layer: ObjectLayer) -> BroadPhaseLayer {
        if layer == OBJECT_LAYER_STATIC {
            BROAD_PHASE_LAYER_STATIC
        } else {
            BROAD_PHASE_LAYER_DYNAMIC
        }
    }
}

/// Determines if an object layer should query a broad phase spatial tree.
pub struct SimpleObjectVsBroadPhaseLayerFilter;

impl ObjectVsBroadPhaseLayerFilter for SimpleObjectVsBroadPhaseLayerFilter {
    fn should_collide(&self, layer1: ObjectLayer, layer2: BroadPhaseLayer) -> bool {
        match layer1 {
            // Static objects only need to test against the dynamic tree
            OBJECT_LAYER_STATIC => layer2 == BROAD_PHASE_LAYER_DYNAMIC,
            // Dynamic objects test against both static and dynamic trees
            _ => true,
        }
    }
}

/// Determines if two object layers can collide in narrowphase.
pub struct SimpleObjectLayerPairFilter;
impl ObjectLayerPairFilter for SimpleObjectLayerPairFilter {
    fn should_collide(&self, layer1: ObjectLayer, layer2: ObjectLayer) -> bool {
        !matches!((layer1, layer2), (OBJECT_LAYER_STATIC, OBJECT_LAYER_STATIC))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_layers() {
        // Test the broad phase
        let broad_phase = SimpleBroadPhaseLayer;
        assert_eq!(broad_phase.get_num_broad_phase_layers(), NUM_BROAD_PHASE_LAYERS);
        assert_eq!(
            broad_phase.get_broad_phase_layer(OBJECT_LAYER_STATIC),
            BROAD_PHASE_LAYER_STATIC
        );
        assert_eq!(
            broad_phase.get_broad_phase_layer(OBJECT_LAYER_DYNAMIC),
            BROAD_PHASE_LAYER_DYNAMIC
        );

        // Test the object vs broad phase filter
        let broad_filter = SimpleObjectVsBroadPhaseLayerFilter;
        // Static objects should not query static broadphase tree
        assert!(!broad_filter.should_collide(OBJECT_LAYER_STATIC, BROAD_PHASE_LAYER_STATIC));
        // Static objects should query dynamic broadphase tree
        assert!(broad_filter.should_collide(OBJECT_LAYER_STATIC, BROAD_PHASE_LAYER_DYNAMIC));
        // Dynamic objects query both trees
        assert!(broad_filter.should_collide(OBJECT_LAYER_DYNAMIC, BROAD_PHASE_LAYER_STATIC));
        assert!(broad_filter.should_collide(OBJECT_LAYER_DYNAMIC, BROAD_PHASE_LAYER_DYNAMIC));

        // Test the Object Layer Pair filter
        let pair_filter = SimpleObjectLayerPairFilter;
        // Static vs Static should NOT collide
        assert!(!pair_filter.should_collide(OBJECT_LAYER_STATIC, OBJECT_LAYER_STATIC));
        // Dynamic vs Static and Dynamic vs Dynamic should collide
        assert!(pair_filter.should_collide(OBJECT_LAYER_STATIC, OBJECT_LAYER_DYNAMIC));
        assert!(pair_filter.should_collide(OBJECT_LAYER_DYNAMIC, OBJECT_LAYER_STATIC));
        assert!(pair_filter.should_collide(OBJECT_LAYER_DYNAMIC, OBJECT_LAYER_DYNAMIC));
    }
}
