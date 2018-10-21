use amethyst_core::nalgebra::Vector3;
use amethyst_core::specs::prelude::Component;
use amethyst_core::specs::storage::HashMapStorage;
use output::Output;

/// An audio listener, add this component to the local player character.
#[derive(Debug)]
pub struct AudioListener {
    /// Output used by this listener to emit sounds to
    pub output: Output,
    /// Position of the left ear relative to the global transform on this entity.
    pub left_ear: Vector3<f32>,
    /// Position of the right ear relative to the global transform on this entity.
    pub right_ear: Vector3<f32>,
}

impl Component for AudioListener {
    type Storage = HashMapStorage<Self>;
}
