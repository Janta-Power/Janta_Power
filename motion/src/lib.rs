// ============================================================================
// MOTION MODULE - PUBLIC API
// ============================================================================
// This file defines the public interface for the motion module.
// All implementation details are in motion.rs
// State definitions are in states.rs

mod states;
mod motion;

// Re-export the public API
pub use motion::Motion;
pub use states::TrackingState;
