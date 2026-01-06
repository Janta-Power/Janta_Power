// This module contains all state-related enums and types for the motion system.
// To add or modify states, edit this file only.

/// - L1: Primary tracking state - actively following the sun
/// - L2: Secondary tracking state - fine-tuning position based on balance
/// - L3: Future tracking state - reserved for future functionality
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum TrackingState {
    L1,
    L2,
    L3,
}

// Add helper functions here if needed for state transitions or validation

impl TrackingState {
    /// Check if the tracker is in an active tracking state
    pub fn is_active(&self) -> bool {
        matches!(self, TrackingState::L1 | TrackingState::L2)
    }
    
    /// Get the next state in the tracking sequence
    pub fn next(&self) -> Option<TrackingState> {
        match self {
            TrackingState::L1 => Some(TrackingState::L2),
            TrackingState::L2 => Some(TrackingState::L3),
            TrackingState::L3 => None,
        }
    }
    
    /// Get the previous state in the tracking sequence
    pub fn previous(&self) -> Option<TrackingState> {
        match self {
            TrackingState::L1 => None,
            TrackingState::L2 => Some(TrackingState::L1),
            TrackingState::L3 => Some(TrackingState::L2),
        }
    }
}

