use chrono::Utc;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Source of an event entering the nervous system
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum EventSource {
    RobotBridge,
    Vision,
    Voice,
    Sensor,
    Brain,
    Interaction,
    SafetyGateway,
    System,
    Custom(String),
}

impl std::fmt::Display for EventSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EventSource::RobotBridge => write!(f, "RobotBridge"),
            EventSource::Vision => write!(f, "Vision"),
            EventSource::Voice => write!(f, "Voice"),
            EventSource::Sensor => write!(f, "Sensor"),
            EventSource::Brain => write!(f, "Brain"),
            EventSource::Interaction => write!(f, "Interaction"),
            EventSource::SafetyGateway => write!(f, "SafetyGateway"),
            EventSource::System => write!(f, "System"),
            EventSource::Custom(s) => write!(f, "{}", s),
        }
    }
}

fn default_pan_range_deg() -> f64 {
    60.0
}
fn default_tilt_deg() -> f64 {
    10.0
}
fn default_speed_deg_s() -> f64 {
    25.0
}

/// Action types dispatched by the Kernel to actuators / adapters
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "action", content = "params")]
pub enum RobotAction {
    Speak {
        text: String,
    },
    NavigateTo {
        x: f64,
        y: f64,
        #[serde(default)]
        yaw: Option<f64>,
    },
    TurnTo {
        yaw: f64,
    },
    NavigateToRoom {
        room: String,
    },
    LookAt {
        x: f32,
        y: f32,
        z: f32,
    },
    ScanRoom {
        #[serde(default = "default_pan_range_deg")]
        pan_range_deg: f64,
        #[serde(default = "default_tilt_deg")]
        tilt_deg: f64,
        #[serde(default = "default_speed_deg_s")]
        speed_deg_s: f64,
    },
    Express {
        emotion: String,
    },
    Stop,
    StartMission {
        #[serde(default)]
        mission_id: Option<String>,
        #[serde(default)]
        mission_name: Option<String>,
        #[serde(default)]
        waypoints: serde_json::Value,
    },
    PatrolRooms {
        #[serde(default)]
        rooms: Vec<String>,
    },
    NavigateWaypoints {
        #[serde(default)]
        waypoints: serde_json::Value,
    },
    ReturnHome,
    Cancel,
    Pause,
    Resume,
    VoiceInterrupt {
        #[serde(default)]
        phrase: Option<String>,
    },
    VoiceResume {
        #[serde(default)]
        phrase: Option<String>,
    },
    StatusQuery {
        #[serde(default)]
        query: Option<String>,
    },
    Move {
        #[serde(default)]
        linear_x: f64,
        #[serde(default)]
        angular_z: f64,
    },
}

impl RobotAction {
    pub fn validate(&self) -> Result<(), String> {
        match self {
            RobotAction::ScanRoom {
                pan_range_deg,
                tilt_deg,
                speed_deg_s,
            } => {
                if !pan_range_deg.is_finite() || !tilt_deg.is_finite() || !speed_deg_s.is_finite() {
                    return Err("ScanRoom parameters must be finite numbers".to_string());
                }
                if *pan_range_deg <= 0.0 || *pan_range_deg > 120.0 {
                    return Err(format!(
                        "ScanRoom pan_range_deg must be in (0.0, 120.0], got {}",
                        pan_range_deg
                    ));
                }
                if *tilt_deg < -45.0 || *tilt_deg > 45.0 {
                    return Err(format!(
                        "ScanRoom tilt_deg must be in [-45.0, 45.0], got {}",
                        tilt_deg
                    ));
                }
                if *speed_deg_s <= 0.0 || *speed_deg_s > 90.0 {
                    return Err(format!(
                        "ScanRoom speed_deg_s must be positive and <= 90.0, got {}",
                        speed_deg_s
                    ));
                }
                Ok(())
            }
            RobotAction::LookAt { x, y, z } => {
                if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                    return Err("LookAt coordinates must be finite numbers".to_string());
                }
                Ok(())
            }
            RobotAction::TurnTo { yaw } => {
                if !yaw.is_finite() {
                    return Err("TurnTo yaw must be a finite number".to_string());
                }
                Ok(())
            }
            RobotAction::NavigateTo { x, y, yaw } => {
                if !x.is_finite() || !y.is_finite() {
                    return Err("NavigateTo coordinates must be finite numbers".to_string());
                }
                if let Some(y_val) = yaw {
                    if !y_val.is_finite() {
                        return Err("NavigateTo yaw must be a finite number".to_string());
                    }
                }
                Ok(())
            }
            RobotAction::NavigateToRoom { room } => {
                if room.trim().is_empty() {
                    return Err("NavigateToRoom room name cannot be empty".to_string());
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }
}

/// Strongly-typed event payloads for physical awareness and reasoning
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum EventType {
    // SENSORY: Robot Telemetry & Navigation
    RobotPosition {
        x: f64,
        y: f64,
        theta: f64,
        speed: f64,
    },
    RobotBattery {
        level: u8,
        charging: bool,
    },
    ObstacleDetected {
        distance: f64,
        direction: String,
    },
    NavigationArrived {
        destination: String,
        success: bool,
    },
    NavigationStarted {
        destination: String,
        #[serde(default)]
        target_x: Option<f64>,
        #[serde(default)]
        target_y: Option<f64>,
        #[serde(default)]
        estimated_duration_sec: Option<f64>,
    },
    NavigationFailed {
        #[serde(default)]
        destination: String,
        #[serde(default)]
        reason: String,
        #[serde(default)]
        stuck_x: Option<f64>,
        #[serde(default)]
        stuck_y: Option<f64>,
        #[serde(default)]
        recovery_attempts: Option<u32>,
    },
    RobotStopped {
        #[serde(default)]
        reason: String,
        #[serde(default)]
        x: Option<f64>,
        #[serde(default)]
        y: Option<f64>,
        #[serde(default)]
        emergency: bool,
    },
    AutonomyStateChanged {
        state: String,
        #[serde(default)]
        previous_state: Option<String>,
        #[serde(default)]
        reason: Option<String>,
        #[serde(default)]
        mission_id: Option<String>,
        #[serde(default)]
        mission_name: Option<String>,
        #[serde(default)]
        x: Option<f64>,
        #[serde(default)]
        y: Option<f64>,
    },
    ScanCompleted {
        room: String,
        #[serde(default)]
        objects_found: Vec<String>,
        success: bool,
        #[serde(default)]
        duration_sec: f64,
    },
    RoomExplored {
        room: String,
        timestamp: i64,
        #[serde(default)]
        objects_count: usize,
    },

    // SENSORY: Vision
    PersonVisible {
        identity: String,
        confidence: f32,
        distance: Option<f32>,
    },
    ObjectDetected {
        class_name: String,
        confidence: f32,
        x: f32,
        y: f32,
        #[serde(default)]
        z: Option<f32>,
        #[serde(default)]
        bbox: Option<Vec<f32>>,
        #[serde(default)]
        depth_m: Option<f32>,
        #[serde(default)]
        frame_id: Option<String>,
    },
    VisionDetection {
        #[serde(default)]
        room: Option<String>,
        #[serde(default)]
        objects: Vec<String>,
        #[serde(default)]
        confidence: Option<f32>,
        #[serde(default)]
        timestamp: Option<f64>,
    },
    LookAtResult {
        #[serde(default)]
        success: bool,
        #[serde(default)]
        workspace_exceeded: bool,
        #[serde(default)]
        target: Option<serde_json::Value>,
        #[serde(default)]
        reason: Option<String>,
    },
    TurnCompleted {
        #[serde(default)]
        target_yaw: f64,
        #[serde(default)]
        actual_yaw: f64,
        #[serde(default)]
        success: bool,
    },

    // SENSORY: Voice & Interaction
    VoiceDetected {
        transcription: String,
        confidence: Option<f32>,
    },
    VoiceIntent {
        intent: String,
        entities: serde_json::Value,
    },

    // ACTUATION: Actions Requested by Brain or Reflex Loops
    ActionRequested(RobotAction),

    // SYSTEM / LIFECYCLE
    AdapterConnected {
        name: String,
    },
    AdapterHeartbeat {
        name: String,
    },
    ErrorDetected {
        error: String,
        context: Option<String>,
    },
    Custom {
        name: String,
        payload: serde_json::Value,
    },
}

/// Canonical Event Envelope passing through Tokio EventBus
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub id: Uuid,
    #[serde(default = "Uuid::new_v4")]
    pub trace_id: Uuid,
    #[serde(default)]
    pub session_id: Option<String>,
    pub timestamp: i64,
    pub source: EventSource,
    pub observed_by: String,
    pub event_type: EventType,
    #[serde(default)]
    pub parent_event_id: Option<Uuid>,
}

impl Event {
    pub fn new(source: EventSource, observed_by: impl Into<String>, event_type: EventType) -> Self {
        Self {
            id: Uuid::new_v4(),
            trace_id: Uuid::new_v4(),
            session_id: None,
            timestamp: Utc::now().timestamp(),
            source,
            observed_by: observed_by.into(),
            event_type,
            parent_event_id: None,
        }
    }
}

/// Canonical Persistent World Object State in MNSE
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct WorldObject {
    pub object_id: String,
    pub class_name: String,
    pub confidence: f32,
    #[serde(default)]
    pub max_confidence: f32,
    pub world_x: f64,
    pub world_y: f64,
    pub world_z: f64,
    pub position: [f64; 3],
    pub first_seen: f64,
    pub last_seen: f64,
    pub observation_count: u32,
    pub currently_visible: bool,
    pub source: String,
    pub frame_id: String,
    #[serde(default)]
    pub bbox: Option<Vec<f32>>,
    #[serde(default)]
    pub depth_m: Option<f32>,
}

impl WorldObject {
    pub fn new(
        object_id: String,
        class_name: String,
        confidence: f32,
        x: f64,
        y: f64,
        z: f64,
        timestamp: f64,
        source: String,
        frame_id: String,
        bbox: Option<Vec<f32>>,
        depth_m: Option<f32>,
    ) -> Self {
        Self {
            object_id,
            class_name,
            confidence,
            max_confidence: confidence,
            world_x: x,
            world_y: y,
            world_z: z,
            position: [x, y, z],
            first_seen: timestamp,
            last_seen: timestamp,
            observation_count: 1,
            currently_visible: true,
            source,
            frame_id,
            bbox,
            depth_m,
        }
    }

    /// 3D Euclidean distance to target world coordinates
    pub fn distance_to(&self, x: f64, y: f64, z: f64) -> f64 {
        let dx = self.world_x - x;
        let dy = self.world_y - y;
        let dz = self.world_z - z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Update object state on new observation
    pub fn update(
        &mut self,
        confidence: f32,
        detected_x: f64,
        detected_y: f64,
        detected_z: f64,
        timestamp: f64,
        source: String,
        frame_id: String,
        bbox: Option<Vec<f32>>,
        depth_m: Option<f32>,
    ) {
        self.observation_count += 1;
        self.last_seen = timestamp;
        self.currently_visible = true;
        self.confidence = confidence;
        if confidence > self.max_confidence {
            self.max_confidence = confidence;
        }

        // Smoothed position update: 0.7 * detected + 0.3 * previous
        self.world_x = 0.7 * detected_x + 0.3 * self.world_x;
        self.world_y = 0.7 * detected_y + 0.3 * self.world_y;
        self.world_z = 0.7 * detected_z + 0.3 * self.world_z;
        self.position = [self.world_x, self.world_y, self.world_z];

        self.source = source;
        self.frame_id = frame_id;
        if bbox.is_some() {
            self.bbox = bbox;
        }
        if depth_m.is_some() {
            self.depth_m = depth_m;
        }
    }

    /// Evaluate visibility lifecycle dynamically against a reference timestamp
    pub fn is_visible_at(&self, current_time: f64, timeout_sec: f64) -> bool {
        (current_time - self.last_seen) <= timeout_sec
    }
}

#[cfg(test)]
mod event_contract_tests {
    use super::{Event, EventSource, EventType};
    use serde_json::json;
    use uuid::Uuid;

    #[test]
    fn event_preserves_adapter_trace_through_kernel_contract() {
        let trace_id = Uuid::new_v4();
        let input = json!({
            "id": Uuid::new_v4(),
            "trace_id": trace_id,
            "session_id": "test-session",
            "timestamp": 123,
            "source": "Brain",
            "observed_by": "gemini_robotics_er",
            "event_type": {
                "type": "ActionRequested",
                "data": {"action": "StartMission", "params": {
                    "mission_id": "mission-test-1",
                    "mission_name": "Kitchen route",
                    "waypoints": [{"name": "Kitchen & Dining", "coords": {"x": -5.0, "y": -2.5}}]
                }}
            },
            "parent_event_id": null
        });

        let event: Event = serde_json::from_value(input).expect("adapter action must deserialize");
        assert_eq!(event.trace_id, trace_id);
        assert_eq!(event.session_id.as_deref(), Some("test-session"));

        let output = serde_json::to_value(event).expect("event must serialize for SSE");
        assert_eq!(output["trace_id"], json!(trace_id));
        assert_eq!(
            output["event_type"]["data"]["params"]["mission_id"],
            "mission-test-1"
        );
    }

    #[test]
    fn legacy_events_get_a_usable_trace_id_and_new_events_parse() {
        let legacy = json!({
            "id": Uuid::new_v4(),
            "timestamp": 123,
            "source": "Interaction",
            "observed_by": "interaction_bridge",
            "event_type": {
                "type": "AutonomyStateChanged",
                "data": {
                    "state": "MISSION_COMPLETED",
                    "previous_state": "MISSION_EXECUTING",
                    "reason": "goal reached",
                    "mission_id": "mission-1"
                }
            }
        });

        let event: Event =
            serde_json::from_value(legacy).expect("coordinator state must deserialize");
        assert_ne!(event.trace_id, Uuid::nil());
        assert!(matches!(event.source, EventSource::Interaction));
        assert!(matches!(
            event.event_type,
            EventType::AutonomyStateChanged { .. }
        ));
    }
}
