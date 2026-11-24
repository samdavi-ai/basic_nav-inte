//! Operator-provisioned adapter grants. A receipt references an exact, immutable
//! kernel-held request and can be consumed only by the same authenticated adapter.
use crate::api::{publish_event, AppState};
use axum::{
    extract::State,
    http::{HeaderMap, StatusCode},
    Json,
};
use chrono::Utc;
use gracemo_types::{Event, EventType};
use serde::Deserialize;
use serde_json::{json, Value};
use std::{
    collections::HashMap,
    fs,
    os::unix::fs::PermissionsExt,
    sync::{Arc, Mutex},
};
use uuid::Uuid;

#[derive(Clone, Deserialize, PartialEq)]
pub struct Grant {
    id: String,
    token: String,
    actions: Vec<String>,
    #[serde(default)]
    adapter_type: Option<String>,
}

pub struct Registry {
    grants: Mutex<Vec<Grant>>,
    grant_path: Option<std::path::PathBuf>,
    transition: tokio::sync::Mutex<()>,
    pending: Mutex<HashMap<Uuid, (Event, Value)>>,
    issuer: Uuid,
    sessions: Mutex<HashMap<String, (Uuid, i64)>>,
}
impl Registry {
    fn read_grants(path: Option<&std::path::Path>) -> anyhow::Result<Vec<Grant>> {
        let grants: Vec<Grant> = match path {
            None => Vec::new(),
            Some(path) => {
                let meta = fs::symlink_metadata(&path)?;
                anyhow::ensure!(
                    meta.is_file() && meta.permissions().mode() & 0o077 == 0,
                    "grant file must be private"
                );
                serde_json::from_slice(&fs::read(path)?)?
            }
        };
        for (i, g) in grants.iter().enumerate() {
            anyhow::ensure!(
                !g.id.is_empty()
                    && g.token.len() >= 32
                    && !grants[..i]
                        .iter()
                        .any(|other| other.id == g.id || other.token == g.token),
                "invalid adapter provisioning"
            );
        }
        Ok(grants)
    }
    pub fn load() -> anyhow::Result<Arc<Self>> {
        let grant_path = std::env::var_os("VIRA_ADAPTER_GRANTS").map(std::path::PathBuf::from);
        let grants = Self::read_grants(grant_path.as_deref())?;
        Ok(Arc::new(Self {
            transition: tokio::sync::Mutex::new(()),
            sessions: Mutex::new(HashMap::new()),
            grants: Mutex::new(grants),
            grant_path,
            pending: Mutex::new(HashMap::new()),
            issuer: Uuid::new_v4(),
        }))
    }
    fn principal(&self, h: &HeaderMap) -> Result<Grant, StatusCode> {
        let bearer = h
            .get("authorization")
            .and_then(|s| s.to_str().ok())
            .and_then(|s| s.strip_prefix("Bearer "))
            .ok_or(StatusCode::UNAUTHORIZED)?;
        // A replaced, deleted or malformed policy must never leave old grants usable.
        let loaded = Self::read_grants(self.grant_path.as_deref())
            .map_err(|_| StatusCode::SERVICE_UNAVAILABLE)?;
        let mut current = self
            .grants
            .lock()
            .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
        if *current != loaded {
            self.pending
                .lock()
                .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
                .clear();
            self.sessions
                .lock()
                .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
                .clear();
            *current = loaded;
        }
        current
            .iter()
            .find(|g| g.token == bearer)
            .cloned()
            .ok_or(StatusCode::UNAUTHORIZED)
    }

}

impl AppState {
    /// Require an authenticated, connected grant with a narrowly scoped capability.
    pub fn require_capability(&self, headers: &HeaderMap, capability: &str) -> Result<String, StatusCode> {
        let grant = self.registry.principal(headers)?;
        if !grant.actions.iter().any(|action| action == capability) {
            return Err(StatusCode::FORBIDDEN);
        }
        session(self, &grant)?;
        Ok(grant.id)
    }
}
// libcrypto is supplied by the supported Ubuntu/Jazzy environment.
#[link(name = "crypto")]
unsafe extern "C" {
    fn SHA256(data: *const u8, len: usize, output: *mut u8) -> *mut u8;
}

fn parameter_digest(parameters: &Value) -> Result<String, StatusCode> {
    let bytes = serde_json::to_vec(parameters).map_err(|_| StatusCode::BAD_REQUEST)?;
    let mut hash = [0u8; 32];
    // Both buffers remain alive and the output is the SHA-256 digest size.
    let result = unsafe { SHA256(bytes.as_ptr(), bytes.len(), hash.as_mut_ptr()) };
    if result.is_null() {
        return Err(StatusCode::INTERNAL_SERVER_ERROR);
    }
    Ok(hash.iter().map(|b| format!("{b:02x}")).collect())
}

pub async fn connect(State(s): State<AppState>, h: HeaderMap) -> Result<Json<Value>, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let g = s.registry.principal(&h)?;
    let now = Utc::now().timestamp();
    let mut sessions = s
        .registry
        .sessions
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    let lease = sessions.entry(g.id.clone()).or_insert((Uuid::new_v4(), 0));
    if lease.1 <= now {
        lease.0 = Uuid::new_v4();
    }
    lease.1 = now + 30;
    Ok(Json(
        json!({"adapter_id":g.id,"adapter_type":g.adapter_type,"capabilities":g.actions,
        "session":lease.0,"expires_at":lease.1}),
    ))
}

pub async fn disconnect(State(s): State<AppState>, h: HeaderMap) -> Result<StatusCode, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let g = s.registry.principal(&h)?;
    s.registry
        .sessions
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
        .remove(&g.id);
    Ok(StatusCode::NO_CONTENT)
}

fn session(s: &AppState, g: &Grant) -> Result<Uuid, StatusCode> {
    let sessions = s
        .registry
        .sessions
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    sessions
        .get(&g.id)
        .filter(|(_, expires)| *expires > Utc::now().timestamp())
        .map(|(id, _)| *id)
        .ok_or(StatusCode::FORBIDDEN)
}

fn check(g: &Grant, event: &Event) -> Result<Value, StatusCode> {
    if g.id != event.observed_by || event.trace_id.is_nil() || event.id.is_nil() {
        return Err(StatusCode::FORBIDDEN);
    }
    let EventType::ActionRequested(action) = &event.event_type else {
        return Err(StatusCode::BAD_REQUEST);
    };
    action
        .validate()
        .map_err(|_| StatusCode::UNPROCESSABLE_ENTITY)?;
    let value = serde_json::to_value(action).map_err(|_| StatusCode::BAD_REQUEST)?;
    let kind = value["action"].as_str().ok_or(StatusCode::BAD_REQUEST)?;
    if !g.actions.iter().any(|a| a == kind) {
        return Err(StatusCode::FORBIDDEN);
    }
    if kind == "StartMission" {
        let params = &value["params"];
        if params["mission_id"]
            .as_str()
            .is_none_or(|s| s.trim().is_empty())
            || params["waypoints"].as_array().is_none_or(|w| {
                w.is_empty()
                    || w.iter().any(|p| {
                        !["x", "y"]
                            .iter()
                            .all(|k| p["coords"][k].as_f64().is_some_and(f64::is_finite))
                    })
            })
        {
            return Err(StatusCode::UNPROCESSABLE_ENTITY);
        }
    }
    Ok(value)
}
async fn check_safety(s: &AppState, action: &Value) -> Result<(), StatusCode> {
    if matches!(
        action["action"].as_str(),
        Some("Stop" | "Cancel" | "VoiceInterrupt" | "StatusQuery")
    ) {
        return Ok(());
    }
    if !matches!(
        action["action"].as_str(),
        Some("Speak" | "Express" | "Pause")
    ) {
        for prior in s
            .recovered_missions
            .iter()
            .filter(|r| r["requires_operator_review"] == true)
        {
            if !s
                .ledger
                .reviewed(
                    prior["action_id"]
                        .as_str()
                        .ok_or(StatusCode::INTERNAL_SERVER_ERROR)?,
                )
                .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
            {
                return Err(StatusCode::CONFLICT);
            }
        }
    }
    if !matches!(action["action"].as_str(), Some("Speak" | "Express" | "Pause"))
        && s.ledger.emergency_hold().map_err(|_| StatusCode::SERVICE_UNAVAILABLE)?.is_some()
    {
        return Err(StatusCode::LOCKED);
    }
    let state = if action["action"]=="StartMission" {
        let summary=crate::k_execution::execute(s,crate::k_execution::MISSION_STATE).await
            .map_err(|_|StatusCode::SERVICE_UNAVAILABLE)?;
        match summary.stage_results.into_iter().next().and_then(|r|r.output) {
            Some(gracemo_k::KValue::String(value)) => Value::String(value),
            _ => return Err(StatusCode::SERVICE_UNAVAILABLE),
        }
    } else { s.live_state.read().await["autonomy_state"].clone() };
    let permitted = match action["action"].as_str() {
        Some("Resume" | "VoiceResume") => state == "MISSION_PAUSED",
        Some("Pause") => matches!(
            state.as_str(),
            Some("MISSION_EXECUTING" | "MISSION_PLANNING" | "OBSTACLE_HANDLING")
        ),
        Some("Speak" | "Express") => true,
        _ => matches!(
            state.as_str(),
            Some("AUTONOMY_READY" | "WAITING_FOR_NEXT_MISSION")
        ),
    };
    if !permitted {
        return Err(StatusCode::FORBIDDEN);
    }
    Ok(())
}
pub async fn authorize(
    State(s): State<AppState>,
    h: HeaderMap,
    Json(event): Json<Event>,
) -> Result<Json<Value>, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let g = s.registry.principal(&h)?;
    let decision = async {
        let generation = session(&s, &g)?;
        let action = check(&g, &event)?;
        check_safety(&s, &action).await?;
        Ok::<_, StatusCode>((generation, action))
    }
    .await;
    let (generation, action) = decision.map_err(|reason| {
        s.ledger
            .record_decision(&event, &g.id, "DENIED", &json!({"status":reason.as_u16()}))
            .map(|_| reason)
            .unwrap_or(StatusCode::INTERNAL_SERVER_ERROR)
    })?;
    let active = s.live_state.read().await["active_mission"].clone();
    let mission = action["params"]
        .get("mission_id")
        .cloned()
        .unwrap_or_else(|| active["id"].clone());
    let now = Utc::now().timestamp();
    let id = Uuid::new_v4();
    let receipt = json!({"authorization_id":id,"action_id":event.id,"adapter_id":g.id,
        "trace_id":event.trace_id,"mission_id":mission,"active_mission_trace_id":active["trace_id"],"capability":action["action"],
        "scope":"single_dispatch","parameter_sha256":parameter_digest(&action["params"])?,"adapter_session":generation,
        "canonical_action":action,"issued_at":now,"expires_at":now+5,"issuer":s.registry.issuer,"policy_version":1});
    let mut pending = s
        .registry
        .pending
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    pending.retain(|_, (_, r)| r["expires_at"].as_i64().unwrap_or(0) > now);
    if pending.len() >= 1024 {
        return Err(StatusCode::TOO_MANY_REQUESTS);
    }
    if pending.values().any(|(stored, _)| stored.id == event.id) {
        return Err(StatusCode::CONFLICT);
    }
    s.ledger
        .record_decision(&event, &g.id, "AUTHORIZED", &receipt)
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    pending.insert(id, (event, receipt.clone()));
    Ok(Json(receipt))
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Dispatch {
    authorization_id: Uuid,
}
pub async fn dispatch(
    State(s): State<AppState>,
    h: HeaderMap,
    Json(req): Json<Dispatch>,
) -> Result<Json<Value>, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let g = s.registry.principal(&h)?;
    let (event, receipt) = {
        let mut pending = s
            .registry
            .pending
            .lock()
            .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
        let (event, r) = pending
            .get(&req.authorization_id)
            .ok_or(StatusCode::CONFLICT)?;
        check(&g, event)?;
        if r["adapter_session"] != json!(session(&s, &g)?) {
            return Err(StatusCode::FORBIDDEN);
        }
        if r["expires_at"].as_i64().unwrap_or(0) <= Utc::now().timestamp() {
            return Err(StatusCode::GONE);
        }
        pending
            .remove(&req.authorization_id)
            .ok_or(StatusCode::CONFLICT)?
    };
    // Persist before broadcasting. A repeated event ID cannot dispatch twice,
    // even under a fresh receipt or after restarting the kernel.
    check_safety(&s, &receipt["canonical_action"]).await?;
    s.ledger
        .record_with_receipt(&event, Some(&receipt))
        .map_err(|_| StatusCode::CONFLICT)?;
    if matches!(
        receipt["capability"].as_str(),
        Some("Stop" | "Cancel" | "Pause" | "VoiceInterrupt")
    ) {
        s.registry
            .pending
            .lock()
            .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
            .retain(|_, (_, r)| {
                matches!(
                    r["capability"].as_str(),
                    Some("Stop" | "Cancel" | "StatusQuery")
                )
            });
    }
    publish_event(s.clone(), event).await;
    Ok(Json(json!({"status":"dispatched","receipt":receipt})))
}

pub async fn decisions(
    State(s): State<AppState>,
    h: HeaderMap,
    axum::extract::Query(q): axum::extract::Query<crate::api::HistoryQuery>,
) -> Result<Json<Vec<Value>>, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let g = s.registry.principal(&h)?;
    s.ledger
        .decisions(&g.id, q.trace_id.as_deref())
        .map(Json)
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Review {
    action_id: Uuid,
    trace_id: Uuid,
    mission_id: String,
}

pub async fn abandon(
    State(s): State<AppState>,
    h: HeaderMap,
    Json(request): Json<Review>,
) -> Result<StatusCode, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let operator = s.registry.principal(&h)?;
    session(&s, &operator)?;
    if !operator
        .actions
        .iter()
        .any(|a| a == "ReviewInterruptedMission")
    {
        return Err(StatusCode::FORBIDDEN);
    }
    let id = request.action_id.to_string();
    let trace = request.trace_id.to_string();
    if !s.recovered_missions.iter().any(|r| {
        r["action_id"] == id
            && r["trace_id"] == trace
            && r["mission_id"] == request.mission_id
            && r["requires_operator_review"] == true
    }) {
        return Err(StatusCode::CONFLICT);
    }
    s.ledger
        .abandon_interrupted(&id, &trace, &request.mission_id, &operator.id)
        .map_err(|_| StatusCode::CONFLICT)?;
    Ok(StatusCode::NO_CONTENT)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReleaseHold { event_id: Uuid }

pub async fn release_hold(State(s): State<AppState>, h: HeaderMap, Json(req): Json<ReleaseHold>)
    -> Result<StatusCode, StatusCode> {
    let _transition = s.registry.transition.lock().await;
    let operator = s.registry.principal(&h)?;
    session(&s, &operator)?;
    if !operator.actions.iter().any(|action| action == "ReleaseEmergencyHold") {
        return Err(StatusCode::FORBIDDEN);
    }
    // This acknowledges the kernel hold only after ROS reports readiness.
    // It does not send a reset to ROS or dispatch an interrupted mission.
    if s.live_state.read().await["autonomy_state"] != "AUTONOMY_READY" {
        return Err(StatusCode::CONFLICT);
    }
    if !s.ledger.release_emergency_hold(&req.event_id.to_string(), &operator.id)
        .map_err(|_| StatusCode::SERVICE_UNAVAILABLE)? {
        return Err(StatusCode::CONFLICT);
    }
    Ok(StatusCode::NO_CONTENT)
}

pub async fn registry_status(State(s):State<AppState>)->Result<Json<Value>,StatusCode> {
    let _transition=s.registry.transition.lock().await;
    let grants=Registry::read_grants(s.registry.grant_path.as_deref()).map_err(|_|StatusCode::SERVICE_UNAVAILABLE)?;
    let cached=s.registry.grants.lock().map_err(|_|StatusCode::INTERNAL_SERVER_ERROR)?;
    let sessions=s.registry.sessions.lock().map_err(|_|StatusCode::INTERNAL_SERVER_ERROR)?;
    let now=Utc::now().timestamp();
    let rows:Vec<_>=grants.iter().map(|g|{
        let connected=*cached==grants && sessions.get(&g.id).is_some_and(|(_,expiry)|*expiry>now);
        json!({"id":g.id,"capabilities":g.actions,"connected":connected})
    }).collect();
    Ok(Json(json!({"adapters":rows})))
}
