//! K stages with real executors in the kernel process.
use crate::api::AppState;
use gracemo_k::{ExecutionSummary, KRuntime, KValue, RuntimeError};

pub const MISSION_STATE: &str = r#"memory::recall[query: "autonomy_state"]"#;

pub async fn execute(state: &AppState, expression: &str) -> Result<ExecutionSummary, String> {
    let plan = gracemo_k::plan_expression(expression)?;
    let snapshot = state.live_state.read().await.clone();
    KRuntime::new()
        .execute_with(&plan, |planned, _| {
            let stage = &planned.stage;
            let fail = |msg: &str| RuntimeError::StageFailed(msg.to_owned());
            let output = match (stage.noun.as_str(), stage.verb.as_str()) {
                ("memory", "recall")
                    if stage.args.get("query") == Some(&KValue::from("autonomy_state")) =>
                {
                    KValue::from(
                        snapshot["autonomy_state"]
                            .as_str()
                            .ok_or_else(|| fail("Unknown autonomy state"))?,
                    )
                }
                ("memory", "recall") if stage.args.get("query") == Some(&KValue::from("visible_objects"))
                    || stage.args.get("query") == Some(&KValue::from("scene_changes"))
                    || stage.args.get("query") == Some(&KValue::from("person_present")) => {
                    let sample = &snapshot["perception"]["sample"];
                    let age = chrono::Utc::now().timestamp_millis() as f64 / 1000.0
                        - sample["captured_at"].as_f64().ok_or_else(|| fail("Missing capture time"))?;
                    if sample["ready"] != true || !(0.0..=3.0).contains(&age) {
                        return Err(fail("No fresh RGB-D observation"));
                    }
                    if stage.args.get("query") == Some(&KValue::from("person_present")) {
                        let tracks = sample["person_tracks"].as_array().ok_or_else(|| fail("Person observation unavailable"))?;
                        KValue::Bool(!tracks.is_empty())
                    } else if stage.args.get("query") == Some(&KValue::from("scene_changes")) {
                        use std::collections::BTreeSet;
                        let before = snapshot["perception"]["previous_classes"].as_array()
                            .ok_or_else(|| fail("No comparable recent view"))?
                            .iter().filter_map(|v| v.as_str()).collect::<BTreeSet<_>>();
                        let after = sample["observations"].as_array().ok_or_else(|| fail("No current observation"))?
                            .iter().filter_map(|v| v["class_name"].as_str()).collect::<BTreeSet<_>>();
                        KValue::Map([
                            ("entered_view".into(), KValue::List(after.difference(&before).map(|v| KValue::from(*v)).collect())),
                            ("left_view".into(), KValue::List(before.difference(&after).map(|v| KValue::from(*v)).collect())),
                        ].into_iter().collect())
                    } else {
                        let objects = sample["observations"].as_array().ok_or_else(|| fail("Object observation unavailable"))?;
                        KValue::List(objects.iter().filter_map(|obj| obj["class_name"].as_str().map(KValue::from)).collect())
                    }
                }
                ("memory", "recall") if stage.args.get("query") == Some(&KValue::from("last_observation")) => {
                    let Some(KValue::String(entity)) = stage.args.get("entity") else {
                        return Err(fail("Expected a class or object ID"));
                    };
                    let objects = state.graph.query_world_objects(None, false)
                        .map_err(|e| fail(&e.to_string()))?;
                    let mut matches = objects.iter().filter(|o|
                        o.object_id == *entity || o.class_name.eq_ignore_ascii_case(entity));
                    let obj = matches.next().ok_or_else(|| fail("Missing observation"))?;
                    if matches.next().is_some() {
                        return Err(fail("Ambiguous entity; specify object ID"));
                    }
                    let observation = state.graph.query_visual_observations(Some(&obj.object_id), false, 1)
                        .map_err(|e| fail(&e.to_string()))?.into_iter().next();
                    let observation_id = observation.as_ref().and_then(|v| v["observation_id"].as_str())
                        .ok_or_else(|| fail("No retained visual observation"))?;
                    KValue::Map([
                        ("object_id".into(), KValue::from(obj.object_id.clone())),
                        ("observation_id".into(), KValue::from(observation_id)),
                        ("event_time".into(), KValue::Number(obj.last_seen)),
                        ("recently_observed".into(), KValue::Bool(obj.currently_visible)),
                        ("observation_age_s".into(), KValue::Number(
                            (chrono::Utc::now().timestamp_millis() as f64 / 1000.0 - obj.last_seen).max(0.0))),
                        ("fresh_within_3s".into(), KValue::Bool(
                            chrono::Utc::now().timestamp_millis() as f64 / 1000.0 - obj.last_seen <= 3.0)),
                        ("confidence".into(), KValue::Number(obj.confidence as f64)),
                        ("frame".into(), KValue::from(obj.frame_id.clone())),
                        ("source".into(), KValue::from(obj.source.clone())),
                        ("last_position".into(), KValue::Coordinates {
                            x: obj.world_x, y: obj.world_y, z: Some(obj.world_z) }),
                    ].into_iter().collect())
                }
                ("memory", "recall") if stage.args.get("query") == Some(&KValue::from("visual_history")) => {
                    let Some(KValue::String(entity)) = stage.args.get("entity") else {
                        return Err(fail("Expected an object class or object ID"));
                    };
                    let objects = state.graph.query_world_objects(None, false)
                        .map_err(|e| fail(&e.to_string()))?;
                    let matches = objects.iter().filter(|o|
                        o.object_id == *entity || o.class_name.eq_ignore_ascii_case(entity)).collect::<Vec<_>>();
                    if matches.is_empty() { return Err(fail("No retained visual observations")); }
                    let mut history = Vec::new();
                    for object in matches {
                        let observations = state.graph.query_visual_observations(Some(&object.object_id), true, 100)
                            .map_err(|e| fail(&e.to_string()))?;
                        for item in observations {
                            let position = &item["position"];
                            let mut row = std::collections::HashMap::from([
                                ("observation_id".into(), KValue::from(item["observation_id"].as_str().unwrap_or(""))),
                                ("object_id".into(), KValue::from(item["object_id"].as_str().unwrap_or(""))),
                                ("class_name".into(), KValue::from(item["class_name"].as_str().unwrap_or(""))),
                                ("captured_at".into(), KValue::Number(item["captured_at"].as_f64().unwrap_or_default())),
                                ("confidence".into(), KValue::Number(item["confidence"].as_f64().unwrap_or_default())),
                                ("status".into(), KValue::from(item["status"].as_str().unwrap_or("unknown"))),
                                ("trace_id".into(), KValue::from(item["trace_id"].as_str().unwrap_or(""))),
                                ("position".into(), KValue::Coordinates { x:position["x"].as_f64().unwrap_or_default(),
                                    y:position["y"].as_f64().unwrap_or_default(),z:position["z"].as_f64() }),
                            ]);
                            if let Some(mission) = item["mission_id"].as_str() {
                                row.insert("mission_id".into(), KValue::from(mission));
                            }
                            history.push(KValue::Map(row));
                        }
                    }
                    KValue::List(history)
                }
                ("graph", "locate") => {
                    let Some(KValue::String(name)) = stage.args.get("entity") else {
                        return Err(fail("Expected an entity name"));
                    };
                    if stage.args.len() != 1 {
                        return Err(fail("Unsupported locate constraint"));
                    }
                    let objects = state
                        .graph
                        .query_world_objects(None, true)
                        .map_err(|e| fail(&e.to_string()))?;
                    let mut matches = objects.iter().filter(|o| {
                        o.object_id == *name || o.class_name.eq_ignore_ascii_case(name)
                    });
                    let obj = matches
                        .next()
                        .ok_or_else(|| fail("No current matching observation"))?;
                    if matches.next().is_some() {
                        return Err(fail("Ambiguous entity; specify object ID"));
                    }
                    KValue::Coordinates {
                        x: obj.world_x,
                        y: obj.world_y,
                        z: Some(obj.world_z),
                    }
                }
                _ => {
                    return Err(fail(&format!(
                        "No live executor for {}::{}",
                        stage.noun, stage.verb
                    )))
                }
            };
            Ok(Some(output))
        })
        .map_err(|e| e.to_string())
}
