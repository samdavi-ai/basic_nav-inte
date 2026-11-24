use anyhow::Result;
use chrono::Utc;
use gracemo_types::{Event, EventType, WorldObject};
use rusqlite::{params, Connection, OptionalExtension};
use serde_json::{json, Value};
use std::path::Path;
use std::sync::{Arc, Mutex};
use tracing::info;

#[derive(Clone)]
pub struct GraphDb {
    conn: Arc<Mutex<Connection>>,
}

impl GraphDb {
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let conn = Connection::open(path)?;

        // WAL mode for maximum concurrency and durability
        conn.pragma_update(None, "journal_mode", "WAL")?;
        conn.pragma_update(None, "synchronous", "NORMAL")?;
        conn.pragma_update(None, "foreign_keys", "ON")?;

        let db = Self {
            conn: Arc::new(Mutex::new(conn)),
        };
        db.migrate()?;
        db.init_apartment_rooms()?;

        Ok(db)
    }

    fn migrate(&self) -> Result<()> {
        let conn = self.conn.lock().unwrap();

        // 1. Nodes table
        conn.execute(
            "CREATE TABLE IF NOT EXISTS nodes (
                id TEXT PRIMARY KEY,
                type TEXT NOT NULL,
                label TEXT NOT NULL,
                properties JSON,
                created_at INTEGER NOT NULL,
                updated_at INTEGER NOT NULL
            )",
            [],
        )?;

        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_nodes_type ON nodes(type)",
            [],
        )?;

        // 2. Edges table (Relational spatial & causal truth)
        conn.execute(
            "CREATE TABLE IF NOT EXISTS edges (
                source_id TEXT NOT NULL,
                target_id TEXT NOT NULL,
                relation TEXT NOT NULL,
                weight INTEGER DEFAULT 1,
                confidence REAL DEFAULT 1.0,
                properties JSON,
                created_at INTEGER NOT NULL,
                updated_at INTEGER NOT NULL,
                PRIMARY KEY (source_id, target_id, relation),
                FOREIGN KEY(source_id) REFERENCES nodes(id) ON DELETE CASCADE,
                FOREIGN KEY(target_id) REFERENCES nodes(id) ON DELETE CASCADE
            )",
            [],
        )?;

        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_edges_source ON edges(source_id)",
            [],
        )?;
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_edges_target ON edges(target_id)",
            [],
        )?;

        conn.execute_batch("CREATE TABLE IF NOT EXISTS visual_observations (
            observation_id TEXT PRIMARY KEY,
            object_id TEXT NOT NULL,
            class_name TEXT NOT NULL,
            captured_at INTEGER NOT NULL,
            image_stamp_ns INTEGER,
            source_camera TEXT,
            coordinate_frame TEXT NOT NULL,
            x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
            confidence REAL NOT NULL,
            detection_id TEXT, track_id TEXT,
            mission_id TEXT, trace_id TEXT NOT NULL,
            provenance_source TEXT NOT NULL,
            observed_by TEXT NOT NULL,
            map_version TEXT,
            status TEXT NOT NULL DEFAULT 'active',
            superseded_by TEXT,
            deleted_at INTEGER, deleted_by TEXT, delete_reason TEXT
        );
        CREATE INDEX IF NOT EXISTS visual_observations_object_time
            ON visual_observations(object_id,captured_at DESC);
        CREATE INDEX IF NOT EXISTS visual_observations_status_time
            ON visual_observations(status,captured_at DESC);")?;

        Ok(())
    }

    fn visual_retention_cutoff() -> i64 {
        let days = std::env::var("VIRA_VISUAL_MEMORY_RETENTION_DAYS")
            .ok().and_then(|v| v.parse::<i64>().ok()).filter(|v| (1..=3650).contains(v)).unwrap_or(30);
        Utc::now().timestamp().saturating_sub(days.saturating_mul(86_400))
    }

    fn expire_visual_observations(&self) -> Result<()> {
        let cutoff = Self::visual_retention_cutoff();
        let conn = self.conn.lock().map_err(|_| anyhow::anyhow!("graph database unavailable"))?;
        conn.execute("UPDATE visual_observations SET status='expired' WHERE captured_at < ?1 AND status IN ('active','superseded')", [cutoff])?;
        conn.execute("DELETE FROM nodes WHERE type='WorldObject' AND CAST(json_extract(properties,'$.last_seen') AS REAL) < ?1", [cutoff as f64])?;
        conn.execute("DELETE FROM nodes WHERE type='Object' AND NOT EXISTS (
            SELECT 1 FROM nodes w WHERE w.type='WorldObject'
            AND lower(json_extract(w.properties,'$.class_name'))=lower(nodes.label))", [])?;
        Ok(())
    }

    fn record_visual_observation(
        &self, event: &Event, object_id: &str, class_name: &str, confidence: f32,
        x: f64, y: f64, z: f64, coordinate_frame: Option<&str>, source_camera: Option<&str>,
        detection_id: Option<&str>, track_id: Option<&str>, image_stamp_ns: Option<i64>,
    ) -> Result<()> {
        let mut conn = self.conn.lock().map_err(|_| anyhow::anyhow!("graph database unavailable"))?;
        let tx = conn.transaction()?;
        let observation_id = event.id.to_string();
        let exists: bool = tx.query_row("SELECT EXISTS(SELECT 1 FROM visual_observations WHERE observation_id=?1)",
            [&observation_id], |row| row.get(0))?;
        if exists { return Ok(()); }
        tx.execute("UPDATE visual_observations SET status='superseded', superseded_by=?1
            WHERE object_id=?2 AND status='active'", params![observation_id, object_id])?;
        tx.execute("INSERT INTO visual_observations(
            observation_id,object_id,class_name,captured_at,image_stamp_ns,source_camera,coordinate_frame,
            x,y,z,confidence,detection_id,track_id,mission_id,trace_id,provenance_source,observed_by,map_version,status)
            VALUES (?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15,?16,?17,NULL,'active')",
            params![observation_id,object_id,class_name,event.timestamp,image_stamp_ns,source_camera,
                coordinate_frame.unwrap_or("world"),x,y,z,confidence,detection_id,track_id,
                event.session_id.as_deref(),event.trace_id.to_string(),event.source.to_string(),event.observed_by])?;
        tx.commit()?;
        Ok(())
    }

    /// Return object observations. Deleted and expired records are never exposed by this view.
    pub fn query_visual_observations(&self, object_id: Option<&str>, include_history: bool, limit: usize) -> Result<Vec<Value>> {
        self.expire_visual_observations()?;
        let conn = self.conn.lock().map_err(|_| anyhow::anyhow!("graph database unavailable"))?;
        let statuses = if include_history { "status IN ('active','superseded')" } else { "status='active'" };
        let sql = format!("SELECT observation_id,object_id,class_name,captured_at,image_stamp_ns,source_camera,
            coordinate_frame,x,y,z,confidence,detection_id,track_id,mission_id,trace_id,provenance_source,
            observed_by,map_version,status,superseded_by FROM visual_observations WHERE {statuses}
            AND (?1 IS NULL OR object_id=?1) ORDER BY captured_at DESC,observation_id DESC LIMIT ?2");
        let mut stmt = conn.prepare(&sql)?;
        let rows = stmt.query_map(params![object_id, limit.clamp(1,500) as i64], |row| {
            Ok(json!({
                "observation_id":row.get::<_,String>(0)?, "object_id":row.get::<_,String>(1)?,
                "class_name":row.get::<_,String>(2)?, "captured_at":row.get::<_,i64>(3)?,
                "image_stamp_ns":row.get::<_,Option<i64>>(4)?, "source_camera":row.get::<_,Option<String>>(5)?,
                "coordinate_frame":row.get::<_,String>(6)?, "position":{"x":row.get::<_,f64>(7)?,"y":row.get::<_,f64>(8)?,"z":row.get::<_,f64>(9)?},
                "confidence":row.get::<_,f64>(10)?, "detection_id":row.get::<_,Option<String>>(11)?,
                "track_id":row.get::<_,Option<String>>(12)?, "mission_id":row.get::<_,Option<String>>(13)?,
                "trace_id":row.get::<_,String>(14)?, "provenance":{"source":row.get::<_,String>(15)?,"observed_by":row.get::<_,String>(16)?},
                "map_version":row.get::<_,Option<String>>(17)?, "status":row.get::<_,String>(18)?,
                "superseded_by":row.get::<_,Option<String>>(19)?
            }))
        })?.collect::<std::result::Result<Vec<_>,_>>()?;
        Ok(rows)
    }

    /// Deleted and expired detections must not reappear through the event ledger views.
    pub fn visual_observation_hidden(&self, observation_id: &str) -> Result<bool> {
        let conn = self.conn.lock().map_err(|_| anyhow::anyhow!("graph database unavailable"))?;
        Ok(conn.query_row("SELECT EXISTS(SELECT 1 FROM visual_observations
            WHERE observation_id=?1 AND status IN ('deleted','expired'))", [observation_id], |row| row.get(0))?)
    }

    /// Tombstone a visual observation while retaining a non-public audit record.
    pub fn delete_visual_observation(&self, observation_id: &str, operator: &str, reason: &str) -> Result<bool> {
        let mut conn = self.conn.lock().map_err(|_| anyhow::anyhow!("graph database unavailable"))?;
        let tx = conn.transaction()?;
        let entity: Option<(String,String)> = tx.query_row(
            "SELECT object_id,class_name FROM visual_observations WHERE observation_id=?1 AND status IN ('active','superseded')",
            [observation_id], |row| Ok((row.get(0)?,row.get(1)?))).optional()?;
        let Some((object_id,class_name)) = entity else { return Ok(false); };
        tx.execute("UPDATE visual_observations SET status='deleted',deleted_at=?1,deleted_by=?2,delete_reason=?3
            WHERE observation_id=?4", params![Utc::now().timestamp(),operator,reason,observation_id])?;
        let active: bool = tx.query_row("SELECT EXISTS(SELECT 1 FROM visual_observations WHERE object_id=?1 AND status='active')",
            [&object_id], |row| row.get(0))?;
        if !active {
            tx.execute("DELETE FROM nodes WHERE id=?1 AND type='WorldObject'", [format!("object:{object_id}")])?;
            let remaining_class: bool = tx.query_row("SELECT EXISTS(SELECT 1 FROM nodes WHERE type='WorldObject'
                AND lower(json_extract(properties,'$.class_name'))=lower(?1))", [&class_name], |row| row.get(0))?;
            if !remaining_class {
                tx.execute("DELETE FROM nodes WHERE id=?1 AND type='Object'", [format!("object:{}",class_name.to_lowercase().trim())])?;
            }
        }
        tx.commit()?;
        Ok(true)
    }

    /// Seed the canonical physical apartment topology
    fn init_apartment_rooms(&self) -> Result<()> {
        let rooms = vec![
            ("room:bedroom", "Room", "Master Bedroom", json!({
                "x_min": -9.0, "x_max": 0.0, "y_min": 1.0, "y_max": 7.0,
                "portal_id": "portal:bedroom", "target_x": -5.0, "target_y": 2.8,
                "provenance": "seeded_topology",
                "explored": false,
                "scan_count": 0,
                "first_explored": serde_json::Value::Null,
                "last_explored": serde_json::Value::Null
            })),
            ("room:study",   "Room", "Home Study", json!({
                "x_min": 0.0, "x_max": 9.0, "y_min": 1.0, "y_max": 7.0,
                "portal_id": "portal:study", "target_x": 4.0, "target_y": 3.8,
                "provenance": "seeded_topology",
                "explored": false,
                "scan_count": 0,
                "first_explored": serde_json::Value::Null,
                "last_explored": serde_json::Value::Null
            })),
            ("room:kitchen", "Room", "Kitchen & Dining", json!({
                "x_min": -9.0, "x_max": 0.0, "y_min": -7.0, "y_max": -1.0,
                "portal_id": "portal:kitchen", "target_x": -4.5, "target_y": -2.5,
                "provenance": "seeded_topology",
                "explored": false,
                "scan_count": 0,
                "first_explored": serde_json::Value::Null,
                "last_explored": serde_json::Value::Null
            })),
            ("room:living",  "Room", "Living Room", json!({
                "x_min": 0.0, "x_max": 9.0, "y_min": -7.0, "y_max": -1.0,
                "portal_id": "portal:living", "target_x": 4.0, "target_y": -2.5,
                "provenance": "seeded_topology",
                "explored": false,
                "scan_count": 0,
                "first_explored": serde_json::Value::Null,
                "last_explored": serde_json::Value::Null
            })),
            ("room:hallway", "Room", "Central Hallway", json!({
                "x_min": -9.0, "x_max": 9.0, "y_min": -1.0, "y_max": 1.0,
                "target_x": 0.0, "target_y": 0.0,
                "provenance": "seeded_topology",
                "explored": false,
                "scan_count": 0,
                "first_explored": serde_json::Value::Null,
                "last_explored": serde_json::Value::Null
            })),
        ];

        for (id, node_type, label, mut props) in rooms {
            let existing_props: Option<String> = {
                let conn = self.conn.lock().unwrap();
                conn.query_row(
                    "SELECT properties FROM nodes WHERE id = ?1",
                    params![id],
                    |row| row.get(0),
                )
                .ok()
            };
            if let Some(s) = existing_props {
                if let Ok(ex) = serde_json::from_str::<Value>(&s) {
                    if let Some(obj) = ex.as_object() {
                        if let Some(exp) = obj.get("explored") {
                            props["explored"] = exp.clone();
                        }
                        if let Some(sc) = obj.get("scan_count") {
                            props["scan_count"] = sc.clone();
                        }
                        if let Some(fe) = obj.get("first_explored") {
                            props["first_explored"] = fe.clone();
                        }
                        if let Some(le) = obj.get("last_explored") {
                            props["last_explored"] = le.clone();
                        }
                    }
                }
            }
            self.upsert_node(id, node_type, label, Some(props))?;
        }

        // Topological Doorway Portals derived from verified apartment_floor.world geometry
        let portals = vec![
            ("portal:bedroom", "Portal", "Bedroom Doorway", json!({
                "door_x": -5.0, "door_y": 1.0, "width": 1.2,
                "approach_x": -5.0, "approach_y": 0.0, "approach_yaw": 1.5708,
                "threshold_x": -5.0, "threshold_y": 1.4, "threshold_yaw": 1.5708,
                "target_x": -5.0, "target_y": 2.8,
                "connects_from": "room:hallway", "connects_to": "room:bedroom",
                "provenance": "seeded_topology"
            })),
            ("portal:study", "Portal", "Study Doorway", json!({
                "door_x": 5.0, "door_y": 1.0, "width": 1.2,
                "approach_x": 5.0, "approach_y": 0.0, "approach_yaw": 1.5708,
                "threshold_x": 5.0, "threshold_y": 1.4, "threshold_yaw": 1.5708,
                "target_x": 4.0, "target_y": 3.8,
                "connects_from": "room:hallway", "connects_to": "room:study",
                "provenance": "seeded_topology"
            })),
            ("portal:kitchen", "Portal", "Kitchen Doorway", json!({
                "door_x": -5.0, "door_y": -1.0, "width": 1.2,
                "approach_x": -5.0, "approach_y": 0.0, "approach_yaw": -1.5708,
                "threshold_x": -5.0, "threshold_y": -1.4, "threshold_yaw": -1.5708,
                "target_x": -4.5, "target_y": -2.5,
                "connects_from": "room:hallway", "connects_to": "room:kitchen",
                "provenance": "seeded_topology"
            })),
            ("portal:living", "Portal", "Living Room Doorway", json!({
                "door_x": 5.0, "door_y": -1.0, "width": 1.2,
                "approach_x": 5.0, "approach_y": 0.0, "approach_yaw": -1.5708,
                "threshold_x": 5.0, "threshold_y": -1.4, "threshold_yaw": -1.5708,
                "target_x": 4.0, "target_y": -2.5,
                "connects_from": "room:hallway", "connects_to": "room:living",
                "provenance": "seeded_topology"
            })),
        ];

        for (id, node_type, label, props) in portals {
            self.upsert_node(id, node_type, label, Some(props.clone()))?;
            let from_room = props["connects_from"].as_str().unwrap();
            let to_room = props["connects_to"].as_str().unwrap();
            self.add_edge(from_room, id, "CONNECTS", 1.0, Some(json!({"provenance": "seeded_topology"})))?;
            self.add_edge(id, to_room, "LEADS_TO", 1.0, Some(json!({"provenance": "seeded_topology"})))?;
            self.add_edge(to_room, id, "CONNECTS", 1.0, Some(json!({"provenance": "seeded_topology"})))?;
            self.add_edge(id, from_room, "LEADS_TO", 1.0, Some(json!({"provenance": "seeded_topology"})))?;
        }

        // Add Robot self-node
        self.upsert_node("robot:vira", "Robot", "GRaCEmo ViRa", Some(json!({"height": 1.45, "type": "Humanoid Service Robot"})))?;

        Ok(())
    }

    pub fn upsert_node(
        &self,
        id: &str,
        node_type: &str,
        label: &str,
        properties: Option<Value>,
    ) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        let now = Utc::now().timestamp();
        let props_str = properties.unwrap_or_else(|| json!({})).to_string();

        conn.execute(
            "INSERT INTO nodes (id, type, label, properties, created_at, updated_at)
             VALUES (?1, ?2, ?3, ?4, ?5, ?5)
             ON CONFLICT(id) DO UPDATE SET
                type = excluded.type,
                label = excluded.label,
                properties = excluded.properties,
                updated_at = excluded.updated_at",
            params![id, node_type, label, props_str, now],
        )?;

        Ok(())
    }

    pub fn add_edge(
        &self,
        source_id: &str,
        target_id: &str,
        relation: &str,
        confidence: f32,
        properties: Option<Value>,
    ) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        let now = Utc::now().timestamp();
        let props_str = properties.unwrap_or_else(|| json!({})).to_string();

        conn.execute(
            "INSERT INTO edges (source_id, target_id, relation, weight, confidence, properties, created_at, updated_at)
             VALUES (?1, ?2, ?3, 1, ?4, ?5, ?6, ?6)
             ON CONFLICT(source_id, target_id, relation) DO UPDATE SET
                weight = weight + 1,
                confidence = excluded.confidence,
                properties = excluded.properties,
                updated_at = excluded.updated_at",
            params![source_id, target_id, relation, confidence, props_str, now],
        )?;

        Ok(())
    }

    pub fn query_full_graph(&self) -> Result<Value> {
        let conn = self.conn.lock().unwrap();

        let mut nodes_stmt = conn.prepare("SELECT id, type, label, properties, updated_at FROM nodes")?;
        let nodes: Vec<Value> = nodes_stmt
            .query_map([], |row| {
                let props_str: String = row.get(3)?;
                let props: Value = serde_json::from_str(&props_str).unwrap_or(json!({}));
                Ok(json!({
                    "id": row.get::<_, String>(0)?,
                    "type": row.get::<_, String>(1)?,
                    "label": row.get::<_, String>(2)?,
                    "properties": props,
                    "updated_at": row.get::<_, i64>(4)?,
                }))
            })?
            .filter_map(Result::ok)
            .collect();

        let mut edges_stmt = conn.prepare(
            "SELECT source_id, target_id, relation, weight, confidence, properties, updated_at FROM edges"
        )?;
        let edges: Vec<Value> = edges_stmt
            .query_map([], |row| {
                let props_str: String = row.get(5)?;
                let props: Value = serde_json::from_str(&props_str).unwrap_or(json!({}));
                Ok(json!({
                    "source": row.get::<_, String>(0)?,
                    "target": row.get::<_, String>(1)?,
                    "relation": row.get::<_, String>(2)?,
                    "weight": row.get::<_, i32>(3)?,
                    "confidence": row.get::<_, f64>(4)?,
                    "properties": props,
                    "updated_at": row.get::<_, i64>(6)?,
                }))
            })?
            .filter_map(Result::ok)
            .collect();

        Ok(json!({
            "nodes": nodes,
            "edges": edges,
            "node_count": nodes.len(),
            "edge_count": edges.len(),
        }))
    }

    /// Fast lookup: Where is an object located in the apartment?
    pub fn query_object_location(&self, object_name: &str) -> Result<Option<Value>> {
        let conn = self.conn.lock().unwrap();
        let target_node_id = format!("object:{}", object_name.to_lowercase().trim());

        let mut stmt = conn.prepare(
            "SELECT n.label, e.relation, r.label, e.confidence, e.properties, e.updated_at
             FROM edges e
             JOIN nodes n ON e.source_id = n.id
             JOIN nodes r ON e.target_id = r.id
             WHERE e.source_id = ?1
             ORDER BY e.updated_at DESC LIMIT 1"
        )?;

        let result = stmt.query_row(params![target_node_id], |row| {
            let props_str: String = row.get(4)?;
            let props: Value = serde_json::from_str(&props_str).unwrap_or(json!({}));
            Ok(json!({
                "object": row.get::<_, String>(0)?,
                "relation": row.get::<_, String>(1)?,
                "location": row.get::<_, String>(2)?,
                "confidence": row.get::<_, f64>(3)?,
                "properties": props,
                "timestamp": row.get::<_, i64>(5)?,
            }))
        });

        match result {
            Ok(val) => Ok(Some(val)),
            Err(rusqlite::Error::QueryReturnedNoRows) => Ok(None),
            Err(e) => Err(anyhow::anyhow!(e)),
        }
    }

    /// Query all tracked world objects, evaluating visibility lifecycle against current timestamp.
    pub fn query_world_objects(
        &self,
        class_filter: Option<&str>,
        visible_only: bool,
    ) -> Result<Vec<WorldObject>> {
        self.expire_visual_observations()?;
        let conn = self.conn.lock().unwrap();
        let current_time = Utc::now().timestamp() as f64;
        let mut stmt = conn.prepare("SELECT properties FROM nodes WHERE type = 'WorldObject'")?;
        let raw_objs: Vec<WorldObject> = stmt
            .query_map([], |row| {
                let props_str: String = row.get(0)?;
                let obj: Result<WorldObject, _> = serde_json::from_str(&props_str);
                match obj {
                    Ok(o) => Ok(Some(o)),
                    Err(_) => Ok(None),
                }
            })?
            .filter_map(Result::ok)
            .flatten()
            .collect();

        let mut filtered = Vec::new();
        for mut obj in raw_objs {
            obj.currently_visible = obj.is_visible_at(current_time, 10.0);
            if let Some(cf) = class_filter {
                if !obj.class_name.eq_ignore_ascii_case(cf) {
                    continue;
                }
            }
            if visible_only && !obj.currently_visible {
                continue;
            }
            filtered.push(obj);
        }
        filtered.sort_by(|a, b| a.object_id.cmp(&b.object_id));
        Ok(filtered)
    }

    /// Query a specific world object by its deterministic ID (e.g. "chair_001")
    pub fn query_world_object_by_id(&self, object_id: &str) -> Result<Option<WorldObject>> {
        let all = self.query_world_objects(None, false)?;
        let clean_id = object_id.strip_prefix("object:").unwrap_or(object_id);
        Ok(all.into_iter().find(|o| o.object_id == clean_id))
    }

    /// Query the nearest world object to given (x, y, z) coordinates matching class
    pub fn query_nearest_world_object(
        &self,
        class_name: &str,
        x: f64,
        y: f64,
        z: f64,
        visible_only: bool,
    ) -> Result<Option<WorldObject>> {
        let objs = self.query_world_objects(Some(class_name), visible_only)?;
        let nearest = objs
            .into_iter()
            .min_by(|a, b| {
                let da = a.distance_to(x, y, z);
                let db = b.distance_to(x, y, z);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            });
        Ok(nearest)
    }

    /// Remove all outgoing edges from source_id with specified relation
    pub fn remove_edges_by_relation(&self, source_id: &str, relation: &str) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        conn.execute(
            "DELETE FROM edges WHERE source_id = ?1 AND relation = ?2",
            params![source_id, relation],
        )?;
        Ok(())
    }

    pub fn canonical_room_id(room: &str) -> &'static str {
        let lower = room.to_lowercase();
        if lower.contains("bed") {
            "room:bedroom"
        } else if lower.contains("kitch") {
            "room:kitchen"
        } else if lower.contains("study") {
            "room:study"
        } else if lower.contains("living") {
            "room:living"
        } else {
            "room:hallway"
        }
    }

    /// Resolve canonical room ID based on physical 2D coordinates (x, y)
    pub fn resolve_room_for_coordinates(x: f64, y: f64, fallback_room: &str) -> &'static str {
        // Apartment boundary geometry derived from verified apartment_floor.world
        // Bedroom: [-9.0, 0.0] x [1.0, 7.0]
        // Study:   [0.0, 9.0] x [1.0, 7.0]
        // Kitchen: [-9.0, 0.0] x [-7.0, -1.0]
        // Living:  [0.0, 9.0] x [-7.0, -1.0]
        // Hallway: [-9.0, 9.0] x [-1.0, 1.0]
        if y > 1.0 && y <= 7.5 {
            if x < 0.0 && x >= -9.5 {
                "room:bedroom"
            } else if x >= 0.0 && x <= 9.5 {
                "room:study"
            } else {
                Self::canonical_room_id(fallback_room)
            }
        } else if y < -1.0 && y >= -7.5 {
            if x < 0.0 && x >= -9.5 {
                "room:kitchen"
            } else if x >= 0.0 && x <= 9.5 {
                "room:living"
            } else {
                Self::canonical_room_id(fallback_room)
            }
        } else if y >= -1.0 && y <= 1.0 && x >= -9.5 && x <= 9.5 {
            "room:hallway"
        } else {
            Self::canonical_room_id(fallback_room)
        }
    }

    /// Query all persistent WorldObjects located in designated room
    pub fn query_objects_in_room(&self, room_id: &str) -> Result<Vec<WorldObject>> {
        let canonical_id = Self::canonical_room_id(room_id);
        let conn = self.conn.lock().unwrap();
        let mut stmt = conn.prepare(
            "SELECT n.properties FROM nodes n
             INNER JOIN edges e ON n.id = e.source_id
             WHERE e.target_id = ?1 AND e.relation = 'LOCATED_IN' AND n.type = 'WorldObject'
             ORDER BY n.id ASC",
        )?;
        let rows = stmt.query_map([canonical_id], |row| {
            let props_str: Option<String> = row.get(0)?;
            Ok(props_str)
        })?;

        let mut objects = Vec::new();
        for r in rows {
            if let Ok(Some(s)) = r {
                if let Ok(obj) = serde_json::from_str::<WorldObject>(&s) {
                    objects.push(obj);
                }
            }
        }
        objects.sort_by(|a, b| a.object_id.cmp(&b.object_id));
        objects.dedup_by(|a, b| a.object_id == b.object_id);
        Ok(objects)
    }

    /// Query the room label or room ID where a specific object is located
    pub fn query_room_for_object(&self, object_id: &str) -> Result<Option<String>> {
        let clean_id = object_id.strip_prefix("object:").unwrap_or(object_id);
        let node_id = format!("object:{}", clean_id);
        let conn = self.conn.lock().unwrap();
        let mut stmt = conn.prepare(
            "SELECT n.label, e.target_id FROM edges e
             LEFT JOIN nodes n ON e.target_id = n.id
             WHERE e.source_id = ?1 AND e.relation = 'LOCATED_IN'
             LIMIT 1",
        )?;
        let mut rows = stmt.query([node_id])?;
        if let Some(row) = rows.next()? {
            let label: Option<String> = row.get(0)?;
            let target_id: String = row.get(1)?;
            Ok(label.or(Some(target_id)))
        } else {
            Ok(None)
        }
    }

    /// Mark a room as authoritatively explored in the SQLite graph
    pub fn mark_room_explored(&self, room_id: &str) -> Result<()> {
        let canonical_id = Self::canonical_room_id(room_id);
        let conn = self.conn.lock().unwrap();
        let now = Utc::now().timestamp();

        let props_str: Option<String> = match conn.query_row(
            "SELECT properties FROM nodes WHERE id = ?1 AND type = 'Room'",
            params![canonical_id],
            |row| row.get(0),
        ) {
            Ok(s) => Some(s),
            Err(rusqlite::Error::QueryReturnedNoRows) => None,
            Err(e) => return Err(anyhow::anyhow!(e)),
        };

        if let Some(s) = props_str {
            let mut props: Value = serde_json::from_str(&s).unwrap_or(json!({}));
            if let Some(obj) = props.as_object_mut() {
                let first_exp = obj.get("first_explored")
                    .and_then(|v| v.as_i64())
                    .unwrap_or(now);
                let scan_count = obj.get("scan_count")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0) + 1;
                obj.insert("explored".to_string(), json!(true));
                obj.insert("first_explored".to_string(), json!(first_exp));
                obj.insert("last_explored".to_string(), json!(now));
                obj.insert("scan_count".to_string(), json!(scan_count));
            }
            conn.execute(
                "UPDATE nodes SET properties = ?1, updated_at = ?2 WHERE id = ?3",
                params![props.to_string(), now, canonical_id],
            )?;
            info!("🗺️ [GRAPH] Room '{}' authoritatively marked as EXPLORED (scan_count: {})", canonical_id, props["scan_count"]);
        }
        Ok(())
    }

    /// Query single room details by ID or canonical name
    pub fn get_room(&self, room_id: &str) -> Result<Option<Value>> {
        let canonical_id = Self::canonical_room_id(room_id);
        let conn = self.conn.lock().unwrap();

        let row_data: Option<(String, String, Option<String>)> = match conn.query_row(
            "SELECT id, label, properties FROM nodes WHERE id = ?1 AND type = 'Room'",
            params![canonical_id],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
        ) {
            Ok(d) => Some(d),
            Err(rusqlite::Error::QueryReturnedNoRows) => None,
            Err(e) => return Err(anyhow::anyhow!(e)),
        };

        if let Some((id, label, props_str)) = row_data {
            let props: Value = props_str
                .and_then(|s| serde_json::from_str(&s).ok())
                .unwrap_or_else(|| json!({}));

            let explored = props.get("explored").and_then(|v| v.as_bool()).unwrap_or(false);
            let scan_count = props.get("scan_count").and_then(|v| v.as_u64()).unwrap_or(0);
            let first_explored = props.get("first_explored").and_then(|v| v.as_i64());
            let last_explored = props.get("last_explored").and_then(|v| v.as_i64());

            let object_count: usize = conn.query_row(
                "SELECT COUNT(DISTINCT e.source_id) FROM edges e
                 INNER JOIN nodes n ON e.source_id = n.id
                 WHERE e.target_id = ?1 AND e.relation = 'LOCATED_IN' AND n.type = 'WorldObject'",
                params![canonical_id],
                |row| row.get(0),
            ).unwrap_or(0);

            let mut connected_stmt = conn.prepare(
                "SELECT DISTINCT e2.target_id FROM edges e1
                 JOIN edges e2 ON e1.target_id = e2.source_id
                 WHERE e1.source_id = ?1 AND e1.relation = 'CONNECTS' AND e2.relation = 'LEADS_TO' AND e2.target_id != e1.source_id"
            )?;
            let connected_rooms: Vec<String> = connected_stmt
                .query_map(params![canonical_id], |row| row.get(0))?
                .filter_map(Result::ok)
                .collect();

            Ok(Some(json!({
                "room_id": id,
                "label": label,
                "explored": explored,
                "scan_count": scan_count,
                "first_explored": first_explored,
                "last_explored": last_explored,
                "object_count": object_count,
                "connected_rooms": connected_rooms,
                "properties": props
            })))
        } else {
            Ok(None)
        }
    }

    /// Query all seeded apartment room nodes with exploration status, object counts, and connections
    pub fn query_all_rooms(&self) -> Result<Vec<Value>> {
        let conn = self.conn.lock().unwrap();
        let mut stmt = conn.prepare(
            "SELECT id, label, properties FROM nodes WHERE type = 'Room' ORDER BY id ASC",
        )?;
        let rows = stmt.query_map([], |row| {
            let id: String = row.get(0)?;
            let label: String = row.get(1)?;
            let props_str: Option<String> = row.get(2)?;
            let props: Value = props_str
                .and_then(|s| serde_json::from_str(&s).ok())
                .unwrap_or_else(|| json!({}));

            let explored = props.get("explored").and_then(|v| v.as_bool()).unwrap_or(false);
            let scan_count = props.get("scan_count").and_then(|v| v.as_u64()).unwrap_or(0);
            let first_explored = props.get("first_explored").and_then(|v| v.as_i64());
            let last_explored = props.get("last_explored").and_then(|v| v.as_i64());

            Ok((id, label, props, explored, scan_count, first_explored, last_explored))
        })?;

        let mut rooms = Vec::new();
        for r in rows {
            if let Ok((id, label, props, explored, scan_count, first_explored, last_explored)) = r {
                let object_count: usize = conn.query_row(
                    "SELECT COUNT(DISTINCT e.source_id) FROM edges e
                     INNER JOIN nodes n ON e.source_id = n.id
                     WHERE e.target_id = ?1 AND e.relation = 'LOCATED_IN' AND n.type = 'WorldObject'",
                    params![id],
                    |row| row.get(0),
                ).unwrap_or(0);

                let mut connected_stmt = conn.prepare(
                    "SELECT DISTINCT e2.target_id FROM edges e1
                     JOIN edges e2 ON e1.target_id = e2.source_id
                     WHERE e1.source_id = ?1 AND e1.relation = 'CONNECTS' AND e2.relation = 'LEADS_TO' AND e2.target_id != e1.source_id"
                )?;
                let connected_rooms: Vec<String> = connected_stmt
                    .query_map(params![id], |row| row.get(0))?
                    .filter_map(Result::ok)
                    .collect();

                rooms.push(json!({
                    "room_id": id,
                    "label": label,
                    "explored": explored,
                    "scan_count": scan_count,
                    "first_explored": first_explored,
                    "last_explored": last_explored,
                    "object_count": object_count,
                    "connected_rooms": connected_rooms,
                    "properties": props
                }));
            }
        }
        Ok(rooms)
    }

    /// Aggregates overall apartment exploration status
    pub fn get_exploration_status(&self) -> Result<Value> {
        let rooms = self.query_all_rooms()?;
        let total = rooms.len();
        let explored_count = rooms
            .iter()
            .filter(|r| r.get("explored").and_then(|v| v.as_bool()).unwrap_or(false))
            .count();
        let unexplored_count = total.saturating_sub(explored_count);
        let total_objects: usize = rooms
            .iter()
            .filter_map(|r| r.get("object_count").and_then(|v| v.as_u64()))
            .sum::<u64>() as usize;
        let all_explored = total > 0 && explored_count == total;

        Ok(json!({
            "total_rooms": total,
            "explored_rooms": explored_count,
            "unexplored_rooms": unexplored_count,
            "all_explored": all_explored,
            "total_objects_grounded": total_objects,
            "rooms": rooms,
        }))
    }

    /// Process live events and build the knowledge graph deterministically
    pub fn process_event(&self, event: &Event, current_room: &str) -> Result<()> {
        match &event.event_type {
            // Real YOLOv11 Visual Detections with 3D Spatial Position
            EventType::ObjectDetected { class_name, confidence, x, y, z, bbox, depth_m, frame_id,
                detection_id, track_id, camera_id, image_stamp_ns } => {
                let detected_z = z.unwrap_or(0.0) as f64;
                let detected_x = *x as f64;
                let detected_y = *y as f64;
                let source_str = event.source.to_string();
                let frame_str = frame_id.as_deref().unwrap_or("world").to_string();
                let timestamp = event.timestamp as f64;

                // 1. Query existing world objects for same class (case-insensitive, unfiltered by visibility)
                let existing_objs = self.query_world_objects(Some(class_name), false)?;

                // 2. Calculate 3D Euclidean distance to all candidates
                let mut candidates: Vec<(f64, WorldObject)> = existing_objs
                    .into_iter()
                    .map(|o| {
                        let d = o.distance_to(detected_x, detected_y, detected_z);
                        (d, o)
                    })
                    .filter(|(d, _)| *d <= 0.5)
                    .collect();

                candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

                // 3. Ambiguity check: if 2+ candidates within 0.5m and |d0 - d1| <= 0.01m, do not merge!
                let is_ambiguous = if candidates.len() >= 2 {
                    (candidates[1].0 - candidates[0].0).abs() <= 0.01
                } else {
                    false
                };

                let room_id = Self::resolve_room_for_coordinates(detected_x, detected_y, current_room);

                let updated_object = if !is_ambiguous && !candidates.is_empty() {
                    // Update existing object
                    let mut obj = candidates[0].1.clone();
                    obj.update(
                        *confidence,
                        detected_x,
                        detected_y,
                        detected_z,
                        timestamp,
                        source_str.clone(),
                        frame_str.clone(),
                        bbox.clone(),
                        *depth_m,
                    );
                    let node_id = format!("object:{}", obj.object_id);
                    self.upsert_node(
                        &node_id,
                        "WorldObject",
                        &obj.class_name,
                        Some(serde_json::to_value(&obj)?),
                    )?;
                    let _ = self.remove_edges_by_relation(&node_id, "LOCATED_IN");
                    self.add_edge(
                        &node_id,
                        room_id,
                        "LOCATED_IN",
                        *confidence,
                        Some(json!({"observed_at": timestamp, "x": obj.world_x, "y": obj.world_y, "z": obj.world_z})),
                    )?;
                    info!("🔄 Updated WorldObject: {} at ({:.2}, {:.2}, {:.2}) [obs: {}, room: {}]", obj.object_id, obj.world_x, obj.world_y, obj.world_z, obj.observation_count, room_id);
                    obj
                } else {
                    // Create new object with deterministic ID: {class_name}_{next_index:03}
                    let all_class_objs = self.query_world_objects(Some(class_name), false)?;
                    let max_idx = all_class_objs
                        .iter()
                        .filter_map(|o| {
                            let parts: Vec<&str> = o.object_id.split('_').collect();
                            parts.last().and_then(|num_str| num_str.parse::<usize>().ok())
                        })
                        .max()
                        .unwrap_or(0);
                    let next_idx = max_idx + 1;
                    let slug = class_name.to_lowercase().replace(' ', "_");
                    let obj_id = format!("{}_{:03}", slug, next_idx);

                    let new_obj = WorldObject::new(
                        obj_id.clone(),
                        class_name.clone(),
                        *confidence,
                        detected_x,
                        detected_y,
                        detected_z,
                        timestamp,
                        source_str,
                        frame_str,
                        bbox.clone(),
                        *depth_m,
                    );

                    let node_id = format!("object:{}", obj_id);
                    self.upsert_node(
                        &node_id,
                        "WorldObject",
                        class_name,
                        Some(serde_json::to_value(&new_obj)?),
                    )?;
                    let _ = self.remove_edges_by_relation(&node_id, "LOCATED_IN");
                    self.add_edge(
                        &node_id,
                        room_id,
                        "LOCATED_IN",
                        *confidence,
                        Some(json!({"observed_at": timestamp, "x": detected_x, "y": detected_y, "z": detected_z})),
                    )?;
                    info!("✨ Created new WorldObject: {} at ({:.2}, {:.2}, {:.2}) [conf: {:.2}, room: {}]", obj_id, detected_x, detected_y, detected_z, confidence, room_id);
                    new_obj
                };

                // Also maintain legacy node for backwards compatibility with query_object_location
                let legacy_id = format!("object:{}", class_name.to_lowercase().trim());
                self.upsert_node(
                    &legacy_id,
                    "Object",
                    class_name,
                    Some(json!({"x": updated_object.world_x, "y": updated_object.world_y, "z": updated_object.world_z, "source": "YOLOv11"})),
                )?;
                let _ = self.remove_edges_by_relation(&legacy_id, "LOCATED_IN");
                self.add_edge(
                    &legacy_id,
                    room_id,
                    "LOCATED_IN",
                    *confidence,
                    Some(json!({"observed_at": timestamp, "x": updated_object.world_x, "y": updated_object.world_y, "z": updated_object.world_z})),
                )?;
                self.record_visual_observation(event, &updated_object.object_id, class_name,
                    *confidence, detected_x, detected_y, detected_z, frame_id.as_deref(),
                    camera_id.as_deref(), detection_id.as_deref(), track_id.as_deref(),
                    *image_stamp_ns)?;
            }

            // Real Odometry / Navigation Arrived
            EventType::NavigationArrived { destination, success } => {
                if *success {
                    let room_id = Self::canonical_room_id(destination);
                    self.add_edge("robot:vira", room_id, "LOCATED_IN", 1.0, Some(json!({"arrived_at": event.timestamp})))?;
                    info!("🕸️ Graph updated: (robot:vira) -[LOCATED_IN]-> ({})", room_id);
                }
            }

            // Scan completed in a room -> authoritative trigger for marking room explored
            EventType::ScanCompleted { room, success, objects_found, duration_sec } => {
                if *success {
                    info!("🔍 ScanCompleted in room {}: {} objects found in {:.2}s. Marking room explored.", room, objects_found.len(), duration_sec);
                    if let Err(e) = self.mark_room_explored(room) {
                        tracing::error!("Failed to mark room {} explored on ScanCompleted: {:?}", room, e);
                    }
                }
            }

            // Explicit RoomExplored event
            EventType::RoomExplored { room, timestamp: _, objects_count: _ } => {
                if let Err(e) = self.mark_room_explored(room) {
                    tracing::error!("Failed to mark room {} explored on RoomExplored event: {:?}", room, e);
                }
            }

            // Human Presence Visible
            EventType::PersonVisible { identity, confidence, distance } => {
                let person_id = format!("person:{}", identity.to_lowercase().trim());
                let room_id = format!("room:{}", current_room.to_lowercase().replace(' ', "_").replace('&', "and").trim());

                self.upsert_node(&person_id, "Person", identity, Some(json!({"distance": distance})))?;
                self.add_edge(&person_id, &room_id, "PRESENT_IN", *confidence, Some(json!({"time": event.timestamp})))?;
            }

            // Custom Facts Remembered via K CLI
            EventType::Custom { name, payload } => {
                if name == "FactRemembered" {
                    if let Some(fact) = payload.get("fact").and_then(|f| f.as_str()) {
                        let fact_id = format!("fact:{}", &event.id.to_string()[..8]);
                        self.upsert_node(&fact_id, "Fact", fact, Some(payload.clone()))?;
                        self.add_edge(&fact_id, "robot:vira", "REMEMBERS", 1.0, None)?;
                    }
                }
            }

            _ => {}
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_seeded_doorway_portals() {
        let db = GraphDb::open(":memory:").expect("Failed to create in-memory db");
        let full = db.query_full_graph().expect("Failed to query graph");
        let nodes = full["nodes"].as_array().expect("nodes must be array");
        let edges = full["edges"].as_array().expect("edges must be array");

        let bedroom_portal = nodes.iter().find(|n| n["id"] == "portal:bedroom").expect("Bedroom portal missing");
        let props = &bedroom_portal["properties"];
        assert_eq!(props["door_x"], -5.0);
        assert_eq!(props["door_y"], 1.0);
        assert_eq!(props["approach_x"], -5.0);
        assert_eq!(props["approach_y"], 0.0);
        assert_eq!(props["threshold_x"], -5.0);
        assert_eq!(props["threshold_y"], 1.4);

        let kitchen_portal = nodes.iter().find(|n| n["id"] == "portal:kitchen").expect("Kitchen portal missing");
        assert_eq!(kitchen_portal["properties"]["door_x"], -5.0);
        assert_eq!(kitchen_portal["properties"]["door_y"], -1.0);

        let study_portal = nodes.iter().find(|n| n["id"] == "portal:study").expect("Study portal missing");
        assert_eq!(study_portal["properties"]["door_x"], 5.0);
        assert_eq!(study_portal["properties"]["door_y"], 1.0);

        let living_portal = nodes.iter().find(|n| n["id"] == "portal:living").expect("Living portal missing");
        assert_eq!(living_portal["properties"]["door_x"], 5.0);
        assert_eq!(living_portal["properties"]["door_y"], -1.0);

        // Verify topological edges
        let connects = edges.iter().find(|e| e["source"] == "room:hallway" && e["target"] == "portal:bedroom" && e["relation"] == "CONNECTS");
        assert!(connects.is_some(), "room:hallway must CONNECT portal:bedroom");

        let leads_to = edges.iter().find(|e| e["source"] == "portal:bedroom" && e["target"] == "room:bedroom" && e["relation"] == "LEADS_TO");
        assert!(leads_to.is_some(), "portal:bedroom must LEAD_TO room:bedroom");
    }

    #[test]
    fn test_object_detected_lifecycle_and_association() {
        use gracemo_types::EventSource;
        let db = GraphDb::open(":memory:").expect("Failed to create in-memory db");

        // 1. Initial detection creates chair_001
        let ev1 = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ObjectDetected {
                class_name: "chair".to_string(),
                confidence: 0.67,
                x: 2.18,
                y: 0.19,
                z: Some(0.52),
                bbox: Some(vec![100.0, 150.0, 200.0, 300.0]),
                depth_m: Some(2.18),
                frame_id: Some("world".to_string()),
                detection_id: None, track_id: None, camera_id: None, image_stamp_ns: None,
            },
        );
        db.process_event(&ev1, "Kitchen & Dining").unwrap();

        let objs = db.query_world_objects(None, false).unwrap();
        assert_eq!(objs.len(), 1);
        assert_eq!(objs[0].object_id, "chair_001");
        assert_eq!(objs[0].class_name, "chair");
        assert_eq!(objs[0].observation_count, 1);
        assert!((objs[0].world_x - 2.18).abs() < 1e-4);
        assert!((objs[0].world_y - 0.19).abs() < 1e-4);
        assert!((objs[0].world_z - 0.52).abs() < 1e-4);

        // 2. Nearby detection (0.03m away <= 0.5m) associates with chair_001
        let ev2 = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ObjectDetected {
                class_name: "chair".to_string(),
                confidence: 0.85,
                x: 2.20,
                y: 0.21,
                z: Some(0.51),
                bbox: Some(vec![102.0, 152.0, 202.0, 302.0]),
                depth_m: Some(2.19),
                frame_id: Some("world".to_string()),
                detection_id: None, track_id: None, camera_id: None, image_stamp_ns: None,
            },
        );
        db.process_event(&ev2, "Kitchen & Dining").unwrap();

        let objs_after = db.query_world_objects(None, false).unwrap();
        assert_eq!(objs_after.len(), 1, "Should associate, not create second chair");
        assert_eq!(objs_after[0].object_id, "chair_001");
        assert_eq!(objs_after[0].observation_count, 2);
        assert_eq!(objs_after[0].confidence, 0.85);
        assert!((objs_after[0].world_x - 2.194).abs() < 1e-4);

        // 3. Distant chair (>0.5m away) creates chair_002
        let ev3 = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ObjectDetected {
                class_name: "chair".to_string(),
                confidence: 0.75,
                x: 4.50,
                y: 1.00,
                z: Some(0.50),
                bbox: None,
                depth_m: None,
                frame_id: None,
                detection_id: None, track_id: None, camera_id: None, image_stamp_ns: None,
            },
        );
        db.process_event(&ev3, "Living Room").unwrap();

        let all_chairs = db.query_world_objects(Some("chair"), false).unwrap();
        assert_eq!(all_chairs.len(), 2);
        assert_eq!(all_chairs[0].object_id, "chair_001");
        assert_eq!(all_chairs[1].object_id, "chair_002");

        // 4. Query nearest chair
        let nearest = db.query_nearest_world_object("chair", 2.0, 0.0, 0.5, false).unwrap();
        assert!(nearest.is_some());
        assert_eq!(nearest.unwrap().object_id, "chair_001");

        let nearest_to_distant = db.query_nearest_world_object("chair", 4.0, 1.0, 0.5, false).unwrap();
        assert_eq!(nearest_to_distant.unwrap().object_id, "chair_002");

        // 5. Query by id
        let by_id = db.query_world_object_by_id("chair_002").unwrap();
        assert!(by_id.is_some());
        assert_eq!(by_id.unwrap().class_name, "chair");
    }

    #[test]
    fn test_coordinate_room_resolution() {
        assert_eq!(GraphDb::resolve_room_for_coordinates(-5.0, 3.0, "Central Hallway"), "room:bedroom");
        assert_eq!(GraphDb::resolve_room_for_coordinates(4.5, 3.0, "Central Hallway"), "room:study");
        assert_eq!(GraphDb::resolve_room_for_coordinates(-4.5, -2.5, "Central Hallway"), "room:kitchen");
        assert_eq!(GraphDb::resolve_room_for_coordinates(4.0, -2.5, "Central Hallway"), "room:living");
        assert_eq!(GraphDb::resolve_room_for_coordinates(0.0, 0.0, "Master Bedroom"), "room:hallway");
    }

    #[test]
    fn test_room_objects_query_and_located_in_persistence() {
        use gracemo_types::EventSource;
        let db = GraphDb::open(":memory:").expect("Failed to create in-memory db");

        // Verify seeded rooms
        let rooms = db.query_all_rooms().expect("Must query rooms");
        assert_eq!(rooms.len(), 5, "Must have 5 seeded rooms");

        // 1. Detect bed in Master Bedroom
        let ev_bed = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ObjectDetected {
                class_name: "bed".to_string(),
                confidence: 0.88,
                x: -5.0,
                y: 3.0,
                z: Some(0.40),
                bbox: None,
                depth_m: Some(2.5),
                frame_id: Some("world".to_string()),
                detection_id: None, track_id: None, camera_id: None, image_stamp_ns: None,
            },
        );
        db.process_event(&ev_bed, "Central Hallway").unwrap();

        // 2. Detect desk in Home Study
        let ev_desk = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ObjectDetected {
                class_name: "desk".to_string(),
                confidence: 0.82,
                x: 4.5,
                y: 3.0,
                z: Some(0.70),
                bbox: None,
                depth_m: Some(2.0),
                frame_id: Some("world".to_string()),
                detection_id: None, track_id: None, camera_id: None, image_stamp_ns: None,
            },
        );
        db.process_event(&ev_desk, "Home Study").unwrap();

        // Query objects in Master Bedroom
        let bedroom_objs = db.query_objects_in_room("room:bedroom").unwrap();
        assert_eq!(bedroom_objs.len(), 1);
        assert_eq!(bedroom_objs[0].object_id, "bed_001");
        assert_eq!(bedroom_objs[0].class_name, "bed");

        // Query objects in Home Study
        let study_objs = db.query_objects_in_room("Home Study").unwrap();
        assert_eq!(study_objs.len(), 1);
        assert_eq!(study_objs[0].object_id, "desk_001");

        // Query room for object
        let bed_room = db.query_room_for_object("bed_001").unwrap();
        assert_eq!(bed_room, Some("Master Bedroom".to_string()));

        let desk_room = db.query_room_for_object("desk_001").unwrap();
        assert_eq!(desk_room, Some("Home Study".to_string()));

        // Query room with zero objects
        let living_objs = db.query_objects_in_room("Living Room").unwrap();
        assert_eq!(living_objs.len(), 0);
    }

    #[test]
    fn test_room_exploration_state_and_queries() {
        use gracemo_types::EventSource;
        let db = GraphDb::open(":memory:").expect("Failed to create in-memory db");

        // Initially, all rooms are unexplored
        let initial_status = db.get_exploration_status().expect("Must get exploration status");
        assert_eq!(initial_status["total_rooms"], 5);
        assert_eq!(initial_status["explored_rooms"], 0);
        assert_eq!(initial_status["unexplored_rooms"], 5);
        assert_eq!(initial_status["all_explored"], false);

        // Get single room before exploration
        let bedroom_initial = db.get_room("room:bedroom").expect("Must get bedroom").unwrap();
        assert_eq!(bedroom_initial["explored"], false);
        assert_eq!(bedroom_initial["scan_count"], 0);
        assert_eq!(bedroom_initial["connected_rooms"].as_array().unwrap().len(), 1);

        // Simulate ScanCompleted in Master Bedroom
        let ev_scan = Event::new(
            EventSource::Vision,
            "perception_service",
            EventType::ScanCompleted {
                room: "room:bedroom".to_string(),
                objects_found: vec!["bed_001".to_string()],
                success: true,
                duration_sec: 4.5,
            },
        );
        db.process_event(&ev_scan, "Master Bedroom").unwrap();

        // Check bedroom state now
        let bedroom_after = db.get_room("room:bedroom").expect("Must get bedroom").unwrap();
        assert_eq!(bedroom_after["explored"], true);
        assert_eq!(bedroom_after["scan_count"], 1);
        assert!(bedroom_after["first_explored"].is_number());
        assert!(bedroom_after["last_explored"].is_number());

        // Check overall status
        let status_mid = db.get_exploration_status().unwrap();
        assert_eq!(status_mid["explored_rooms"], 1);
        assert_eq!(status_mid["unexplored_rooms"], 4);
        assert_eq!(status_mid["all_explored"], false);

        // Mark remaining rooms explored
        db.mark_room_explored("room:hallway").unwrap();
        db.mark_room_explored("room:study").unwrap();
        db.mark_room_explored("room:kitchen").unwrap();
        db.mark_room_explored("room:living").unwrap();

        let status_all = db.get_exploration_status().unwrap();
        assert_eq!(status_all["explored_rooms"], 5);
        assert_eq!(status_all["unexplored_rooms"], 0);
        assert_eq!(status_all["all_explored"], true);
    }

    #[test]
    fn test_visual_observation_supersede_delete_and_retention_views() {
        use gracemo_types::EventSource;
        let db = GraphDb::open(":memory:").expect("in-memory database");
        let mut first = Event::new(EventSource::Vision, "camera", EventType::ObjectDetected {
            class_name: "chair".into(), confidence: 0.8, x: 1.0, y: 2.0,
            z: Some(0.5), bbox: None, depth_m: Some(1.0), frame_id: Some("map".into()),
            detection_id: Some("det-1".into()), track_id: Some("track-7".into()),
            camera_id: Some("front-rgbd".into()), image_stamp_ns: Some(1234),
        });
        first.session_id = Some("mission-9".into());
        db.process_event(&first, "Home Study").unwrap();
        let mut second = Event::new(EventSource::Vision, "camera", EventType::ObjectDetected {
            class_name: "chair".into(), confidence: 0.9, x: 1.1, y: 2.0,
            z: Some(0.5), bbox: None, depth_m: Some(1.1), frame_id: Some("map".into()),
            detection_id: Some("det-2".into()), track_id: Some("track-7".into()),
            camera_id: Some("front-rgbd".into()), image_stamp_ns: Some(2234),
        });
        second.session_id = Some("mission-9".into());
        db.process_event(&second, "Home Study").unwrap();

        let active = db.query_visual_observations(Some("chair_001"), false, 10).unwrap();
        assert_eq!(active.len(), 1);
        assert_eq!(active[0]["observation_id"], second.id.to_string());
        assert_eq!(active[0]["mission_id"], "mission-9");
        assert_eq!(active[0]["track_id"], "track-7");
        assert_eq!(active[0]["source_camera"], "front-rgbd");
        assert_eq!(active[0]["image_stamp_ns"], 2234);
        let history = db.query_visual_observations(Some("chair_001"), true, 10).unwrap();
        assert_eq!(history.len(), 2);
        assert_eq!(history.iter().find(|row| row["observation_id"] == first.id.to_string()).unwrap()["status"], "superseded");

        assert!(db.delete_visual_observation(&second.id.to_string(), "operator", "privacy request").unwrap());
        assert!(db.query_visual_observations(Some("chair_001"), false, 10).unwrap().is_empty());
        assert!(db.visual_observation_hidden(&second.id.to_string()).unwrap());
        assert!(db.query_world_objects(Some("chair"), false).unwrap().is_empty());
        assert!(!db.delete_visual_observation(&second.id.to_string(), "operator", "repeat").unwrap());
        let after_delete = db.query_visual_observations(Some("chair_001"), true, 10).unwrap();
        assert_eq!(after_delete.len(), 1);
        assert_eq!(after_delete[0]["observation_id"], first.id.to_string());
    }
}
