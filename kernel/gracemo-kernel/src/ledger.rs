use anyhow::Result;
use gracemo_types::Event;
use rusqlite::Connection;
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use tracing::info;

/// Persistent "PAST" Record of Observed Events.
/// Implemented as an append-only SQLite WAL ledger.
#[derive(Clone)]
pub struct Ledger {
    conn: Arc<Mutex<Connection>>,
}

impl Ledger {
    pub fn open() -> Result<Self> {
        let db_dir = std::env::var_os("GRACEMO_STATE_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                let home_dir = std::env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
                PathBuf::from(home_dir).join(".gracemo")
            });
        std::fs::create_dir_all(&db_dir)?;
        let db_path = db_dir.join("ledger.db");

        let conn = Connection::open(&db_path)?;
        conn.execute_batch(
            "PRAGMA journal_mode = WAL;
             PRAGMA synchronous = NORMAL;
             CREATE TABLE IF NOT EXISTS events (
                 id TEXT PRIMARY KEY,
                 trace_id TEXT NOT NULL DEFAULT '',
                 timestamp INTEGER NOT NULL,
                 source TEXT NOT NULL,
                 observed_by TEXT NOT NULL,
                 event_type TEXT NOT NULL,
                 payload TEXT NOT NULL
             );
             CREATE INDEX IF NOT EXISTS idx_events_ts ON events(timestamp DESC);",
        )?;
        // Older local ledgers predate distributed trace IDs. Migrate in place
        // so existing event history remains readable.
        let has_trace_id = {
            let mut stmt = conn.prepare("PRAGMA table_info(events)")?;
            let columns = stmt.query_map([], |row| row.get::<_, String>(1))?;
            let found = columns.flatten().any(|column| column == "trace_id");
            found
        };
        if !has_trace_id {
            conn.execute(
                "ALTER TABLE events ADD COLUMN trace_id TEXT NOT NULL DEFAULT ''",
                [],
            )?;
        }
        for column in ["parent_event_id", "session_id"] {
            let exists: bool = conn.query_row(
                "SELECT EXISTS(SELECT 1 FROM pragma_table_info('events') WHERE name=?1)",
                [column], |row| row.get(0))?;
            if !exists {
                conn.execute(&format!("ALTER TABLE events ADD COLUMN {column} TEXT"), [])?;
            }
        }
        conn.execute("CREATE INDEX IF NOT EXISTS events_trace_ts ON events(trace_id,timestamp)", [])?;
        conn.execute_batch("CREATE TABLE IF NOT EXISTS dispatch_receipts (
            event_id TEXT PRIMARY KEY, mission_key TEXT UNIQUE, receipt TEXT NOT NULL
        )")?;
        conn.execute_batch("CREATE TABLE IF NOT EXISTS action_decisions (
            sequence INTEGER PRIMARY KEY, action_id TEXT NOT NULL, trace_id TEXT NOT NULL,
            adapter_id TEXT NOT NULL, mission_id TEXT, authorization_id TEXT,
            outcome TEXT NOT NULL, recorded_at INTEGER NOT NULL, detail TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS action_decisions_trace ON action_decisions(trace_id,sequence);")?;
        conn.execute_batch("CREATE TABLE IF NOT EXISTS operator_reviews(
            action_id TEXT PRIMARY KEY, trace_id TEXT NOT NULL, mission_id TEXT NOT NULL,
            operator_id TEXT NOT NULL, reviewed_at INTEGER NOT NULL, decision TEXT NOT NULL
        )")?;
        conn.execute_batch("CREATE TABLE IF NOT EXISTS emergency_holds(
            sequence INTEGER PRIMARY KEY, event_id TEXT UNIQUE NOT NULL,
            released_by TEXT, released_at INTEGER
        )")?;
        // Import earlier authenticated stops once. An old stop with uncertain
        // reset history requires an explicit operator acknowledgement.
        conn.execute_batch("INSERT OR IGNORE INTO emergency_holds(event_id)
            SELECT event_id FROM dispatch_receipts WHERE json_extract(receipt,'$.capability')='Stop'
            ORDER BY rowid")?;
        info!("💾 SQLite Event Ledger active at {:?}", db_path);

        Ok(Self {
            conn: Arc::new(Mutex::new(conn)),
        })
    }

    pub fn record_event(&self, event: &Event) -> Result<()> {
        self.record_with_receipt(event,None)
    }

    pub fn record_with_receipt(&self,event:&Event,receipt:Option<&Value>)->Result<()> {
        let payload_str = serde_json::to_string(&event.event_type).unwrap_or_default();
        let source_str = format!("{:?}", event.source);
        let event_type_name = format!("{:?}", event.event_type);
        let short_name = event_type_name
            .split('{')
            .next()
            .unwrap_or("Unknown")
            .trim();

        {
            let mut db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
            let tx=db.transaction()?;
            if let Some(r)=receipt {
                Self::insert_decision(&tx,event,&event.observed_by,"DISPATCHED",r)?;
                let mission_key=if r["capability"]=="StartMission" {r["mission_id"].as_str()} else {None};
                tx.execute("INSERT INTO dispatch_receipts VALUES (?1,?2,?3)",
                           (event.id.to_string(),mission_key,r.to_string()))?;
            }
            tx.execute(
                "INSERT INTO events (id, trace_id, timestamp, source, observed_by, event_type, payload, session_id, parent_event_id) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)",
                (
                    event.id.to_string(),
                    event.trace_id.to_string(),
                    event.timestamp,
                    source_str,
                    event.observed_by.clone(),
                    short_name,
                    payload_str,
                    event.session_id.as_deref(),
                    event.parent_event_id.map(|id| id.to_string()),
                ),
            )?;
            if receipt.is_some_and(|r| r["capability"] == "Stop") ||
                matches!(&event.event_type, gracemo_types::EventType::AutonomyStateChanged { state, .. } if state == "EMERGENCY_STOP")
            {
                tx.execute("INSERT OR IGNORE INTO emergency_holds(event_id) VALUES(?1)", [event.id.to_string()])?;
            }
            tx.commit()?;
        }
        Ok(())
    }

    pub fn record_decision(&self, event:&Event, adapter:&str, outcome:&str, detail:&Value)->Result<()> {
        let db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
        Self::insert_decision(&db,event,adapter,outcome,detail)
    }

    fn insert_decision(db:&Connection,event:&Event,adapter:&str,outcome:&str,detail:&Value)->Result<()> {
        db.execute("INSERT INTO action_decisions(action_id,trace_id,adapter_id,mission_id,authorization_id,
            outcome,recorded_at,detail) VALUES (?1,?2,?3,?4,?5,?6,unixepoch(),?7)",
            (event.id.to_string(),event.trace_id.to_string(),adapter,detail["mission_id"].as_str(),
             detail["authorization_id"].as_str(),outcome,detail.to_string()))?;
        Ok(())
    }

    pub fn decisions(&self,adapter:&str,trace:Option<&str>)->Result<Vec<Value>>{
        let db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
        let mut stmt=db.prepare("SELECT action_id,trace_id,outcome,recorded_at,detail FROM action_decisions
            WHERE adapter_id=?1 AND (?2 IS NULL OR trace_id=?2) ORDER BY sequence DESC LIMIT 200")?;
        let rows=stmt.query_map((adapter,trace), |r|Ok((r.get::<_,String>(0)?,r.get::<_,String>(1)?,
            r.get::<_,String>(2)?,r.get::<_,i64>(3)?,r.get::<_,String>(4)?)))?;
        let mut decisions=Vec::new();
        for row in rows {
            let (action,trace,outcome,time,detail)=row?;
            decisions.push(json!({"action_id":action,"trace_id":trace,"outcome":outcome,
                "recorded_at":time,"detail":serde_json::from_str::<Value>(&detail)?}));
        }
        Ok(decisions)
    }

    /// Snapshot recorded starts and their last correlated coordinator observation.
    /// No action is published while reconstructing this view.
    pub fn recover_missions(&self)->Result<Vec<Value>> {
        let db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
        let mut starts=db.prepare("SELECT id,trace_id,payload,timestamp FROM events
            WHERE json_extract(payload,'$.type')='ActionRequested'
            AND json_extract(payload,'$.data.action')='StartMission' ORDER BY rowid")?;
        let mut latest=db.prepare("SELECT id,payload,timestamp FROM events WHERE trace_id=?1
            AND json_extract(payload,'$.type')='AutonomyStateChanged'
            AND json_extract(payload,'$.data.mission_id')=?2 ORDER BY
            json_extract(payload,'$.data.state') IN ('MISSION_COMPLETED','MISSION_CANCELLED') DESC,rowid DESC LIMIT 1")?;
        let mut result=Vec::new();
        let mut rows=starts.query([])?;
        while let Some(row)=rows.next()? {
            let trace:String=row.get(1)?;
            let action:Value=serde_json::from_str(&row.get::<_,String>(2)?)?;
            let mission=&action["data"]["params"]["mission_id"];
            let mut events=latest.query((&trace,mission.as_str()))?;
            let last=if let Some(event)=events.next()? {
                let payload:Value=serde_json::from_str(&event.get::<_,String>(1)?)?;
                json!({"id":event.get::<_,String>(0)?,"state":payload["data"]["state"],"timestamp":event.get::<_,i64>(2)?})
            } else {Value::Null};
            let terminal=matches!(last["state"].as_str(),Some("MISSION_COMPLETED"|"MISSION_CANCELLED"));
            result.push(json!({"mission_id":mission,"trace_id":trace,"action_id":row.get::<_,String>(0)?,
                "requested_at":row.get::<_,i64>(3)?,"last_recorded_observation":last,
                "requires_operator_review":!terminal,"automatic_resume":false}));
        }
        Ok(result)
    }

    pub fn emergency_hold(&self) -> Result<Option<String>> {
        use rusqlite::OptionalExtension;
        let db = self.conn.lock().map_err(|_| anyhow::anyhow!("ledger unavailable"))?;
        Ok(db.query_row("SELECT event_id FROM emergency_holds WHERE released_by IS NULL
                         ORDER BY sequence DESC LIMIT 1", [], |row| row.get(0)).optional()?)
    }

    pub fn release_emergency_hold(&self, id: &str, operator: &str) -> Result<bool> {
        let mut db = self.conn.lock().map_err(|_| anyhow::anyhow!("ledger unavailable"))?;
        let tx = db.transaction()?;
        let n = tx.execute("UPDATE emergency_holds SET released_by=?2, released_at=unixepoch()
            WHERE released_by IS NULL AND ?1=(SELECT event_id FROM emergency_holds
            WHERE released_by IS NULL ORDER BY sequence DESC LIMIT 1)", (id, operator))?;
        tx.commit()?;
        Ok(n > 0)
    }

    pub fn reviewed(&self,id:&str)->Result<bool>{
        let db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
        Ok(db.query_row("SELECT EXISTS(SELECT 1 FROM operator_reviews WHERE action_id=?1)",[id],|r|r.get(0))?)
    }

    pub fn abandon_interrupted(&self, action:&str, trace:&str, mission:&str, operator:&str)->Result<()> {
        let db=self.conn.lock().map_err(|_|anyhow::anyhow!("ledger unavailable"))?;
        db.execute("INSERT INTO operator_reviews VALUES(?1,?2,?3,?4,unixepoch(),'ABANDON')",
            (action,trace,mission,operator))?;
        Ok(())
    }

    pub fn query_recent(&self, limit: usize) -> Vec<Value> {
        self.query_history(limit, None)
    }

    pub fn query_history(&self, limit: usize, trace: Option<&str>) -> Vec<Value> {
        let mut results = Vec::new();
        if let Ok(db) = self.conn.lock() {
            if let Ok(mut stmt) = db.prepare(
                "SELECT id, trace_id, timestamp, source, observed_by, event_type, payload, session_id, parent_event_id FROM events WHERE (?2 IS NULL OR trace_id=?2) ORDER BY timestamp DESC, rowid DESC LIMIT ?1",
            ) {
                if let Ok(rows) = stmt.query_map((limit,trace), |row| {
                    Ok(json!({
                        "id": row.get::<_, String>(0)?,
                        "trace_id": row.get::<_, String>(1)?,
                        "timestamp": row.get::<_, i64>(2)?,
                        "source": row.get::<_, String>(3)?,
                        "observed_by": row.get::<_, String>(4)?,
                        "event_type": row.get::<_, String>(5)?,
                        "payload": row.get::<_, String>(6)?,
                        "session_id": row.get::<_, Option<String>>(7)?,
                        "parent_event_id": row.get::<_, Option<String>>(8)?,
                    }))
                }) {
                    for r in rows.flatten() {
                        results.push(r);
                    }
                }
            }
        }
        results
    }
}
