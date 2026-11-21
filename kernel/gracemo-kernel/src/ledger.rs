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
        info!("💾 SQLite Event Ledger active at {:?}", db_path);

        Ok(Self {
            conn: Arc::new(Mutex::new(conn)),
        })
    }

    pub fn record_event(&self, event: &Event) -> Result<()> {
        let payload_str = serde_json::to_string(&event.event_type).unwrap_or_default();
        let source_str = format!("{:?}", event.source);
        let event_type_name = format!("{:?}", event.event_type);
        let short_name = event_type_name
            .split('{')
            .next()
            .unwrap_or("Unknown")
            .trim();

        if let Ok(db) = self.conn.lock() {
            db.execute(
                "INSERT INTO events (id, trace_id, timestamp, source, observed_by, event_type, payload) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
                (
                    event.id.to_string(),
                    event.trace_id.to_string(),
                    event.timestamp,
                    source_str,
                    event.observed_by.clone(),
                    short_name,
                    payload_str,
                ),
            )?;
        }
        Ok(())
    }

    pub fn query_recent(&self, limit: usize) -> Vec<Value> {
        let mut results = Vec::new();
        if let Ok(db) = self.conn.lock() {
            if let Ok(mut stmt) = db.prepare(
                "SELECT id, trace_id, timestamp, source, observed_by, event_type, payload FROM events ORDER BY timestamp DESC LIMIT ?1",
            ) {
                if let Ok(rows) = stmt.query_map([limit], |row| {
                    Ok(json!({
                        "id": row.get::<_, String>(0)?,
                        "trace_id": row.get::<_, String>(1)?,
                        "timestamp": row.get::<_, i64>(2)?,
                        "source": row.get::<_, String>(3)?,
                        "observed_by": row.get::<_, String>(4)?,
                        "event_type": row.get::<_, String>(5)?,
                        "payload": row.get::<_, String>(6)?,
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
