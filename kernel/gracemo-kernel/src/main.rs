use gracemo_kernel::api::{create_router, process_incoming_event, AppState};
use gracemo_kernel::graph::GraphDb;
use gracemo_kernel::inspector_state::create_shared_state;
use gracemo_kernel::ledger::Ledger;
use gracemo_types::Event;
use std::{fs, net::SocketAddr, path::PathBuf};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    net::UnixListener,
    sync::broadcast,
};
use tracing::{info, Level};
use tracing_subscriber::FmtSubscriber;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::INFO)
        .finish();
    tracing::subscriber::set_global_default(subscriber)?;

    info!("🧠 Initializing GRaCEmo Kernel v0.0.3 (Dual-Memory Grounded Substrate)...");

    // A Gazebo fixture can coexist with the standalone kernel without sharing
    // its command socket, HTTP listener, or memory databases.
    let isolated = std::env::args().any(|arg| arg == "--isolated-simulation");
    let db_dir = if isolated {
        PathBuf::from(std::env::var("GRACEMO_STATE_DIR")?)
    } else {
        let home_dir = std::env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
        PathBuf::from(home_dir).join(".gracemo")
    };
    std::fs::create_dir_all(&db_dir)?;

    // Serialize isolated owners before reclaiming a socket left by a prior run.
    // The OS releases this lock even if the process is terminated abruptly.
    let _simulation_owner = if isolated {
        let lock = fs::OpenOptions::new().create(true).truncate(false).write(true)
            .open(db_dir.join("kernel.lock"))?;
        lock.try_lock()?;
        Some(lock)
    } else {
        None
    };

    // 1. Initialize SQLite Ledger (PAST) & Graph Memory (RELATIONS)
    let ledger = Ledger::open()?;
    let graph_path = db_dir.join("graph.db");
    let graph = GraphDb::open(&graph_path)?;
    info!("🕸️ SQLite Relational Graph active at {:?}", graph_path);

    // 2. Initialize In-Memory InspectorState (NOW)
    let live_state = create_shared_state();
    let (tx, _rx) = broadcast::channel::<Event>(2048);

    let state = AppState {
        recovered_missions: std::sync::Arc::new(ledger.recover_missions()?),
        registry: gracemo_kernel::authorization::Registry::load()?,
        tx: tx.clone(),
        live_state: live_state.clone(),
        ledger: ledger.clone(),
        graph: graph.clone(),
    };

    // 3. Start Unix Domain Socket Listener (/tmp/gracemo.sock)
    let socket_path = if isolated {
        let path = db_dir.join("gracemo.sock");
        use std::os::unix::fs::FileTypeExt;
        if fs::symlink_metadata(&path).is_ok_and(|m| m.file_type().is_socket()) {
            fs::remove_file(&path)?;
        }
        path
    } else {
        let path = PathBuf::from("/tmp/gracemo.sock");
        let _ = fs::remove_file(&path);
        path
    };
    let unix_listener = UnixListener::bind(&socket_path)?;
    info!("🔌 Unix Domain Socket listening at {}", socket_path.display());

    let socket_state = state.clone();
    tokio::spawn(async move {
        while let Ok((stream, _)) = unix_listener.accept().await {
            let state_clone = socket_state.clone();
            tokio::spawn(async move {
                let reader = BufReader::new(stream);
                let mut lines = reader.lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if let Ok(event) = serde_json::from_str::<Event>(&line) {
                        if let Err(reason) = process_incoming_event(state_clone.clone(), event).await {
                            tracing::warn!("Rejected socket event: {}", reason);
                        }
                    }
                }
            });
        }
    });

    // 4. Start Axum HTTP REST, Context Compiler & SSE API
    let app = create_router(state);
    let port = if isolated { 0 } else { 7780 };
    let listener = tokio::net::TcpListener::bind(SocketAddr::from(([127, 0, 0, 1], port))).await?;
    let addr = listener.local_addr()?;
    info!("🚀 GRaCEmo Nervous System HTTP API listening on http://{}", addr);
    if isolated {
        let pending = db_dir.join("listen.tmp");
        fs::write(&pending, addr.port().to_string())?;
        fs::rename(pending, db_dir.join("listen.port"))?;
    }
    axum::serve(listener, app).await?;

    Ok(())
}
