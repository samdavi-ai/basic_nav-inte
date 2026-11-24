#!/usr/bin/env bash
# Start (or reuse) the local simulation kernel without disturbing another service.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$ROOT/kernel/target/debug/gracemo-kernel"

if command -v curl >/dev/null 2>&1 && curl --fail --silent --max-time 1 http://127.0.0.1:7780/health >/dev/null; then
    printf 'GRaCEmo kernel is already healthy at http://127.0.0.1:7780; leaving it untouched.\n'
    exit 0
fi

if [[ ! -x "$BIN" ]]; then
    printf 'Building gracemo-kernel...\n'
    cargo build --manifest-path "$ROOT/kernel/Cargo.toml" --bin gracemo-kernel
fi

printf 'Starting GRaCEmo kernel on http://127.0.0.1:7780...\n'
exec "$BIN"
