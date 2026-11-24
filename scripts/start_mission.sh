#!/usr/bin/env bash
# Dispatch a mission only through the standalone Jazzy simulation pipeline.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT/scripts/run_autonomous_mission.py" "$@"
