#!/usr/bin/env bash
# Deprecated compatibility name.  The supported prototype is the standalone
# ROS 2 Jazzy/Gazebo Harmonic launcher, which owns only its own process groups.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/launch_full_simulation.sh" "$@"
