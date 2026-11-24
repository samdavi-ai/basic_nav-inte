#!/usr/bin/env bash
# Compatibility entry point for the supported ROS 2 Jazzy / Gazebo Harmonic stack.
# The former Humble+distrobox script killed every Gazebo process on the machine;
# keep this name for users, but delegate lifecycle ownership to the safe launcher.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/launch_full_simulation.sh" "$@"
