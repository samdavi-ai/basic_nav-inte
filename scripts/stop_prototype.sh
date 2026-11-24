#!/usr/bin/env bash
# The supported launchers terminate their own process groups on Ctrl+C.  This
# compatibility command must never kill a Gazebo/ROS process it does not own.
set -euo pipefail

printf '%s\n' 'No global shutdown was performed.' \
  'Stop the terminal that started the supported launcher (Ctrl+C); it will publish the simulation safe-stop and reap only its recorded process groups.'
