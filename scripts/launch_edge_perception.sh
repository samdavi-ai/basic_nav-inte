#!/usr/bin/env bash
# Standalone sensor consumer. The camera driver and TF publisher supply its inputs.
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
profile=${1:-cpu}
if [[ $# -gt 0 ]]; then shift; fi
case "$profile" in
  cpu) hz=5; provider=CPUExecutionProvider ;;
  jetson) hz=5; provider=TensorrtExecutionProvider ;;
  constrained) hz=3; provider=CPUExecutionProvider ;;
  *) echo "Expected cpu, jetson, or constrained" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$root:$root/adapters/sdk:${PYTHONPATH:-}"
export VIRA_DETECTION_HZ=${VIRA_DETECTION_HZ:-$hz}
export VIRA_ORT_PROVIDER=${VIRA_ORT_PROVIDER:-$provider}
export VIRA_DETECTOR_ONNX=${VIRA_DETECTOR_ONNX:-$root/models/yolo11n.onnx}
test -s "$VIRA_DETECTOR_ONNX" || { echo "Detector file missing" >&2; exit 2; }
exec python3 -m adapters.perception.perception_service "$@"
