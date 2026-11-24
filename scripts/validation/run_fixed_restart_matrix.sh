#!/usr/bin/env bash
# Each read-only source-map trial starts a private camera-only Gazebo world.
set -eu
source_map=docs/validation/visual_all_06/original-patrol01/map.db
output=${1:?fresh destination required}
mkdir "$output"
for room in hallway kitchen bedroom study living; do
  for variant in 0 1; do
    offset=0
    heading=0
    if [[ $variant == 1 ]]; then offset=.25; heading=.35; fi
    if ! bash scripts/validation/run_static_rgbd_trial.sh "$output/$room-$variant" \
         --source "$source_map" --room "$room" --offset "$offset" --heading "$heading" --seconds 15; then
      echo "Trial failed in $room-$variant" >&2
    fi
  done
done
