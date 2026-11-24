# Geometry acceptance

**Gate A failed.** The earlier revised doorway patrol reached 11/11 and overlapped both an exact wall and the projected kitchen chair. The wall witness at a native Gazebo pose is −15.751 mm. The independent oracle brackets the worst linearly interpolated translation/shortest-yaw clearance between −16.023 and −16.008 mm. The original study turn already had an interpolated overlap of 0.782–0.799 mm. These are padded footprint results. [Wall geometry and witnesses](../master_repair/GEOMETRY_VALIDATION.md)

The recorded local footprint is a 0.72 × 0.70 m rectangle. With half dimensions 0.36 and 0.35 m and orientation θ, each world vertex is the model XY position plus its signed half-dimension vector rotated by θ. The corner radius is 0.5021 m. The apartment SDF defines base_link directly above base_footprint by 0.115 m with zero horizontal and yaw offset. The launch used a generated copy of that world. An independent dump of the running model's complete frame graph was not captured. [World definition](../../../ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world)

## Furniture replay

The offline replay loads the cached Fuel chair v3 OBJ using Gazebo's mesh loader, applies the collision scale, link yaw and each chair's world placement, then unions the projected triangles. It rotates the padded rectangular footprint separately at every native pose. A center-distance bound using both enclosing radii skips poses only when they cannot improve the minimum or overlap. GEOS measures polygon distance and intersection area. Distances are unsigned here; overlap area distinguishes penetration from edge contact.

Of 38,146 native poses on the revised doorway patrol, 23 overlapped chair_1. Peak overlap was 0.00021191 m² at 44.204 s. For chair_2 the closest distance was 1.8410 m. The prior −24.506 mm measurement used the enclosing box. The mesh result confirms a planar navigation violation; it does not measure 3D chassis contact, penetration depth or overlap between samples. [Mesh hash, poses and measurements](doorway_chairs.json)

## Candidate

The private planner uses differential-drive lattice primitives at 0.05 m resolution and 0.5 m turn radius without reverse expansion. The guard transforms the map plan into odom at the odometry stamp and checks interpolated rectangles through the requested shim turn against occupied cell squares. Subdivision caps center-plus-corner travel per interval and adds a half-interval margin. Turn checking uses a path-derived target heading. Alignment with the controller's actual target, including translation during slip, remains unqualified.

The runner still uses the private revised staging configuration. The candidate has to reach and safely traverse the kitchen and study to supersede the earlier failure. Positive clearance on a prefix cannot qualify either turn. See the [per-run results](results.json).
