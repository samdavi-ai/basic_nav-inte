"""RGB-D graph mapping and policy observation alongside a private Gazebo mission."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
import struct
import zlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_camera_records(db, width, height, depth_divisor=2):
    """Validate the single-camera record layout used by the tested Jazzy backend."""
    if db.execute("SELECT version FROM Admin").fetchall() != [('0.23.14',)]:
        raise ValueError("Unverified RTAB-Map database version")
    records = db.execute("""SELECT DISTINCT substr(calibration,1,324),length(calibration),substr(depth,1,26)
                            FROM Data WHERE length(image)>0 AND length(depth)>0""")
    expected_depth = (b'\x89PNG\r\n\x1a\n' + struct.pack('>I4sIIBB', 13, b'IHDR',
                      width // depth_divisor, height // depth_divisor, 8, 6))
    focal = width / (2 * math.tan(1.15 / 2))
    k = (focal, 0, width / 2, 0, focal, height / 2, 0, 0, 1)
    identity = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    projection = (focal, 0, width / 2, 0, 0, focal, height / 2, 0, 0, 0, 1, 0)
    expected = k + (0,) * 5 + identity + projection
    found = False
    for prefix, size, depth_header in records:
        if depth_header != expected_depth:
            raise ValueError("Saved depth image format or resolution differs from the configured camera")
        if size != 372 or struct.unpack_from('<11i', prefix) != (
                0, 23, 14, 0, width, height, 9, 5, 9, 12, 12):
            raise ValueError("Incompatible saved RGB-D camera format or resolution")
        if not all(math.isclose(a, b, abs_tol=1e-5)
                   for a, b in zip(struct.unpack_from('<35d', prefix, 44), expected)):
            raise ValueError("Incompatible saved RGB-D camera intrinsics")
        found = True
    if not found:
        raise ValueError("No calibrated RGB-D keyframes")


def validate_saved_graph(db):
    ids, poses = db.execute("SELECT opt_ids,opt_poses FROM Admin").fetchone()
    if not ids or not poses:
        raise ValueError("Missing optimized keyframe graph")
    node_ids = [i for i, in struct.iter_unpack('<i', zlib.decompress(ids))]
    if not node_ids or len(zlib.decompress(poses)) != 48 * len(node_ids):
        raise ValueError("Invalid saved graph poses")
    nodes = json.dumps(node_ids)
    count, = db.execute("SELECT count(*) FROM Node WHERE id IN (SELECT value FROM json_each(?))",
                        (nodes,)).fetchone()
    if count != len(node_ids):
        raise ValueError("Missing or duplicate optimized keyframe IDs")
    if db.execute("""SELECT 1 FROM Feature f LEFT JOIN Word w ON w.id=f.word_id
                     WHERE f.node_id IN (SELECT value FROM json_each(?))
                     AND f.word_id>0 AND w.id IS NULL LIMIT 1""", (nodes,)).fetchone():
        raise ValueError("Saved graph references missing visual dictionary words")


def mapping_nodes(context):
    database = Path(LaunchConfiguration('database').perform(context)).absolute()
    visual = LaunchConfiguration('visual_odometry').perform(context) == 'true'
    if database.exists():
        raise ValueError(f'Choose a new mapping database path: {database}')
    source = LaunchConfiguration('source_map').perform(context)
    parameters = {}
    if source:
        saved = Path(source).resolve(strict=True)
        expected = LaunchConfiguration('source_sha256').perform(context)
        with saved.open('rb') as source_file:
            fingerprint = hashlib.file_digest(source_file, 'sha256').hexdigest()
        if not visual or fingerprint != expected:
            raise ValueError("Map fingerprint or localization mode mismatch")
        with sqlite3.connect(saved.as_uri() + "?mode=ro", uri=True) as original:
            if original.execute("PRAGMA integrity_check").fetchone() != ("ok",):
                raise ValueError("Invalid map database")
            scale = int(LaunchConfiguration('camera_scale').perform(context))
            divisor = int(LaunchConfiguration('depth_divisor').perform(context))
            validate_camera_records(original, 320 * scale, 240 * scale, divisor)
            validate_saved_graph(original)
            with database.open("xb"):
                pass
            with sqlite3.connect(database) as trial:
                original.backup(trial)
                trial.execute("UPDATE Admin SET opt_last_localization=NULL")
        parameters = {'Mem/IncrementalMemory':'false','Mem/InitWMWithAllNodes':'true',
                      'RGBD/StartAtOrigin':'false' if LaunchConfiguration('blind_localization').perform(context)=='true' else 'true'}
    if visual and LaunchConfiguration('benchmark_camera_rate').perform(context) == 'true':
        # Evaluate every synchronized RGB-D pair. The visual odometry frame
        # intentionally has no navigation TF; its stamped message is available.
        parameters.update({'Rtabmap/DetectionRate': '0', 'wait_for_transform': 0.01})
    nodes = [
        Node(package='rtabmap_slam', executable='rtabmap', namespace='visual_slam',
             name='rtabmap', output='screen', parameters=[{
                 **parameters, 'use_sim_time': True, 'database_path': str(database),
                 'frame_id': 'base_footprint', 'map_frame_id': 'visual_map',
                 'odom_frame_id': '' if visual else 'odom', 'publish_tf': False,
                 'subscribe_depth': True, 'subscribe_rgb': True,
                 'approx_sync': True, 'approx_sync_max_interval': .025,
             }], remappings=[('rgb/image','/camera/image_raw'),
                              ('rgb/camera_info','/camera/camera_info'),
                              ('depth/image','/camera/depth_image')]),
    ]
    if visual:
        descriptor = LaunchConfiguration('descriptor').perform(context)
        odom_params = {'Vis/FeatureType':'1'} if descriptor == 'sift' else {}
        if LaunchConfiguration('distributed_features').perform(context) == 'true':
            odom_params.update({'Vis/GridRows':'2', 'Vis/GridCols':'2', 'GFTT/MinDistance':'3'})
        if LaunchConfiguration('optical_flow').perform(context) == 'true':
            # Track image patches against the previous keyframe.
            odom_params.update({'Odom/Strategy': '1', 'Vis/CorType': '1'})
        if LaunchConfiguration('planar_registration').perform(context) == 'true':
            odom_params['Reg/Force3DoF'] = 'true'
        if LaunchConfiguration('metric_rgbd').perform(context) == 'true':
            # Estimate relative camera motion using depth on both matched images.
            odom_params['Vis/EstimationType'] = '0'
        nodes.append(Node(package='rtabmap_odom', executable='rgbd_odometry',
            namespace='visual_slam', output='screen', parameters=[{
                'use_sim_time': True, 'frame_id': 'base_footprint',
                **odom_params, 'odom_frame_id': 'visual_odom', 'publish_tf': False,
                'approx_sync': True, 'approx_sync_max_interval': .025,
            }], remappings=[('rgb/image','/camera/image_raw'),
                            ('rgb/camera_info','/camera/camera_info'),
                            ('depth/image','/camera/depth_image')]))
        return nodes
    return nodes + [
        Node(package='gracemo_rl', executable='rl_local_planner', name='sim_policy_advisor',
             output='screen', parameters=[{'use_sim_time': True, 'allow_reverse': False, 'inference_threads': 1,
                                           'cmd_vel_topic': '/validation/policy_proposal'}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('database'),
                              DeclareLaunchArgument('descriptor', default_value='default', choices=['default','sift']),
                              DeclareLaunchArgument('optical_flow', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('planar_registration', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('source_map', default_value=''),
                              DeclareLaunchArgument('blind_localization', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('distributed_features', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('depth_divisor', default_value='2', choices=['1','2']),
                              DeclareLaunchArgument('metric_rgbd', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('benchmark_camera_rate', default_value='false', choices=['true','false']),
                              DeclareLaunchArgument('camera_scale', default_value='1', choices=['1','2']),
                              DeclareLaunchArgument('source_sha256', default_value=''),
                              DeclareLaunchArgument('visual_odometry', default_value='false', choices=['true','false']),
                              OpaqueFunction(function=mapping_nodes)])
