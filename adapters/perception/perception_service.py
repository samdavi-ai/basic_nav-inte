"""
GRaCEmo ViRa — Perception Service
ROS 2 Perception Node that synchronizes RGB + Depth camera frames, executes YOLO11
detection, calculates robust 3D metric positions via camera intrinsics, transforms them
into the canonical world frame via TF, and emits canonical ObjectDetected events to the Kernel.
"""

import os
import time
import logging
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import psutil
from .person_tracks import PersonTracks
from typing import Optional, List, Dict, Any, Callable

import cv2
import numpy as np
import requests

from .models import BoundingBox, Point3D, Detection, PerceptionResult
from .detector import ObjectDetector
from .depth_estimator import DepthEstimator
from .coordinate_transformer import CoordinateTransformer
from .visualizer import PerceptionVisualizer

logger = logging.getLogger("perception.service")


class PerceptionService:
    """
    Core perception engine integrating detector, depth association, 3D projection,
    TF transformation, kernel event emission, and visual debugging.
    Can be used standalone or embedded inside a ROS 2 node.
    """

    def __init__(
        self,
        kernel_url: str = "http://127.0.0.1:7780",
        confidence_threshold: float = 0.35,
        model_name: str = "yolo11n.pt",
        neighborhood_size: int = 7,
        emit_to_kernel: bool = True,
        min_emit_interval_sec: float = 0.5,
        save_debug_frames: bool = True,
        debug_frame_path: str = "/tmp/gracemo_perception_debug.jpg",
        ros_node=None,
        target_frame="world",
    ):
        self.kernel_url = kernel_url
        self.emit_to_kernel = emit_to_kernel
        self.min_emit_interval_sec = min_emit_interval_sec
        self.save_debug_frames = save_debug_frames
        self.debug_frame_path = debug_frame_path
        self.last_emit_time: Dict[str, float] = {}

        self.detector = ObjectDetector(model_name=model_name, confidence_threshold=confidence_threshold)
        self.depth_estimator = DepthEstimator(neighborhood_size=neighborhood_size)
        self.transformer = CoordinateTransformer(target_frame=target_frame, node=ros_node)
        self.visualizer = PerceptionVisualizer()

        self.mission = {}
        self.tracker = PersonTracks()
        self.people = []
        self.tracking_ms = None
        self.ingest_ms = None
        self.detector_ms = None
        self.reid_enabled = os.getenv("VIRA_REID", "0") == "1"
        self.face_enabled = os.getenv("VIRA_FACE", "0") == "1"
        self.face_engine = None
        self.face_samples = {}
        self.face_status = "disabled"
        self.degraded = False
        self.latest_result: Optional[PerceptionResult] = None
        self.detection_callbacks: List[Callable[[Detection], None]] = []

    def register_callback(self, cb: Callable[[Detection], None]):
        """Registers a listener callback for newly verified 3D detections."""
        self.detection_callbacks.append(cb)

    def set_camera_info(self, camera_info_msg: Any):
        """Updates camera intrinsics directly from sensor_msgs/CameraInfo."""
        self.depth_estimator.update_camera_info(camera_info_msg)

    def process_frame(
        self,
        image_bgr: np.ndarray,
        depth_map: np.ndarray,
        fallback_tf=None,
        stamp=None,
    ) -> PerceptionResult:
        """
        Executes the full perception pipeline on a synchronized pair of RGB and Depth frames.

        1. Runs YOLO object detector on RGB frame -> 2D BBoxes
        2. Associates depth at bbox center with median filtering -> Depth (m)
        3. Pinhole back-projection using CameraInfo -> Camera-frame (Xc, Yc, Zc)
        4. TF tree transformation -> World-frame (Xw, Yw, Zw)
        5. Builds structured Detection objects
        6. Emits canonical ObjectDetected events to Kernel (throttled)
        7. Renders visual debug overlay
        """
        now = time.time()
        result = PerceptionResult(timestamp=now)

        if image_bgr is None or depth_map is None:
            return result

        result.frame_height, result.frame_width = image_bgr.shape[:2]
        result.depth_height, result.depth_width = depth_map.shape[:2]

        # 1. Object Detection
        start = time.perf_counter()
        raw_detections = self.detector.detect(image_bgr)
        self.detector_ms = (time.perf_counter()-start)*1000
        start = time.perf_counter()
        capture = stamp.nanoseconds/1e9 if stamp is not None else time.monotonic()
        self.people = self.tracker.update([row for row in raw_detections if row[0]=="person"],
                                           image_bgr, capture, self.reid_enabled and not self.degraded)
        self.tracking_ms = (time.perf_counter()-start)*1000
        self._update_faces(image_bgr, capture)


        # 2. Depth Association & 3D Spatial Calculation
        for class_name, conf, bbox in raw_detections:
            depth_m, pos_camera = self.depth_estimator.estimate_3d_position(
                bbox=bbox,
                depth_img=depth_map,
                rgb_width=result.frame_width,
                rgb_height=result.frame_height,
            )

            if depth_m is None or pos_camera is None:
                continue

            # 3. Transform to World Frame
            pos_world = self.transformer.transform_to_world(pos_camera, fallback_transform=fallback_tf, stamp=stamp)
            if pos_world is None:
                # A camera-frame point cannot update the metric world graph without TF.
                continue

            detection = Detection(
                class_name=class_name,
                confidence=conf,
                bbox=bbox,
                depth_m=depth_m,
                position_camera=pos_camera,
                position_world=pos_world,
                timestamp=now,
            )
            track = next((p for p in self.people if class_name == "person" and p["bbox"] == list(bbox.as_tuple())), None)
            if track:
                detection.id = track["detection_id"]
                detection.track_id = track["track_id"]
            detection.depth_patch_mad_m = self.depth_estimator.last_depth_mad_m
            detection.camera_id = pos_camera.frame_id
            detection.image_stamp_ns = stamp.nanoseconds if stamp is not None else None
            result.detections.append(detection)

            # Notify registered callbacks (e.g. gaze bridge)
            for cb in self.detection_callbacks:
                try:
                    cb(detection)
                except Exception as e:
                    logger.error(f"Error in detection callback: {e}")

            # 4. Throttled Kernel Event Emission
            if self.emit_to_kernel:
                self._emit_detection_to_kernel(detection)

        # 5. Visual Debug Overlay
        if self.save_debug_frames:
            annotated = self.visualizer.annotate_frame(
                image_bgr,
                result.detections,
                save_path=self.debug_frame_path if self.save_debug_frames else None,
            )
            result.annotated_frame = annotated

        self.latest_result = result
        return result

    def _update_faces(self, image_bgr: np.ndarray, capture: float) -> None:
        """Attach a bounded, persistent face status to each live person track."""
        if self.degraded:
            self.face_status = "deferred_resource_limit"
            return
        if not self.face_enabled:
            self.face_status = "disabled"
            return

        self.face_status = "awaiting_person"
        self.face_samples = {key: value for key, value in self.face_samples.items()
                             if key in self.tracker.tracks}
        height, width = image_bgr.shape[:2]
        for person in self.people:
            track_id = person["track_id"]
            previous = self.face_samples.get(track_id)
            x1, y1, x2, y2 = map(int, person["bbox"])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            crop = image_bgr[y1:y2, x1:x2] if x2 > x1 and y2 > y1 else None

            if crop is None or not crop.size:
                # A malformed/clipped ROI is an observation failure, not a
                # reason to abort the entire RGB-D frame or expose stale data.
                sample = dict(face_count=None, face_state="FACE_ROI_INVALID", face_stamp_s=capture)
                self.face_samples[track_id] = (capture, None, sample)
            elif previous is None or capture - previous[0] >= 0.5:
                thumbnail = cv2.resize(crop, (16, 16), interpolation=cv2.INTER_AREA)
                if previous is None or previous[1] is None or cv2.norm(thumbnail, previous[1], cv2.NORM_INF) > 8:
                    if self.face_engine is None:
                        from adapters.vision.gracemo_vision.face_id import FaceRecognizer
                        self.face_engine = FaceRecognizer({})
                    faces = self.face_engine.detect_faces(crop)
                    available = ((self.face_engine.detector is not None and not self.face_engine.failed)
                                 or self.face_engine.cascade is not None)
                    state = "FACE_NOT_FOUND" if available else "FACE_PIPELINE_UNAVAILABLE"
                    if faces:
                        state = "FACE_DETECTED"
                    sample = dict(face_count=len(faces), face_state=state, face_stamp_s=capture)
                    self.face_samples[track_id] = (capture, thumbnail, sample)

            # The sample is guaranteed above for a new/invalid ROI. A changed
            # ROI updates it; a stable ROI keeps the last explicit result.
            saved = self.face_samples.get(track_id)
            if saved is not None:
                person.update(saved[2])
                self.face_status = saved[2]["face_state"]

    def _emit_detection_to_kernel(self, detection: Detection):
        """Emits canonical ObjectDetected event to the Kernel if throttled period elapsed."""
        now = time.time()
        key = detection.track_id or detection.class_name
        if len(self.last_emit_time)>128:
            self.last_emit_time = {k:v for k,v in self.last_emit_time.items() if now-v<self.min_emit_interval_sec}
        last_time = self.last_emit_time.get(key, 0.0)
        if now - last_time < self.min_interval_for(detection.class_name):
            return

        envelope = detection.to_canonical_event(source="Vision", observed_by="perception_service")
        if self.mission.get("trace_id"):
            envelope.update(trace_id=self.mission["trace_id"],session_id=self.mission.get("id"))
        try:
            start = time.perf_counter()
            resp = requests.post(f"{self.kernel_url}/emit", json=envelope, timeout=1.0)
            self.ingest_ms = (time.perf_counter()-start)*1000 if resp.ok else None
            if resp.status_code == 200:
                self.last_emit_time[key] = now
                logger.debug(f"✓ Emitted ObjectDetected for '{detection.class_name}' at world ({detection.position_world.x:.2f}, {detection.position_world.y:.2f})")
        except Exception:
            pass

    def min_interval_for(self, class_name: str) -> float:
        return self.min_emit_interval_sec


def create_ros_perception_node(node_name: str = "gracemo_perception_node"):
    """
    Factory function creating a full ROS 2 Node wrapping PerceptionService,
    subscribing to /camera/image_raw, /camera/depth_image, /camera/camera_info.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage, CameraInfo as RosCameraInfo

    class RosPerceptionNode(Node):
        def __init__(self):
            super().__init__(node_name)

            from message_filters import Subscriber, ApproximateTimeSynchronizer
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import String
            from gracemo_sdk import AdapterClient
            self.kernel = AdapterClient("perception_service", os.getenv("VIRA_KERNEL_URL","http://127.0.0.1:7780"))
            self.status_pub = self.create_publisher(String, "/perception/status", 10)
            self.service = PerceptionService(ros_node=self, target_frame="map", save_debug_frames=False,
                kernel_url=os.environ.get("VIRA_KERNEL_URL", "http://127.0.0.1:7780"))
            self.frames = 0
            self.count = None
            self.observations = []
            self.captured_at = None
            self.image_stamp_ns = None
            self.last_processed = None
            self.pending = None
            self.pairs = self.dropped = self.rejected = 0
            self.sync_skew_ms = None
            self.input_rejections = {}
            self.worker = ThreadPoolExecutor(max_workers=1)
            self.status_worker = ThreadPoolExecutor(max_workers=1)
            self.job = self.status_job = None
            self.previous_status_stamp = None
            self.stats = {}
            from .edge_health import BoardSensors
            self.board = BoardSensors()
            cv2.setNumThreads(2)
            self.process = psutil.Process()
            self.visual_odom = None
            if os.getenv("VIRA_VISUAL_ODOMETRY") == "1":
                from .visual_odometry import VisualOdometry
                self.visual_odom = VisualOdometry()
            self.started = time.monotonic()
            self.hz = float(self.declare_parameter("detection_hz",float(os.getenv("VIRA_DETECTION_HZ","5"))).value)
            if not .1 <= self.hz <= 30:
                raise ValueError("detection_hz outside 0.1–30 Hz")
            self.rgb = Subscriber(self, RosImage, "/camera/image_raw", qos_profile=qos_profile_sensor_data)
            self.depth = Subscriber(self, RosImage, "/camera/depth_image", qos_profile=qos_profile_sensor_data)
            self.sync = ApproximateTimeSynchronizer([self.rgb,self.depth],queue_size=6,slop=.10)
            self.sync.registerCallback(self._pair)
            self.create_subscription(RosCameraInfo, "/camera/camera_info", self._calibrate, qos_profile_sensor_data)
            self.active_mission = {}
            self.create_subscription(String,"/autonomy/mission_status",self._mission,10)
            self.calibration = None
            self.create_timer(1/self.hz,self._process_tick)
            self.create_timer(1.,self._status)

        def _mission(self, msg):
            try:
                self.active_mission = json.loads(msg.data).get("mission") or {}
            except (ValueError, AttributeError):
                self.active_mission = {}

        def _pair(self,rgb,depth):
            self.pairs += 1
            self.dropped += int(self.pending is not None)
            self.pending=(rgb,depth,self.calibration,self.active_mission)

        def _calibrate(self, info):
            self.calibration = info

        def destroy_node(self):
            self.worker.shutdown(wait=True,cancel_futures=True)
            self.status_worker.shutdown(wait=True,cancel_futures=True)
            if self.visual_odom is not None:
                try:
                    self.visual_odom.save()
                except OSError as exc:
                    self.get_logger().error(f"Visual map save failed: {exc}")
            return super().destroy_node()

        def _status(self):
            import json
            from std_msgs.msg import String
            age = None if self.last_processed is None else time.monotonic()-self.last_processed
            model_ready=self.service.detector.ort_session is not None or self.service.detector.yolo_model is not None
            msg=String()
            status=dict(model_loaded=model_ready,frames_processed=self.frames,
                age_s=age,ready=model_ready and age is not None and age<3,
                localized_detections_last_frame=self.count,observations=self.observations,captured_at=self.captured_at,
                image_stamp_ns=self.image_stamp_ns)
            ram = psutil.virtual_memory()
            elapsed = max(.001,time.monotonic()-self.started)
            board = self.board.read()
            temp = board["temperature_c"]
            self.service.degraded = ram.available < 512*1024**2 or (temp is not None and temp >= 85)
            with self.sync.lock:
                sync_occupancy = [len(queue) for queue in self.sync.queues]
            status.update(self.stats, rgb_sync_occupancy=sync_occupancy[0], depth_sync_occupancy=sync_occupancy[1],
                          pending_pair_occupancy=int(self.pending is not None),
                          synchronized_pairs=self.pairs,dropped_pending=self.dropped,
                          rejected_inputs=self.rejected,input_rejections=self.input_rejections,process_rss_bytes=self.process.memory_info().rss,
                          cpu_percent=self.process.cpu_percent(),
                          available_ram_bytes=ram.available,degraded=self.service.degraded,
                          acquired_pair_hz=self.pairs/elapsed,processed_hz=self.frames/elapsed,
                          detector_target_hz=self.hz, **board,requested_provider=os.getenv("VIRA_ORT_PROVIDER","CPUExecutionProvider"))
            if not status["ready"] and "visual_odometry" in status:
                status["visual_odometry"] = {"state": "stale", "pose_current": False}
            msg.data=json.dumps(status)
            changed = (self.image_stamp_ns,status["ready"]) != self.previous_status_stamp
            if changed and (self.status_job is None or self.status_job.done()):
                self.previous_status_stamp = (self.image_stamp_ns,status["ready"])
                self.status_job = self.status_worker.submit(self.kernel.emit, "Custom",
                    {"name":"rgbd_perception_status","payload":status},source="Vision")
            self.status_pub.publish(msg)

        def _process_tick(self):
            if self.job is not None:
                if not self.job.done():
                    return
                try:
                    outcome = self.job.result()
                except Exception as error:
                    self.get_logger().error(f"Perception worker failed: {type(error).__name__}")
                    outcome = "worker_exception"
                self.job = None
                if isinstance(outcome, str):
                    self.rejected += 1
                    self.input_rejections[outcome] = self.input_rejections.get(outcome, 0) + 1
                else:
                    self.frames += 1
                    self.observations,self.captured_at,self.image_stamp_ns,self.stats = outcome
                    self.count = len(self.observations)
                    self.last_processed = time.monotonic()
            if self.pending is not None:
                pair = self.pending
                self.pending = None
                self.job = self.worker.submit(self._process, *pair)

        def _process(self, rgb, depth, calibration, mission):
            from rclpy.time import Time
            if (calibration is None or calibration.header.frame_id != rgb.header.frame_id
                    or (calibration.width, calibration.height) != (rgb.width,rgb.height)
                    or not all(np.isfinite(k) and k>0 for k in (calibration.k[0],calibration.k[4]))):
                return "calibration"
            self.service.mission = mission
            self.service.set_camera_info(calibration)
            if self.service.detector.ort_session is None and self.service.detector.yolo_model is None:
                return "detector_unavailable"
            if rgb.encoding not in ("rgb8","bgr8") or depth.encoding!="32FC1" or rgb.header.frame_id!=depth.header.frame_id:
                return "encoding_or_frame"
            stamp=Time.from_msg(rgb.header.stamp)
            now = self.get_clock().now()
            ages = [(now - t).nanoseconds for t in (stamp, Time.from_msg(depth.header.stamp))]
            if min(ages) < 0:
                return "ahead_of_clock"
            if max(ages) > 1_000_000_000:
                return "stale"
            if self.image_stamp_ns is not None and stamp.nanoseconds <= self.image_stamp_ns:
                return "duplicate_or_out_of_order"
            image=np.ndarray((rgb.height,rgb.width,3),np.uint8,buffer=rgb.data,strides=(rgb.step,3,1))
            if rgb.encoding=="rgb8":
                image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            distances=np.ndarray((depth.height,depth.width),dtype=">f4" if depth.is_bigendian else "<f4",
                                 buffer=depth.data,strides=(depth.step,4))
            captured = time.time()-(self.get_clock().now()-stamp).nanoseconds*1e-9
            result = self.service.process_frame(image,distances,stamp=stamp)
            metrics = dict(last_ingest_ack_ms=self.service.ingest_ms, mission_id=mission.get("id"), trace_id=mission.get("trace_id"),person_tracks=self.service.people, detector_ms=self.service.detector_ms,
                           tracking_ms=self.service.tracking_ms,face_status=self.service.face_status,
                           reid_enabled=self.service.reid_enabled, camera_frame=rgb.header.frame_id,
                           sync_skew_ms=abs((Time.from_msg(depth.header.stamp)-stamp).nanoseconds)/1e6,
                           providers=self.service.detector.ort_session.get_providers()
                                     if self.service.detector.ort_session else [])
            if self.visual_odom is not None:
                start = time.perf_counter()
                metrics["visual_odometry"] = self.visual_odom.update(
                    image, distances, calibration.k, stamp.nanoseconds, self.service.people)
                metrics["visual_odometry"]["latency_ms"] = (time.perf_counter()-start)*1000
            return [d.to_dict() for d in result.detections],captured,stamp.nanoseconds,metrics

    return RosPerceptionNode

if __name__ == "__main__":
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = create_ros_perception_node()()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
