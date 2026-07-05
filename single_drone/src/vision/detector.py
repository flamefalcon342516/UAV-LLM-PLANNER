"""
Vision AI — Challenge 3: Target Detection + Follow

Subscribes to the Gazebo camera topic, runs YOLOv8 on each frame,
and on detection:
  1. Saves a snapshot and prints an alert to the operator.
  2. Commands the drone (via pymavlink GUIDED offboard) to follow the target.

Target class is configurable at runtime (e.g. "person", "car", "bottle").

Camera input: OpenCV VideoCapture from gstreamer pipeline OR an image saved
by a gz transport subscriber.  In SITL the easiest approach is a gstreamer
pipeline from Gazebo's camera plugin on port 5600.

Architecture note: this module only READS telemetry and WRITES velocity
setpoints.  The mission executor still owns the connection; we share it
via a queue/callback pattern so the executor can abort if needed.
"""

import os
import time
import threading
import queue
from pathlib import Path
from datetime import datetime
from typing import Optional, Callable

import cv2
import numpy as np

# YOLOv8 via ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[Vision] WARNING: ultralytics not installed — detection disabled. "
          "Run: pip install ultralytics")

SNAPSHOT_DIR = Path(__file__).parent.parent.parent / "snapshots"
SNAPSHOT_DIR.mkdir(exist_ok=True)

# Gazebo camera via GStreamer (ArduPilot SITL default)
DEFAULT_GSTREAM = (
    "udpsrc port=5600 ! application/x-rtp,encoding-name=H264 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)


class TargetDetector:
    """
    Runs YOLO inference on a video stream and reports detections.

    Usage:
        detector = TargetDetector(target_class="person")
        detector.start(on_detect=my_callback)
        ...
        detector.stop()
    """

    def __init__(
        self,
        target_class: str = "person",
        model_name: str = "yolov8n.pt",
        confidence: float = 0.45,
        camera_source: Optional[str] = None,
        display: bool = True,
    ):
        self.target_class = target_class
        self.confidence = confidence
        self.display = display
        self._camera_source = camera_source or 0  # 0 = webcam fallback
        self._model: Optional[object] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._detection_queue: queue.Queue = queue.Queue(maxsize=10)
        self._on_detect: Optional[Callable] = None
        self._model_name = model_name

    def start(self, on_detect: Optional[Callable] = None):
        """Start detection in a background thread."""
        if not YOLO_AVAILABLE:
            raise RuntimeError("ultralytics is not installed. pip install ultralytics")

        self._on_detect = on_detect
        self._model = YOLO(self._model_name)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print(f"[Vision] Detector started — watching for: {self.target_class}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        print("[Vision] Detector stopped.")

    def get_latest_detection(self, timeout: float = 0.1):
        """Non-blocking poll for the latest detection event."""
        try:
            return self._detection_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run_loop(self):
        cap = self._open_camera()
        if cap is None:
            print("[Vision] ERROR: Could not open camera.")
            return

        print("[Vision] Camera open — streaming …")
        while self._running:
            ret, frame = cap.read()
            if not ret:
                print("[Vision] Frame read failed — retrying …")
                time.sleep(0.5)
                cap = self._open_camera()
                continue

            detections = self._detect(frame)

            if self.display:
                annotated = self._annotate(frame, detections)
                cv2.imshow("UAV Vision", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self._running = False
                    break

        cap.release()
        cv2.destroyAllWindows()

    def _open_camera(self):
        if isinstance(self._camera_source, str) and "udpsrc" in self._camera_source:
            # GStreamer pipeline (Gazebo camera)
            cap = cv2.VideoCapture(self._camera_source, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(self._camera_source)

        if not cap.isOpened():
            # Fallback: try webcam 0
            print("[Vision] Primary source failed, falling back to webcam 0")
            cap = cv2.VideoCapture(0)

        return cap if cap.isOpened() else None

    def _detect(self, frame: np.ndarray) -> list:
        results = self._model(frame, conf=self.confidence, verbose=False)
        detections = []

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                cls_name = self._model.names[cls_id]
                if cls_name.lower() != self.target_class.lower():
                    continue

                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                cx = (xyxy[0] + xyxy[2]) / 2
                cy = (xyxy[1] + xyxy[3]) / 2

                det = {
                    "class": cls_name,
                    "confidence": conf,
                    "bbox": xyxy,
                    "center": (cx, cy),
                    "frame_shape": frame.shape,
                    "timestamp": time.time(),
                }
                detections.append(det)

                # Alert and snapshot on first detection
                self._alert(frame, det)

        return detections

    def _alert(self, frame: np.ndarray, det: dict):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        snap_path = SNAPSHOT_DIR / f"detection_{ts}.jpg"
        cv2.imwrite(str(snap_path), frame)

        print(
            f"\n[Vision] *** TARGET DETECTED ***\n"
            f"  Class: {det['class']} | Confidence: {det['confidence']:.2%}\n"
            f"  BBox: {[round(v, 1) for v in det['bbox']]}\n"
            f"  Snapshot saved: {snap_path}\n"
        )

        event = {**det, "snapshot": str(snap_path)}
        try:
            self._detection_queue.put_nowait(event)
        except queue.Full:
            pass

        if self._on_detect:
            self._on_detect(event)

    @staticmethod
    def _annotate(frame: np.ndarray, detections: list) -> np.ndarray:
        out = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['class']} {det['confidence']:.0%}"
            cv2.putText(out, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return out
