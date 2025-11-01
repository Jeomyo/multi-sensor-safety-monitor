#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import yaml
import torch
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.trackers.byte_tracker import BYTETracker

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose


# ------------------------- Paths -------------------------
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
WEIGHT_PATH = os.path.join(PKG_DIR, 'best.pt')
TRACKER_PATH = os.path.join(PKG_DIR, 'bytetrack.yaml')


class YOLOv10Node(Node):
    def __init__(self):
        super().__init__('yolov10_node')

        # ROS I/O
        self.bridge = CvBridge()
        self.last_img = None
        self.last_stamp = None

        self.create_subscription(Image, '/camera/image_raw', self.img_callback, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(PoseArray, '/yolo/dets', 1)
        self.vis_pub = self.create_publisher(Image, '/yolo/debug_image', 1)

        # Detector (예행연습: PyTorch. 나중에 INT8 넣을 때 predict 자리만 교체)
        try:
            self.model = YOLO(WEIGHT_PATH)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        self.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        device = next(self.model.model.parameters()).device
        self.get_logger().info(f"YOLO model on: {device}")

        # Detection params (YOLO 스타일 유지)
        self.imgsz = 640
        self.conf_thres = 0.25
        self.class_whitelist = None  # 예: [0,1] 만 통과. None이면 전체.

        # BYTETracker 초기화 (yaml 키 다양성 대비 매핑)
        try:
            with open(TRACKER_PATH, 'r') as f:
                y = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"Failed to read bytetrack.yaml: {e}")
            y = {}

        self.fps = 20  # timer 0.05s → 20 FPS
        # Ultralytics가 기대하는 args 네이밍으로 폴백 매핑
        from types import SimpleNamespace
        args = SimpleNamespace(
            # 핵심 임계값들
            track_high_thresh = y.get('track_high_thresh', y.get('track_thresh', y.get('high_thresh', 0.25))),
            track_low_thresh  = y.get('track_low_thresh',  y.get('low_thresh',  y.get('conf_thres', 0.1))),
            new_track_thresh  = y.get('new_track_thresh',  y.get('new_thresh',  0.5)),

            # 매칭/버퍼
            match_thresh      = y.get('match_thresh', 0.8),
            track_buffer      = y.get('track_buffer', 30),

            # 점수 융합 (Ultralytics 일부 버전에서 필수)
            fuse_score        = y.get('fuse_score', True),
            # fuse_weight       = y.get('fuse_weight', 0.98),

            # # 기타(버전별로 참조 가능)
            # min_box_area      = y.get('min_box_area', 0),
            # aspect_ratio_thresh = y.get('aspect_ratio_thresh', 20.0),
            # mot20             = y.get('mot20', False),

            # # 근접/보조 임계값 (일부 구현에서 사용)
            # proximity_thresh  = y.get('proximity_thresh', 0.5),
        )
        self.bt = BYTETracker(args, frame_rate=self.fps)
        self.get_logger().info(f"BYTETracker args: {args}")

        # Warmup (optional)
        try:
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            _ = self.model.predict(dummy, imgsz=self.imgsz, conf=self.conf_thres, verbose=False)
        except Exception as e:
            self.get_logger().warn(f"YOLO warmup failed: {e}")

        # Main loop
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info("YOLOv10Node started. Subscribed to /camera/image_raw")

    # ------------------------- Callbacks -------------------------
    def img_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_img = img
            self.last_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    # ------------------------- Main loop -------------------------
    def loop(self):
        if self.last_img is None:
            return

        img = self.last_img
        stamp = self.last_stamp
        H, W = img.shape[:2]
        drawn = img.copy()

        # 1) Detection
        try:
            pred = self.model.predict(img, imgsz=self.imgsz, conf=self.conf_thres, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO predict() failed: {e}")
            return

        if len(pred) == 0 or not hasattr(pred[0], 'boxes') or pred[0].boxes is None:
            self.publish_results([], [], {}, stamp, drawn)
            return

        # Boxes 객체를 그대로 사용 (Ultralytics BYTETracker가 기대)
        boxes = pred[0].boxes.cpu()

        # (선택) 클래스 화이트리스트 마스킹
        if self.class_whitelist is not None and len(boxes) > 0:
            mask = np.isin(boxes.cls.numpy().astype(np.int64), self.class_whitelist)
            boxes = boxes[mask]

        if len(boxes) == 0:
            self.publish_results([], [], {}, stamp, drawn)
            return

        # 2) BYTETracker 업데이트 (img 전달 권장)
        try:
            _ = self.bt.update(boxes, img=img)
        except Exception as e:
            self.get_logger().error(f"BYTETracker update failed: {e}")
            return

        # 3) 현재 활성 트랙에서 결과 꺼내기 (STrack API)
        tracks_xyxy, tracks_tid, tid2cls, tid2score = [], [], {}, {}
        for t in self.bt.tracked_stracks:
            if not getattr(t, "is_activated", False):
                continue
            x1, y1, x2, y2 = t.xyxy.astype(float)
            tracks_xyxy.append([x1, y1, x2, y2])
            tid = int(getattr(t, "track_id", -1))
            tracks_tid.append(tid)
            tid2score[tid] = float(getattr(t, "score", 1.0) or 1.0)
            cobj = getattr(t, "cls", None)
            # cls가 텐서면 숫자로 캐스팅
            if hasattr(cobj, "item"):
                cobj = cobj.item()
            tid2cls[tid] = int(cobj) if cobj is not None else -1

        # 4) 퍼블리시 + 디버그 드로잉
        self.publish_results(tracks_xyxy, tracks_tid, tid2cls, stamp, drawn, tid2score)

    # ------------------------- Helpers -------------------------
    def publish_results(self, tracks_xyxy, tracks_tid, tid2cls, stamp, drawn, tid2score=None):
        H, W = drawn.shape[:2]
        pa = PoseArray()
        pa.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        pa.header.frame_id = 'camera'

        for (x1, y1, x2, y2), tid in zip(tracks_xyxy, tracks_tid):
            # clamp
            x1 = float(max(0.0, min(W - 1.0, x1))); y1 = float(max(0.0, min(H - 1.0, y1)))
            x2 = float(max(0.0, min(W - 1.0, x2))); y2 = float(max(0.0, min(H - 1.0, y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            c   = float(tid2score.get(tid, 1.0)) if tid2score is not None else 1.0
            cid = int(tid2cls.get(tid, -1))

            # Pose 인코딩 규칙: position.z=track_id, orientation.z=conf, orientation.w=class_id
            p = Pose()
            p.position.x = x1; p.position.y = y1; p.position.z = float(tid)
            p.orientation.x = x2; p.orientation.y = y2
            p.orientation.z = c;  p.orientation.w = float(cid)
            pa.poses.append(p)

            # draw
            x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(drawn, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
            name = self.model.names.get(cid, f'cls{cid}') if cid >= 0 else 'track'
            tag = f'{name}#{tid}' if tid >= 0 else f'{name}'
            label = f'{tag} {c:.2f}'
            cv2.putText(drawn, label, (x1i, max(0, y1i - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        self.det_pub.publish(pa)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(drawn, encoding='bgr8')
            dbg_msg.header.stamp = pa.header.stamp
            dbg_msg.header.frame_id = 'camera'
            self.vis_pub.publish(dbg_msg)
        except Exception as e:
            self.get_logger().warn(f"debug image publish failed: {e}")


def main():
    rclpy.init()
    node = YOLOv10Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
