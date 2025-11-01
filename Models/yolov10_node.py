#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseArray, Pose
from ultralytics import YOLO

# 의존성:
#   pip install ultralytics==8.* torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
# 준비물:
#   yolov10m.pt (실행 스크립트와 동일 경로 or 절대경로 지정)
#   (선택) ByteTrack/BoT-SORT YAML: bytetrack.yaml 또는 botsort.yaml
#        - rosparam ~tracker_cfg 로 경로 지정 가능 (미지정 시 기본 bytetrack.yaml)

class YOLOv10Node:
    def __init__(self):
        # ROS I/O
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.det_pub = rospy.Publisher("/yolo/dets", PoseArray, queue_size=1)
        self.vis_pub = rospy.Publisher("/yolo/debug_image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.last_img = None
        self.last_stamp = None

        # 금지구역 설정 (비어있으면 기능 꺼짐)
        # 예) rosparam:  rosparam set /yolov10_node/ban_rects "[[480,0,640,200],[0,380,640,480]]"
        self.ban_polys = [
            np.array([[20,480],[160,390],[480,390],[620,480]], np.int32)
        ]
        self.ban_mode  = rospy.get_param("~ban_mode", "mask")  # "mask" or "filter"

        # 모델 로드
        self.model = YOLO("yolov10s.pt")
        if torch.cuda.is_available():
            self.model.to("cuda:0")
        else:
            self.model.to("cpu")
        device = next(self.model.model.parameters()).device
        rospy.loginfo("YOLO model loaded on device: %s" % str(device))

        # 트래커 YAML (rosparam으로 바꿀 수 있음)
        default_yaml = rospy.get_param("~tracker_cfg", "bytetrack.yaml")
        if os.path.isabs(default_yaml) and not os.path.exists(default_yaml):
            rospy.logwarn("Tracker YAML not found at %s, falling back to 'bytetrack.yaml'" % default_yaml)
            default_yaml = "bytetrack.yaml"
        self.tracker_cfg = default_yaml

        # (선택) 워밍업
        try:
            dummy = np.zeros((512, 512, 3), dtype=np.uint8)
            _ = self.model.predict(dummy, imgsz=512, conf=0.25, verbose=False)
        except Exception as e:
            rospy.logwarn("YOLO warmup failed: %s" % str(e))

        # 관심 클래스 필터 (COCO): person, bicycle, car, motorbike, bus, truck, cat, dog, horse, sheep, cow
        self.allowed_ids = [0, 1, 2, 3, 5, 7, 9, 15, 16, 17, 18, 19]

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    # ------------------------ Utils ------------------------
    @staticmethod
    def _center_in_any_poly(x1,y1,x2,y2, polys):
        cx = 0.5*(x1+x2)
        cy = 0.5*(y1+y2)
        for poly in polys:
            if cv2.pointPolygonTest(poly, (cx,cy), False) >= 0:
                return True
        return False

    @staticmethod
    def _apply_ban_mask(img, polys):
        out = img.copy()
        for poly in polys:
            cv2.fillPoly(out, [poly], (0,0,0))  # 검은색으로 영역 덮기
        return out

    # ------------------------ ROS callbacks ------------------------
    def img_callback(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_img = img
            self.last_stamp = msg.header.stamp
        except Exception as e:
            rospy.logwarn("cv_bridge compressed error: %s" % str(e))

    # ------------------------ Main loop ------------------------
    def loop(self):
        if self.last_img is None:
            return

        img = self.last_img
        stamp = self.last_stamp
        H, W = img.shape[:2]
        drawn = img.copy()

        # 금지구역 모드에 따라 입력 이미지 선택
        feed_img = img
        if self.ban_polys and self.ban_mode.lower() == "mask":
            feed_img = self._apply_ban_mask(img, self.ban_polys)

        # ── 내장 트래커로 추적 (persist=True 중요) ──
        try:
            results = self.model.track(feed_img, persist=True, tracker=self.tracker_cfg)
        except Exception as e:
            rospy.logerr("YOLO track() failed: %s" % str(e))
            return

        # PoseArray 준비
        pa = PoseArray()
        pa.header.stamp = stamp if stamp is not None else rospy.Time.now()
        pa.header.frame_id = "camera"

        # 결과 파싱
        if len(results) > 0 and hasattr(results[0], "boxes") and results[0].boxes is not None:
            r0 = results[0]
            boxes = r0.boxes
            xyxy = boxes.xyxy.detach().cpu().numpy() if boxes.xyxy is not None else np.zeros((0, 4))
            conf = boxes.conf.detach().cpu().numpy()  if boxes.conf is not None else np.zeros((0,))
            clss = boxes.cls.detach().cpu().numpy()   if boxes.cls  is not None else np.zeros((0,))
            ids  = boxes.id
            ids  = ids.int().cpu().numpy() if ids is not None else -np.ones((len(conf),), dtype=np.int32)

            for i in range(len(conf)):
                x1, y1, x2, y2 = xyxy[i]
                c   = float(conf[i])
                cid = int(clss[i])
                tid = int(ids[i])

                # 관심 클래스 필터를 쓰려면 주석 해제
                if cid not in self.allowed_ids:
                    continue

                # 금지구역 후처리 필터 모드: bbox 중심이 금지 사각형 안이면 버림
                if self.ban_polys and self.ban_mode.lower() == "filter":
                    if self._center_in_any_poly(x1, y1, x2, y2, self.ban_polys):
                        continue

                # 경계 클램프
                x1 = max(0, min(W - 1, x1)); y1 = max(0, min(H - 1, y1))
                x2 = max(0, min(W - 1, x2)); y2 = max(0, min(H - 1, y2))
                if x2 <= x1 or y2 <= y1:
                    continue

                # Pose 인코딩 규칙 유지 (w=class_id, z=conf)
                p = Pose()
                p.position.x = float(x1); p.position.y = float(y1)
                p.position.z = float(tid)            # track_id
                p.orientation.x = float(x2); p.orientation.y = float(y2)
                p.orientation.z = float(c)           # conf
                p.orientation.w = float(cid)         # class_id
                pa.poses.append(p)

                # 디버그 드로잉
                x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(drawn, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                name = self.model.names.get(cid, f"cls{cid}")
                tag  = f"{name}#{tid}" if tid >= 0 else f"{name}"
                label = f"{tag} {c:.2f}"
                cv2.putText(drawn, label, (x1i, max(0, y1i - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                rospy.loginfo("Track %s updated: cid=%d, conf=%.2f",
                              (str(tid) if tid >= 0 else "NA"), cid, c)

        # 디버그: 마스크 박스 시각화(원본 화면 위 그리기)
        for poly in self.ban_polys:
            cv2.polylines(drawn, [poly], isClosed=True, color=(0,0,255), thickness=2)
            cv2.putText(drawn, "BAN", tuple(poly[1]),  # 적당한 꼭짓점 근처에
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2, cv2.LINE_AA)

        # 퍼블리시
        self.det_pub.publish(pa)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(drawn, encoding="bgr8")
            dbg_msg.header.stamp = pa.header.stamp
            dbg_msg.header.frame_id = "camera"
            self.vis_pub.publish(dbg_msg)
        except Exception as e:
            rospy.logwarn("debug image publish failed: %s" % str(e))


if __name__ == "__main__":
    rospy.init_node("yolov10_node")
    YOLOv10Node()
    rospy.spin()
