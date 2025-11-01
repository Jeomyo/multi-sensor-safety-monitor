#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseArray, Pose
from numpy.linalg import inv
from std_msgs.msg import Int32
from morai_msgs.msg import EgoVehicleStatus

# === 카메라/라이다 파라미터 (기존 유지) ===
parameters_cam = {
    "WIDTH": 640, "HEIGHT": 480, "FOV": 100,
    "X": 1.8, "Y": 0, "Z": 1.5,
    "YAW": 0, "PITCH": 0.0, "ROLL": 0
}
parameters_lidar = {
    "X": 3.6, "Y": 0, "Z": 0.8,
    "YAW": 0, "PITCH": 0, "ROLL": 0
}

def getRotMat(RPY):
    cosR, cosP, cosY = math.cos(RPY[0]), math.cos(RPY[1]), math.cos(RPY[2])
    sinR, sinP, sinY = math.sin(RPY[0]), math.sin(RPY[1]), math.sin(RPY[2])
    rotRoll  = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotPitch = np.array([cosP,0,sinP, 0,1,0, -sinP,0,cosP]).reshape(3,3)
    rotYaw   = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    return rotYaw @ rotPitch @ rotRoll

def getTransformMat(params_cam, params_lidar):
    lidarPosition = np.array([params_lidar[i] for i in ["X","Y","Z"]])
    camPosition   = np.array([params_cam[i]   for i in ["X","Y","Z"]])
    lidarRPY      = np.array([params_lidar[i] for i in ["ROLL","PITCH","YAW"]])
    camRPY        = np.array([params_cam[i]   for i in ["ROLL","PITCH","YAW"]])
    # 차량축→카메라축 보정(-90deg roll, -90deg yaw)
    camRPY = camRPY + np.array([-90*math.pi/180, 0, -90*math.pi/180])

    camRot = getRotMat(camRPY)
    Tr_cam_to_vehicle = np.r_[np.c_[camRot, camPosition.reshape(3,1)], [[0,0,0,1]]]

    lidarRot = getRotMat(lidarRPY)
    Tr_lidar_to_vehicle = np.r_[np.c_[lidarRot, lidarPosition.reshape(3,1)], [[0,0,0,1]]]

    invTr = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = (invTr @ Tr_lidar_to_vehicle).round(6)
    rospy.loginfo("Tr_lidar_to_cam:\n%s", Tr_lidar_to_cam)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    fx = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    cx = params_cam["WIDTH"]/2.0
    cy = params_cam["HEIGHT"]/2.0
    K = np.array([[fx, 0,  cx],
                  [0,  fx, cy],
                  [0,  0,   1]], dtype=np.float32)
    rospy.loginfo("Camera K:\n%s", K)
    return K

class ROIClusterStopNode:
    """
    입력:
      - /clusters (PoseArray: 1클러스터=9 Pose)
      - /yolo/dets (PoseArray: x1,y1 / x2,y2 / class_id=orientation.w / track_id=position.z)
      - /image_jpeg/compressed (디버그용 이미지)
    출력:
      - /cluster_centers_roi (PoseArray: 게이팅된 점, 로컬좌표 그대로, orientation.w=track_id)
      - /target_speed (Int32: 0=STOP, 1=SLOW, 2=NORMAL)
    """
    def __init__(self):
        # --- Params ---
        self.shrink = rospy.get_param("~shrink", 0.0)
        self.allowed_ids = set(rospy.get_param("~allowed_ids", [0, 1, 2, 3, 5, 7, 15, 16, 17, 18, 19]))
        self.slow_thr_m = rospy.get_param("~slow_thr_m", 15)
        self.stop_dist_thr = rospy.get_param("~stop_dist_thr", 8.5)  # m

        # 대표점 기반 같은 물체 판단 허용 거리 (r_rep와의 차)
        self.rep_tol_m = rospy.get_param("~rep_tol_m", 5.0)

        # 정지 후 정적 판정 파라미터
        self.static_delay_s = rospy.get_param("~static_delay_s", 0.1)    # 정지 후 관찰 시작까지 딜레이
        self.static_window_s = rospy.get_param("~static_window_s", 0.3)  # 관찰 윈도 길이
        self.static_tol_m = rospy.get_param("~static_tol_m", 0.01)       # min/max 변동 허용(m)

        # 정적 트랙 자동 해제 파라미터
        self.static_forget_s = rospy.get_param("~static_forget_s", 3.0)  # 안 보이면 몇 초 후 잊기

        self.person_cids  = set(rospy.get_param("~person_cids",  [0,15,16,17,18,19]))      # person
        self.vehicle_cids = set(rospy.get_param("~vehicle_cids", [1,2,3,5,7]))  # car,bus,truck
        self.person_static_check_m  = float(rospy.get_param("~person_static_check_m", 10.0))
        self.vehicle_static_check_m = float(rospy.get_param("~vehicle_static_check_m", 30.0))

        # [NEW] 사람 전용 'static-1' 단계 파라미터
        self.static1_recheck_m = float(rospy.get_param("~static1_recheck_m", 4.5))  # 4.5m 미만 재정지 재판정 트리거
        self.static1_forget_s  = float(rospy.get_param("~static1_forget_s", 3.0))

        # --- Subs/Pubs ---
        self.img_sub   = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_cb, queue_size=1)
        self.dets_sub  = rospy.Subscriber("/yolo/dets", PoseArray, self.dets_cb, queue_size=1)
        self.clust_sub = rospy.Subscriber("/clusters", PoseArray, self.clusters_cb, queue_size=1)
        self.status_sub = rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_cb, queue_size=1)

        self.roi_pub   = rospy.Publisher("/cluster_centers_roi", PoseArray, queue_size=1)
        self.debug_pub = rospy.Publisher("/yolo/roi_debug_image", Image, queue_size=1)
        self.mode_pub = rospy.Publisher("/target_speed", Int32, queue_size=1)

        # --- State ---
        self.bridge = CvBridge()
        self.img = None
        self.last_dets = None
        self.last_clusters = None

        # 정지/정적 판정 상태
        self.stop_state = False
        self.stop_first_ts = None
        self.obs_start_ts = None
        self.min_dist_hist = []   # list of (t, dist)

        self.ego_speed_mps = 0.0
        self.speed_zero_since = None
        self.SPEED_ZERO_THR = 0.10   # m/s 이하를 '정지'로 간주 (파라미터 추가 없이 고정)
        self.SPEED_HOLD_S   = 0.30   # 이 시간 이상 연속 정지일 때 관찰 시작

        # 정적 트랙 관리
        self.static_tids = set()
        self.static_last_seen = {}

        # [NEW] static-1 (사람 전용) 상태
        self.static1_tids = set()
        self.static1_last_seen = {}

        # --- Calib ---
        self.K  = getCameraMat(parameters_cam)
        self.Tr = getTransformMat(parameters_cam, parameters_lidar)
        self.W  = parameters_cam["WIDTH"]; self.H = parameters_cam["HEIGHT"]

        self.loop()

    def img_cb(self, msg): self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    def dets_cb(self, msg): self.last_dets = msg
    def clusters_cb(self, msg): self.last_clusters = msg
    def status_cb(self, msg): self.ego_speed_mps = float(msg.velocity.x)

    def _project_xyz_to_uv(self, xyz_cam):
        uvw = self.K.dot(xyz_cam)
        return uvw[:2, :] / uvw[2:3, :]

    def loop(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.last_clusters is None or self.last_dets is None or self.img is None:
                rate.sleep(); continue

            clusters = self.last_clusters
            dets     = self.last_dets

            if len(clusters.poses) == 0:
                self.roi_pub.publish(PoseArray(header=clusters.header))
                self.mode_pub.publish(Int32(data=2))        # NORMAL 모드로 퍼블리시
                rate.sleep(); continue

            # --- 모든 클러스터 점 가져오기 ---
            poses_all = clusters.poses

            # LiDAR local -> Cam
            xyz_l = np.array([[p.position.x, p.position.y, p.position.z] for p in poses_all], dtype=np.float32).T
            ones  = np.ones((1, xyz_l.shape[1]), dtype=np.float32)
            xyz1  = np.vstack([xyz_l, ones])
            xyz_cam_h = self.Tr.dot(xyz1)
            xyz_cam   = xyz_cam_h[:3, :]
            z = xyz_cam[2, :]
            valid = z > 0
            if not np.any(valid):
                self.roi_pub.publish(PoseArray(header=clusters.header))
                self.mode_pub.publish(Int32(data=2)) 
                rate.sleep(); continue

            xyz_cam = xyz_cam[:, valid]
            keep_idx = np.nonzero(valid)[0]

            # Project
            uv = self._project_xyz_to_uv(xyz_cam)
            u, v = uv[0, :], uv[1, :]
            in_img = (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H)
            u, v = u[in_img], v[in_img]
            keep_idx = keep_idx[in_img]

            if keep_idx.size == 0:
                self.roi_pub.publish(PoseArray(header=clusters.header))
                self.mode_pub.publish(Int32(data=2)) 
                rate.sleep(); continue

            draw = self.img.copy()
            for ui, vi in zip(u.astype(int), v.astype(int)):
                cv2.circle(draw, (ui, vi), 3, (0, 0, 255), -1)

            # --- YOLO ROI 준비 ---
            dets_list = []
            if len(dets.poses) > 0:
                for p in dets.poses:
                    x1, y1 = float(p.position.x), float(p.position.y)
                    x2, y2 = float(p.orientation.x), float(p.orientation.y)
                    cid    = int(round(p.orientation.w))
                    tid    = int(round(p.position.z))
                    if self.allowed_ids and (cid not in self.allowed_ids): continue
                    if self.shrink > 0.0:
                        w, h = x2-x1, y2-y1
                        x1s = x1+self.shrink*w; y1s = y1+self.shrink*h
                        x2s = x2-self.shrink*w; y2s = y2-self.shrink*h
                    else:
                        x1s, y1s, x2s, y2s = x1,y1,x2,y2
                    cx, cy = 0.5*(x1s+x2s), 0.5*(y1s+y2s)
                    dets_list.append((x1s, y1s, x2s, y2s, tid, cx, cy, cid))
                    cv2.rectangle(draw, (int(x1s),int(y1s)), (int(x2s),int(y2s)), (0,255,0), 2)

            assigned_tid = np.full(keep_idx.size, -1, dtype=np.int32)
            inside_any   = np.zeros(keep_idx.size, dtype=bool)

            if dets_list:
                for i, (ui, vi) in enumerate(zip(u, v)):
                    best_d = 1e18
                    for (x1s,y1s,x2s,y2s,tid,cx,cy,cid) in dets_list:
                        if (ui>=x1s) and (ui<=x2s) and (vi>=y1s) and (vi<=y2s):
                            inside_any[i] = True
                            d = (ui-cx)**2 + (vi-cy)**2
                            if d < best_d:
                                best_d = d; assigned_tid[i] = tid

            for (ui, vi, ok) in zip(u.astype(int), v.astype(int), inside_any):
                cv2.circle(draw, (ui, vi), 4, (0,255,0) if ok else (0,0,255), -1)

            # --- bbox 중심 기반 대표점 선택 & 클러스터 정제 ---
            tid_center = {}
            for (x1s, y1s, x2s, y2s, tid, cx, cy, cid) in dets_list:
                tid_center[tid] = (cx, cy)

            unique_tids = np.unique(assigned_tid[(assigned_tid != -1) & inside_any])
            rep_indices = {}   # tid -> 대표점 j (u/v/keep_idx 인덱스)
            rep_ranges  = {}   # tid -> r_rep (m)
            tid_cid = {}
            for (x1s,y1s,x2s,y2s,tid,cx,cy,cid) in dets_list:
                tid_cid[int(tid)] = int(cid)

            for tid in unique_tids:
                cx, cy = tid_center.get(tid, (None, None))
                if cx is None:
                    continue
                mask = (assigned_tid == tid) & inside_any
                idxs = np.where(mask)[0]
                if idxs.size == 0:
                    continue

                # bbox 중심에 가장 가까운 포인트(픽셀 거리 최소)를 대표점으로
                d2 = (u[idxs] - cx)**2 + (v[idxs] - cy)**2
                j_rep = idxs[np.argmin(d2)]

                # 대표점의 라이다 범위 r_rep 계산
                p_rep = poses_all[int(keep_idx[j_rep])]
                r_rep = math.hypot(p_rep.position.x, p_rep.position.y)

                rep_indices[tid] = j_rep
                rep_ranges[tid]  = r_rep

                # 대표점 시각화(노란색)
                cv2.circle(draw, (int(u[j_rep]), int(v[j_rep])), 6, (0,255,255), 2)

                # 같은 tid의 다른 점들 중 |r - r_rep| > rep_tol_m 은 제외
                for j in idxs:
                    if j == j_rep:
                        continue
                    p_j = poses_all[int(keep_idx[j])]
                    r_j = math.hypot(p_j.position.x, p_j.position.y)
                    if abs(r_j - r_rep) > self.rep_tol_m:
                        inside_any[j] = False

            # --- 퍼블리시 + 정지 판단 ---
            out = PoseArray()
            out.header = clusters.header
            min_dist = float("inf")

            # 현재 프레임 시각
            now = rospy.Time.now().to_sec()

            # 이번 프레임에 보인 tid들의 last_seen 갱신을 위해 미리 집계
            seen_tids = np.unique(assigned_tid[(assigned_tid != -1) & inside_any])

            # class & 거리 기반 정적 판정 후보
            candidate_tids = set()
            for t in seen_tids:
                t = int(t)
                cid = tid_cid.get(t, None)
                r   = rep_ranges.get(t, None)  # 대표점 거리 r_rep
                if (cid is None) or (r is None):
                    continue
                if (cid in self.person_cids) and (r <= self.person_static_check_m):
                    candidate_tids.add(t)
                elif (cid in self.vehicle_cids) and (r <= self.vehicle_static_check_m):
                    candidate_tids.add(t)

            for j, idx in enumerate(keep_idx):
                p_src = poses_all[int(idx)]
                tid_j = int(assigned_tid[j]) if assigned_tid[j] != -1 else None

                # out(시각화/디버깅)에는 그대로 남김
                p_out = Pose()
                p_out.position = p_src.position
                p_out.orientation = p_src.orientation
                p_out.orientation.w = float(assigned_tid[j])
                if inside_any[j]:
                    out.poses.append(p_out)

                # min_dist 갱신은 inside_any & (정적 제외) + [NEW] static-1은 임계 이내일 때만 포함
                if not inside_any[j]:
                    continue
                if (tid_j is not None) and (tid_j in self.static_tids):
                    continue

                lx, ly = p_src.position.x, p_src.position.y
                dist = math.hypot(lx, ly)

                # [NEW] static-1(tid)이면 dist < static1_recheck_m 일 때만 min_dist 후보로 인정
                if (tid_j is not None) and (tid_j in self.static1_tids):
                    if not (dist < self.static1_recheck_m):
                        continue

                if dist < min_dist:
                    min_dist = dist

            self.roi_pub.publish(out)

            # 기본 stop 여부
            base_stop = (min_dist < self.stop_dist_thr)

            # 정지/정적 판정 로직
            if base_stop:
                if not self.stop_state:
                    # 정지 진입
                    self.stop_state = True
                    self.stop_first_ts = now
                    self.obs_start_ts = None
                    self.min_dist_hist = []

                # 관찰 시작(딜레이 후)
                if self.ego_speed_mps <= self.SPEED_ZERO_THR:
                    if self.speed_zero_since is None:
                        self.speed_zero_since = now
                else:
                    self.speed_zero_since = None
                    # 관찰 중이었다면 속도 조건 깨지는 즉시 관찰 리셋(오검 방지)
                    if self.obs_start_ts is not None:
                        self.obs_start_ts = None
                        self.min_dist_hist = []

                if (self.obs_start_ts is None) \
                and (now - self.stop_first_ts >= self.static_delay_s) \
                and (self.speed_zero_since is not None) \
                and ((now - self.speed_zero_since) >= self.SPEED_HOLD_S):
                    self.obs_start_ts = now
                    self.min_dist_hist = []

                # 관찰 중이면 히스토리 축적
                if self.obs_start_ts is not None:
                    self.min_dist_hist.append((now, min_dist))
                    # 윈도 밖 데이터 정리
                    while self.min_dist_hist and (now - self.min_dist_hist[0][0] > self.static_window_s):
                        self.min_dist_hist.pop(0)

                    # 윈도 충족 시 변동 폭 체크
                    if (now - self.obs_start_ts) >= self.static_window_s and len(self.min_dist_hist) >= 2:
                        ds = [d for (_, d) in self.min_dist_hist if np.isfinite(d)]
                        if ds:
                            delta = max(ds) - min(ds)   # 동/정적 공통 변동폭
                            if delta <= self.static_tol_m:
                                # 정적 객체로 판단 → 이번 프레임 stop 해제(재출발)
                                rospy.loginfo("[STATIC or STATIC-1] min_dist=%.2f, Δdist=%.3f, tids=%s",
                                              min_dist, delta, list(seen_tids))
                                base_stop = False
                                self.stop_state = False
                                self.stop_first_ts = None
                                self.obs_start_ts = None
                                self.min_dist_hist = []
                                # 이번 프레임 보인 tid들을 static-1(사람) 또는 static(차량)으로 등록/승격
                                for tid in seen_tids:
                                    tid = int(tid)
                                    if tid not in candidate_tids:
                                        continue
                                    cid = tid_cid.get(tid, None)
                                    if cid is None:
                                        continue
                                    if cid in self.person_cids:
                                        # 사람이면: 이미 static-1 이면 static으로 승격, 아니면 static-1로 신규 등록
                                        if tid in self.static1_tids:
                                            # 승격
                                            self.static1_tids.discard(tid)
                                            self.static1_last_seen.pop(tid, None)
                                            self.static_tids.add(tid)
                                            self.static_last_seen[tid] = now
                                            rospy.loginfo("[PROMOTE] person tid=%d: STATIC-1 → STATIC", tid)
                                        else:
                                            # 신규 static-1
                                            self.static1_tids.add(tid)
                                            self.static1_last_seen[tid] = now
                                            rospy.loginfo("[STATIC-1] person tid=%d registered", tid)
                                    elif cid in self.vehicle_cids:
                                        # 차량류는 기존대로 바로 static
                                        self.static_tids.add(tid)
                                        self.static_last_seen[tid] = now
                                        rospy.loginfo("[STATIC] vehicle tid=%d registered", tid)
                            else:
                                # 동적 로그
                                rospy.loginfo("[DYNAMIC] min_dist=%.2f, Δdist=%.3f (tol=%.3f)",
                                              min_dist, delta, self.static_tol_m)
            else:
                # 달리는 중이면 상태 초기화
                if self.stop_state:
                    self.stop_state = False
                self.stop_first_ts = None
                self.obs_start_ts = None
                self.speed_zero_since = None
                self.min_dist_hist = []

            # 프레임마다 보인 tid의 last_seen 갱신 (static, static-1 모두)
            for tid in seen_tids:
                tid = int(tid)
                self.static_last_seen[tid] = now if tid in self.static_tids else self.static_last_seen.get(tid, None)
                if tid in self.static1_tids:
                    self.static1_last_seen[tid] = now

            # 오래 안 보인 정적 tid들은 자동 해제
            if self.static_tids:
                forget = []
                for tid in list(self.static_tids):
                    ts = self.static_last_seen.get(tid, 0.0)
                    if now - ts > self.static_forget_s:
                        forget.append(tid)
                for tid in forget:
                    self.static_tids.discard(tid)
                    self.static_last_seen.pop(tid, None)

            # [NEW] 오래 안 보인 static-1 tid들도 자동 해제
            if self.static1_tids:
                forget1 = []
                for tid in list(self.static1_tids):
                    ts = self.static1_last_seen.get(tid, 0.0)
                    if now - ts > self.static1_forget_s:
                        forget1.append(tid)
                for tid in forget1:
                    self.static1_tids.discard(tid)
                    self.static1_last_seen.pop(tid, None)

            # 속도 모드 퍼블리시
            if base_stop:
                mode = 0  # STOP
            elif np.isfinite(min_dist) and (min_dist < self.slow_thr_m):
                mode = 1  # SLOW
            else:
                mode = 2  # NORMAL

            self.mode_pub.publish(Int32(data=int(mode)))

            # 정적/정적-1 시각화 보강
            try:
                if dets_list:
                    for (x1s, y1s, x2s, y2s, tid, cx, cy, cid) in dets_list:
                        itid = int(tid)
                        if itid in self.static_tids:
                            pt1 = (int(x1s), int(y1s))
                            pt2 = (int(x2s), int(y2s))
                            # 정적 트랙: 노랑색 박스
                            cv2.rectangle(draw, pt1, pt2, (255, 255, 0), 2)
                            label = f"ID {itid} STATIC"
                            text_org = (pt1[0], max(20, pt1[1] - 6))
                            cv2.putText(draw, label, text_org,
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
                        elif itid in self.static1_tids:
                            pt1 = (int(x1s), int(y1s))
                            pt2 = (int(x2s), int(y2s))
                            # [NEW] static-1: 하늘색(청록) 박스
                            cv2.rectangle(draw, pt1, pt2, (255, 200, 100), 2)
                            label = f"ID {itid} STATIC-1"
                            text_org = (pt1[0], max(20, pt1[1] - 6))
                            cv2.putText(draw, label, text_org,
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2, cv2.LINE_AA)
            except Exception as e:
                rospy.logwarn("static/static-1 label draw failed: %s", str(e))

            # 디버그 텍스트
            if min_dist < float("inf"):
                cv2.putText(draw, f"min_dist: {min_dist:.2f} m", (20,40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255), 2, cv2.LINE_AA)
            cv2.putText(draw, f"MODE: {mode} (0=STOP,1=SLOW,2=NORM)", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200,255,200), 2, cv2.LINE_AA)

            # 디버그 이미지
            try:
                dbg = self.bridge.cv2_to_imgmsg(draw, encoding="bgr8")
                dbg.header.stamp = rospy.Time.now()
                dbg.header.frame_id = "camera"
                self.debug_pub.publish(dbg)
            except Exception as e:
                rospy.logwarn("roi debug publish failed: %s", str(e))

            # cv2.imshow("ROI Debug", draw)
            # cv2.waitKey(1)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("cluster_roi_stop_node")
    ROIClusterStopNode()
