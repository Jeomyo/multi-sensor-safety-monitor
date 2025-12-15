#!/usr/bin/env python3
# ROS2 (rclpy) version

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose
from dx_msgs.msg import PersonGatedWithID, PersonGatedWithIDArray
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge
from numpy.linalg import inv


# =========================
# User parameters
# =========================
PERSON_CLASS_ID = 5          # 커스텀 모델 person 클래스 ID (예: 5: Person)
LIDAR_FRAME     = "laser"    # LiDAR 프레임 이름

# YOLO 클래스 ID 예시:
# 0: Hardhat
# 2: NO-Hardhat
# 4: NO-Safety Vest
# 5: Person
# 7: Safety Vest
HELMET_POS_IDS = {0}   # Hardhat
VEST_POS_IDS   = {7}   # Safety Vest

# IOU 스레시홀드 (Person bbox와 헬멧/조끼 bbox 매칭 기준)
IOU_THRESHOLD_HELMET = 0.01
IOU_THRESHOLD_VEST   = 0.2

# Temporal smoothing: 몇 프레임까지 miss를 허용할지
HELMET_MISS_MAX = 3   # ~0.15초 (timer 0.05s 기준)
VEST_MISS_MAX   = 3


parameters_cam = {
    "WIDTH": 640,
    "HEIGHT": 480,
    "FOV_H": 78.0,
    "FOV_V": 44.0,
    # Camera position in vehicle frame [m]
    "X": -0.2,
    "Y": -0.04,
    "Z": 1.2,
    # small misalignment about VEHICLE axes [rad]
    "YAW":  -0.05,
    "PITCH": 0.55,
    "ROLL":  0.0,
    # extra fine-tuning purely about camera X (optical) axis [deg]
    "tilt_about_cam_x_deg": 0.0,
}

parameters_lidar = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "YAW": math.pi,
    "PITCH": 0.0,
    "ROLL": math.pi,
}


# =========================
# Math helpers
# =========================
def getRotMat(RPY):
    roll, pitch, yaw = RPY[0], RPY[1], RPY[2]
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)

    rotRoll  = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=np.float64)
    rotPitch = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=np.float64)
    rotYaw   = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=np.float64)
    return rotYaw @ rotPitch @ rotRoll

def rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=np.float64)

def getTransformMat(params_cam, params_lidar):
    lidarPosition = np.array([params_lidar[i] for i in ["X","Y","Z"]], dtype=np.float64)
    camPosition   = np.array([params_cam[i]   for i in ["X","Y","Z"]], dtype=np.float64)

    lidarRPY = np.array([params_lidar[i] for i in ["ROLL","PITCH","YAW"]], dtype=np.float64)
    camRPY   = np.array([params_cam[i]   for i in ["ROLL","PITCH","YAW"]], dtype=np.float64)

    # camera optical -> vehicle (clean axis map)
    R_fix_camopt_to_vehicle = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]
    ], dtype=np.float64)

    R_misalignment_vehicle = getRotMat(camRPY)
    R_camopt_to_vehicle = R_misalignment_vehicle @ R_fix_camopt_to_vehicle

    tilt_deg = float(params_cam.get("tilt_about_cam_x_deg", 0.0))
    if abs(tilt_deg) > 1e-9:
        R_tilt_camX = rot_x(math.radians(tilt_deg))
        R_camopt_to_vehicle = R_camopt_to_vehicle @ R_tilt_camX.T

    Tr_cam_to_vehicle = np.eye(4, dtype=np.float64)
    Tr_cam_to_vehicle[:3,:3] = R_camopt_to_vehicle
    Tr_cam_to_vehicle[:3, 3] = camPosition

    R_lidar_to_vehicle = getRotMat(lidarRPY)
    Tr_lidar_to_vehicle = np.eye(4, dtype=np.float64)
    Tr_lidar_to_vehicle[:3,:3] = R_lidar_to_vehicle
    Tr_lidar_to_vehicle[:3, 3] = lidarPosition

    Tr_lidar_to_cam = inv(Tr_cam_to_vehicle) @ Tr_lidar_to_vehicle
    Tr_lidar_to_cam = Tr_lidar_to_cam.round(6)
    print("Tr_lidar_to_cam=\n", Tr_lidar_to_cam)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    W, H = params_cam["WIDTH"], params_cam["HEIGHT"]
    FOV_h, FOV_v = params_cam["FOV_H"], params_cam["FOV_V"]
    fx = W / (2 * math.tan(math.radians(FOV_h / 2)))
    fy = H / (2 * math.tan(math.radians(FOV_v / 2)))
    cx, cy = W / 2 , H / 2 + 10
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    print("K=\n", K)
    return K

def get_R_fix_camopt_to_vehicle(apply_yz_flip: bool = True) -> np.ndarray:
    R_base = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0],
    ], dtype=np.float64)

    if not apply_yz_flip:
        return R_base

    D = np.diag([1.0, -1.0, -1.0])
    R_adjusted = D @ R_base
    return R_adjusted


# ================ IOU helper ================
def bbox_iou(b1, b2):
    x1_min, y1_min, x1_max, y1_max = b1
    x2_min, y2_min, x2_max, y2_max = b2

    inter_xmin = max(x1_min, x2_min)
    inter_ymin = max(y1_min, y2_min)
    inter_xmax = min(x1_max, x2_max)
    inter_ymax = min(y1_max, y2_max)

    if inter_xmax <= inter_xmin or inter_ymax <= inter_ymin:
        return 0.0

    inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin)
    area1 = (x1_max - x1_min) * (y1_max - y1_min)
    area2 = (x2_max - x2_min) * (y2_max - y2_min)
    if area1 <= 0 or area2 <= 0:
        return 0.0

    return inter_area / float(area1 + area2 - inter_area)


# =========================
# PointCloud2 helper
# =========================
def make_cloud_xyz32(points_xyz, frame_id, stamp):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = int(points_xyz.shape[0])

    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12  # 3 * float32
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    msg.data = points_xyz.astype(np.float32).tobytes() if msg.width > 0 else b''
    return msg


# =========================
# Node
# =========================
class LiDARToCameraTransformNode(Node):
    def __init__(self, params_cam, params_lidar):
        super().__init__('ex_calib_ros2')

        # QoS
        qos_img  = qos_profile_sensor_data
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = QoSReliabilityPolicy.RELIABLE
        scan_qos.history     = QoSHistoryPolicy.KEEP_LAST
        scan_qos.durability  = QoSDurabilityPolicy.VOLATILE

        self.bridge = CvBridge()
        self.img = None
        self.pc_np = None
        self.width  = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        # person / helmet / vest bboxes (pixel 기반)
        # person_bboxes: list of (xmin, ymin, xmax, ymax, track_id)
        # helmet_bboxes: list of (xmin, ymin, xmax, ymax)
        # vest_bboxes:   list of (xmin, ymin, xmax, ymax)
        self.person_bboxes = []
        self.helmet_bboxes = []
        self.vest_bboxes   = []
        self.last_det_stamp = None  # /dx/detections의 stamp 보존

        # Temporal smoothing 상태
        self.helmet_state = {}  # track_id -> bool
        self.helmet_miss  = {}  # track_id -> int
        self.vest_state   = {}  # track_id -> bool
        self.vest_miss    = {}  # track_id -> int

        # Precompute
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat    = getCameraMat(params_cam)

        # Subscriptions
        self.create_subscription(Image,     '/dx/annotated_image', self.img_callback,  qos_img)
        self.create_subscription(LaserScan, '/scan',               self.scan_callback, qos_profile=scan_qos)
        self.create_subscription(Detection2DArray, '/dx/detections', self.det_callback, qos_profile=qos_profile_sensor_data)

        # Publishers (LiDAR frame)
        self.pub_person_poses   = self.create_publisher(PoseArray,          '/dx/person_gated',         1)
        self.pub_person_cloud   = self.create_publisher(PointCloud2,        '/dx/person_gated_cloud',   1)
        self.pub_person_with_id = self.create_publisher(PersonGatedWithIDArray, '/dx/person_gated_with_id', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("LiDAR->Camera projection + bbox median gating + PPE matching + temporal smoothing started.")

    # ------------ Callbacks -------------
    def img_callback(self, msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w = self.img.shape[:2]
            self.height, self.width = h, w
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")

    def scan_callback(self, msg: LaserScan):
        xs, ys = [], []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                xs.append(r * math.cos(angle))   # forward (vehicle +X)
                ys.append(r * math.sin(angle))   # left    (vehicle +Y)
            angle += msg.angle_increment

        if xs:
            xyz = np.stack(
                [np.array(xs, dtype=np.float32),
                 np.array(ys, dtype=np.float32),
                 np.zeros(len(xs), dtype=np.float32)],
                axis=1
            )
            self.pc_np = xyz  # (N,3) in LiDAR frame (2D LiDAR → z=0)
        else:
            self.pc_np = None

    def det_callback(self, msg: Detection2DArray):
        """/dx/detections에서 person/helmet/vest bbox들을 저장."""
        self.person_bboxes = []
        self.helmet_bboxes = []
        self.vest_bboxes   = []
        self.last_det_stamp = msg.header.stamp  # 타임스탬프 보존

        for det in msg.detections:
            if not det.results:
                continue

            cls_str = det.results[0].hypothesis.class_id
            try:
                cls_id = int(cls_str)
            except ValueError:
                continue

            bb = det.bbox
            cx = bb.center.position.x
            cy = bb.center.position.y
            w  = bb.size_x
            h  = bb.size_y

            xmin = int(cx - w/2)
            ymin = int(cy - h/2)
            xmax = int(cx + w/2)
            ymax = int(cy + h/2)

            xmin = max(0, xmin); ymin = max(0, ymin)
            xmax = min(self.width-1, xmax); ymax = min(self.height-1, ymax)
            if xmax <= xmin or ymax <= ymin:
                continue

            if cls_id == PERSON_CLASS_ID:
                track_id = -1
                if det.id:
                    try:
                        track_id = int(det.id)
                    except ValueError:
                        track_id = -1
                self.person_bboxes.append((xmin, ymin, xmax, ymax, track_id))

            if cls_id in HELMET_POS_IDS:
                self.helmet_bboxes.append((xmin, ymin, xmax, ymax))

            if cls_id in VEST_POS_IDS:
                self.vest_bboxes.append((xmin, ymin, xmax, ymax))

    # ------------ Projection -------------
    def transform_lidar_to_camera(self, pc_lidar_homo):
        cam_temp = self.TransformMat @ pc_lidar_homo  # (4,N)
        return cam_temp[:3, :]

    def project_camera_to_image_with_indices(self, pc_camera):
        N = pc_camera.shape[1]
        idx_all = np.arange(N, dtype=np.int32)

        uvw = self.CameraMat @ pc_camera  # (3,N)
        valid_front = uvw[2, :] > 0.0
        if not np.any(valid_front):
            return None, None, None

        uvw = uvw[:, valid_front]
        pc_front = pc_camera[:, valid_front]
        idx_front = idx_all[valid_front]

        uvw /= uvw[2, :]
        u, v = uvw[0, :], uvw[1, :]
        in_img = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        if not np.any(in_img):
            return None, None, None

        uvw_vis    = uvw[:, in_img]
        pc_cam_vis = pc_front[:, in_img]
        idx_vis    = idx_front[in_img]
        return uvw_vis, pc_cam_vis, idx_vis

    # ------------ Temporal smoothing helper -------------
    def apply_temporal_smoothing(self, track_id, helmet_now, vest_now):
        """
        track_id별로 helmet / vest temporal smoothing 적용.
        track_id < 0 인 경우에는 smoothing 없이 현재 값 그대로 사용.
        """
        if track_id is None or track_id < 0:
            return helmet_now, vest_now

        tid = int(track_id)

        # ---- Helmet ----
        prev_h = self.helmet_state.get(tid, False)
        miss_h = self.helmet_miss.get(tid, 0)

        if helmet_now:
            helmet_flag = True
            self.helmet_state[tid] = True
            self.helmet_miss[tid]  = 0
        else:
            if prev_h:
                miss_h += 1
                if miss_h >= HELMET_MISS_MAX:
                    helmet_flag = False
                    self.helmet_state[tid] = False
                else:
                    helmet_flag = True
                    self.helmet_state[tid] = True
                self.helmet_miss[tid] = miss_h
            else:
                helmet_flag = False
                self.helmet_state[tid] = False
                self.helmet_miss[tid]  = miss_h

        # ---- Vest ----
        prev_v = self.vest_state.get(tid, False)
        miss_v = self.vest_miss.get(tid, 0)

        if vest_now:
            vest_flag = True
            self.vest_state[tid] = True
            self.vest_miss[tid]  = 0
        else:
            if prev_v:
                miss_v += 1
                if miss_v >= VEST_MISS_MAX:
                    vest_flag = False
                    self.vest_state[tid] = False
                else:
                    vest_flag = True
                    self.vest_state[tid] = True
                self.vest_miss[tid] = miss_v
            else:
                vest_flag = False
                self.vest_state[tid] = False
                self.vest_miss[tid]  = miss_v

        return helmet_flag, vest_flag

    # ------------ Timer loop -------------
    def timer_callback(self):
        if self.img is None or self.pc_np is None:
            return

        xyz = self.pc_np  # (N,3) LiDAR frame
        pc_h = np.concatenate([xyz, np.ones((xyz.shape[0], 1), dtype=np.float32)], axis=1).T  # (4,N)
        pc_cam = self.transform_lidar_to_camera(pc_h)  # (3,N)

        ret = self.project_camera_to_image_with_indices(pc_cam)
        if ret[0] is None:
            return
        uvw, pc_cam_vis, idx_vis = ret
        u = uvw[0, :]; v = uvw[1, :]

        vis = self.img.copy()
        for x, y in zip(u.astype(int), v.astype(int)):
            if 0 <= x < self.width and 0 <= y < self.height:
                cv2.circle(vis, (x, y), 1, (0, 255, 0), -1)

        stamp = self.last_det_stamp if self.last_det_stamp is not None else self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = LIDAR_FRAME

        persons_msg = PersonGatedWithIDArray()
        persons_msg.header.stamp = stamp
        persons_msg.header.frame_id = LIDAR_FRAME

        gated_points_lidar = []

        for (xmin, ymin, xmax, ymax, track_id) in self.person_bboxes:
            person_box = (xmin, ymin, xmax, ymax)

            inside = (u >= xmin) & (u <= xmax) & (v >= ymin) & (v <= ymax)
            if not np.any(inside):
                continue

            idx_lidar = idx_vis[inside]
            pts_lidar = self.pc_np[idx_lidar, :]
            gated_points_lidar.append(pts_lidar)

            xL, yL, zL = (
                float(np.median(pts_lidar[:, 0])),
                float(np.median(pts_lidar[:, 1])),
                float(np.median(pts_lidar[:, 2]))
            )

            dist_m = math.sqrt(xL**2 + yL**2 + zL**2)

            p = Pose()
            p.position.x = xL
            p.position.y = yL
            p.position.z = zL
            pose_array.poses.append(p)

            # ---- 현재 프레임 기준 helmet_now / vest_now 계산 ----
            pcx = 0.5 * (xmin + xmax)
            pcy = 0.5 * (ymin + ymax)
            p_h = (ymax - ymin)

            band_top    = ymin               # 머리 ~ 상체 윗부분까지
            band_bottom = pcy                # 사람 중심까지 (상단 ~ 중앙)

            helmet_now = False
            best_h_box = None
            for hb in self.helmet_bboxes:
                hx1, hy1, hx2, hy2 = hb
                hcx = 0.5 * (hx1 + hx2)
                hcy = 0.5 * (hy1 + hy2)

                if (xmin <= hcx <= xmax) and (band_top <= hcy <= band_bottom):
                    helmet_now = True
                    best_h_box = hb
                    break

            best_iou_v = 0.0
            best_v_box = None
            for vb in self.vest_bboxes:
                iou_val = bbox_iou(person_box, vb)
                if iou_val > best_iou_v:
                    best_iou_v = iou_val
                    best_v_box = vb
            vest_now = (best_iou_v >= IOU_THRESHOLD_VEST)

            # ---- Temporal smoothing 적용 ----
            helmet_flag, vest_flag = self.apply_temporal_smoothing(track_id, helmet_now, vest_now)
            safe_flag = helmet_flag and vest_flag

            person = PersonGatedWithID()
            person.header.stamp = stamp
            person.header.frame_id = LIDAR_FRAME
            person.track_id = int(track_id) if track_id is not None else -1
            person.pose = p
            person.helmet = helmet_flag
            person.vest   = vest_flag
            person.safe   = safe_flag
            persons_msg.persons.append(person)

            # ---- 시각화: LiDAR median → Camera 변환 후 이미지에 점 + 텍스트 ----
            ptL_h = np.array([[xL, yL, zL, 1.0]], dtype=np.float64).T
            ptC   = self.transform_lidar_to_camera(ptL_h)
            uvC   = self.CameraMat @ ptC
            if uvC[2, 0] > 0:
                uvC /= uvC[2, 0]
                u_m, v_m = int(uvC[0, 0]), int(uvC[1, 0])
                if 0 <= u_m < self.width and 0 <= v_m < self.height:
                    color = (0, 0, 255) if not safe_flag else (255, 0, 0)
                    cv2.circle(vis, (u_m, v_m), 4, color, -1)

                    label = f"{dist_m:.1f}m  H:{int(helmet_flag)}  V:{int(vest_flag)}"
                    cv2.putText(vis, label, (u_m + 8, v_m - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)

            # ---- bbox 그리기 ----
            # cv2.rectangle(vis, (xmin, ymin), (xmax, ymax), (0, 255, 255), 2)

            if helmet_now and best_h_box is not None:
                hx1, hy1, hx2, hy2 = best_h_box
                cv2.rectangle(vis, (hx1, hy1), (hx2, hy2), (0, 255, 0), 2)

            if vest_now and best_v_box is not None:
                vx1, vy1, vx2, vy2 = best_v_box
                cv2.rectangle(vis, (vx1, vy1), (vx2, vy2), (255, 255, 0), 2)

        self.pub_person_poses.publish(pose_array)
        self.pub_person_with_id.publish(persons_msg)

        if len(gated_points_lidar) > 0:
            all_pts_lidar = np.vstack(gated_points_lidar).astype(np.float32)
        else:
            all_pts_lidar = np.zeros((0,3), dtype=np.float32)
        cloud_msg = make_cloud_xyz32(all_pts_lidar, frame_id=LIDAR_FRAME, stamp=stamp)
        self.pub_person_cloud.publish(cloud_msg)

        cv2.imshow("Lidar->Camera Projection + Median Gating + PPE Temporal", vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LiDARToCameraTransformNode(parameters_cam, parameters_lidar)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()