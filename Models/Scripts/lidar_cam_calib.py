#!/usr/bin/env python3
# ROS2 (rclpy) version

import math
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from numpy.linalg import inv


# =========================
# User parameters
# =========================
parameters_cam = {
    "WIDTH": 1280,
    "HEIGHT": 720,
    "FOV_H": 78.0,
    "FOV_V": 44.0,
    # Camera position in vehicle frame [m]
    "X": -0.2,
    "Y": -0.04,
    "Z": 1.0,
    # Small misalignment of CAMERA (optical) frame w.r.t vehicle frame [rad]
    # (These are now clean: no axis-mixing; see R_camopt_to_vehicle below)
    "YAW":  0.0,   # around +Z_vehicle
    "PITCH": 0.4,    # around +Y_vehicle
    "ROLL":  0.0,    # around +X_vehicle

    # Optional: extra fine-tuning purely about camera X (optical) axis [deg]
    # (-) looks up (올려다보기), (+) looks down (내려다보기)
    "tilt_about_cam_x_deg": 0.0,   # e.g., -2.0 to slightly look up
}

parameters_lidar = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "YAW": 0.0,
    "PITCH": 0.0,
    "ROLL": 0.0,
}


# =========================
# Math helpers
# =========================
def getRotMat(RPY):
    """Return R = Rz(yaw) @ Ry(pitch) @ Rx(roll), using vehicle-axis convention."""
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
    """
    Build Tr_lidar_to_cam (camera 'optical' frame) with clean axis mapping.
    Vehicle frame: +X forward, +Y left, +Z up (REP-103)
    Camera optical: +Z forward, +X right, +Y down (OpenCV/REP-103)
    """

    # Positions
    lidarPosition = np.array([params_lidar[i] for i in ["X","Y","Z"]], dtype=np.float64)
    camPosition   = np.array([params_cam[i]   for i in ["X","Y","Z"]], dtype=np.float64)

    # Small misalignment angles defined about VEHICLE axes
    lidarRPY = np.array([params_lidar[i] for i in ["ROLL","PITCH","YAW"]], dtype=np.float64)
    camRPY   = np.array([params_cam[i]   for i in ["ROLL","PITCH","YAW"]], dtype=np.float64)

    # 1) Fixed axis mapping: camera 'optical' -> vehicle (constant)
    #    vehicle X = cam Z, vehicle Y = -cam X, vehicle Z = -cam Y
    R_fix_camopt_to_vehicle = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]
    ], dtype=np.float64)

    # 2) Apply your small misalignment about VEHICLE axes (clean, no mixing)
    #    If your camRPY are deltas around the vehicle axes, they LEFT-multiply.
    R_misalignment_vehicle = getRotMat(camRPY)

    # Total rotation: cam_optical -> vehicle
    R_camopt_to_vehicle = R_misalignment_vehicle @ R_fix_camopt_to_vehicle

    # (Optional) extra fine-tuning purely about camera X (optical) axis
    tilt_deg = float(params_cam.get("tilt_about_cam_x_deg", 0.0))
    if abs(tilt_deg) > 1e-9:
        R_tilt_camX = rot_x(math.radians(tilt_deg))
        # This is a rotation in CAMERA space, so we apply it on the CAMERA side:
        # Equivalent to pre-multiplying the cam->veh rotation.
        R_camopt_to_vehicle = R_camopt_to_vehicle @ R_tilt_camX.T  # (cam-side pre-rot)

    # Build homogeneous transforms
    Tr_cam_to_vehicle = np.eye(4, dtype=np.float64)
    Tr_cam_to_vehicle[:3,:3] = R_camopt_to_vehicle
    Tr_cam_to_vehicle[:3, 3] = camPosition

    R_lidar_to_vehicle = getRotMat(lidarRPY)
    Tr_lidar_to_vehicle = np.eye(4, dtype=np.float64)
    Tr_lidar_to_vehicle[:3,:3] = R_lidar_to_vehicle
    Tr_lidar_to_vehicle[:3, 3] = lidarPosition

    # We want lidar -> camera_optical
    Tr_lidar_to_cam = inv(Tr_cam_to_vehicle) @ Tr_lidar_to_vehicle

    Tr_lidar_to_cam = Tr_lidar_to_cam.round(6)
    print("Tr_lidar_to_cam=\n", Tr_lidar_to_cam)
    return Tr_lidar_to_cam


def getCameraMat(params_cam):
    W, H = params_cam["WIDTH"], params_cam["HEIGHT"]
    FOV_h, FOV_v = params_cam["FOV_H"], params_cam["FOV_V"]

    fx = W / (2 * math.tan(math.radians(FOV_h / 2)))
    fy = H / (2 * math.tan(math.radians(FOV_v / 2)))
    cx, cy = W / 2 - 3 , H / 2 + 40

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    print("K=\n", K)
    return K


def draw_pts_img(img, xi, yi, clamp=False):
    if img is None:
        return None
    out = img.copy()
    h, w = out.shape[:2]

    if clamp:
        xi = np.clip(xi.astype(np.int32), 0, w-1)
        yi = np.clip(yi.astype(np.int32), 0, h-1)

    for x, y in zip(xi, yi):
        if 0 <= x < w and 0 <= y < h:
            cv2.circle(out, (int(x), int(y)), 2, (0, 255, 0), -1)
    return out


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

        # Precompute
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat    = getCameraMat(params_cam)

        # Subscriptions
        self.create_subscription(Image,     '/camera/image_raw', self.img_callback,  qos_img)
        self.create_subscription(LaserScan, '/scan',             self.scan_callback, qos_profile=scan_qos)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("LiDAR->Camera projection node (ROS2) started.")

    def img_callback(self, msg: Image):
        # self.get_logger().info(f"Image: {msg.width}x{msg.height}, enc={msg.encoding}")
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")

    def scan_callback(self, msg: LaserScan):
        xs, ys = [], []
        angle = msg.angle_min

        # ROS standard: x = r*cos(theta), y = r*sin(theta), theta increases CCW from +x
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                xs.append(-r * math.cos(angle))  # forward
                ys.append(r * math.sin(angle))  # left
            angle += msg.angle_increment

        if xs:
            xyz = np.stack(
                [np.array(xs, dtype=np.float32),
                 np.array(ys, dtype=np.float32),
                 np.zeros(len(xs), dtype=np.float32)],
                axis=1
            )
            self.pc_np = xyz
        else:
            self.pc_np = None

    def transform_lidar_to_camera(self, pc_lidar_homo):
        cam_temp = self.TransformMat @ pc_lidar_homo  # (4,N)
        return cam_temp[:3, :]

    def project_camera_to_image(self, pc_camera):
        uvw = self.CameraMat @ pc_camera  # (3,N)

        # Keep only points in front of camera
        valid = uvw[2, :] > 0.0
        if not np.any(valid):
            return None
        uvw = uvw[:, valid]
        uvw /= uvw[2, :]

        u, v = uvw[0, :], uvw[1, :]
        # self.get_logger().info(f"u[min,max]=[{float(u.min()):.1f},{float(u.max()):.1f}] "
        #                        f"v[min,max]=[{float(v.min()):.1f},{float(v.max()):.1f}]")

        valid2 = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        if not np.any(valid2):
            return None
        return uvw[:, valid2]

    def timer_callback(self):
        if self.img is None or self.pc_np is None:
            return

        xyz = self.pc_np
        pc_h = np.concatenate([xyz, np.ones((xyz.shape[0], 1), dtype=np.float32)], axis=1).T  # (4,N)

        # LIDAR -> Camera(optical)
        pc_cam = self.transform_lidar_to_camera(pc_h)

        z = pc_cam[2, :]
        # self.get_logger().info(f"z>0 ratio={np.mean(z>0):.3f}, "
        #                        f"z[min,max]=[{float(z.min()):.2f},{float(z.max()):.2f}]")

        # Camera -> Image
        uvw = self.project_camera_to_image(pc_cam)
        if uvw is None:
            # self.get_logger().info("proj pts=0 (uvw=None)")
            return

        proj = draw_pts_img(self.img, uvw[0, :], uvw[1, :], clamp=False)
        if proj is not None:
            cv2.imshow("Lidar->Camera Projection (2D LiDAR, ROS2)", proj)
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
