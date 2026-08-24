# ==============================================================================
# SCRIPT: fall_detector_all_in_one.py
#
# ROLE:
#   - Hailo YOLO pose 파이프라인 위에서 낙상 감지 + 스냅샷 + job JSON 생성 + MQTT 트리거 처리
#   - Hailo는 이 프로세스 하나만 사용 (다른 코드에서 Hailo 쓰지 말 것)
#
# FEATURES:
#   1) Hailo pose → 낙상 감지 (velocity + posture hybrid)
#   2) 낙상 확정 시:
#        - fall_snapshots/ 에 JPG 저장
#        - analysis_queue/ 에 job_*.json 생성
#   3) MQTT /system/camera/capture 구독:
#        - 외부에서 이 토픽으로 메시지 보내면, 그 프레임으로 수동 스냅샷 + job JSON 생성
#        - payload가 JSON이면 그대로 job 안에 meta로 넣어줌
#   4) fallen 상태에서 사람이 일어나 standing으로 회복되면:
#        - recovered 이벤트를 JSON 로그로 생성 (snapshot 없이)
#        - 이후 다시 낙상 재탐지 가능
# ==============================================================================

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

import os
import time
import datetime
import json
from dataclasses import dataclass
from typing import Optional, List

import cv2
import numpy as np
import hailo
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

from hailo_apps.hailo_app_python.core.common.buffer_utils import (
    get_caps_from_pad,
    get_numpy_from_buffer,
)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.pose_estimation.pose_estimation_pipeline import (
    GStreamerPoseEstimationApp,
)

# ------------------------------------------------------------------------------
# 환경 변수 및 기본 설정
# ------------------------------------------------------------------------------
load_dotenv()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SNAPSHOT_DIR = os.path.join(BASE_DIR, "fall_snapshots")
ANALYSIS_QUEUE_DIR = os.path.join(BASE_DIR, "analysis_queue")

os.makedirs(SNAPSHOT_DIR, exist_ok=True)
os.makedirs(ANALYSIS_QUEUE_DIR, exist_ok=True)

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "fall_all_in_one")
MQTT_TOPIC_CAPTURE = os.getenv("MQTT_TOPIC_CAMERA_CAPTURE", "/system/camera/capture")

# 낙상 알고리즘 파라미터 (env에서 없으면 기본값 사용)
VELOCITY_THRESHOLD = float(os.getenv("VELOCITY_THRESHOLD", "0.08"))
VERTICAL_SPREAD_THRESHOLD = float(os.getenv("VERTICAL_SPREAD_THRESHOLD", "0.35"))
EMA_ALPHA = float(os.getenv("EMA_ALPHA", "0.1"))
POST_FALL_DWELL_FRAMES = int(os.getenv("POST_FALL_DWELL_FRAMES", "45"))
PROLONGED_LIE_DOWN_FRAMES = int(os.getenv("PROLONGED_LIE_DOWN_FRAMES", "90"))
RECOVERY_FRAMES_THRESHOLD = int(os.getenv("RECOVERY_FRAMES_THRESHOLD", "15"))

DEBUG_LOG_INTERVAL = 5.0  # 초 단위


# ------------------------------------------------------------------------------
# 상태 관리용 데이터 클래스
# ------------------------------------------------------------------------------
@dataclass
class PersonState:
    last_hip_y: Optional[float] = None
    last_ts: Optional[float] = None
    velocity: float = 0.0
    smoothed_vertical_spread: float = 1.0
    status: str = "standing"  # standing / monitoring / fallen
    h_dwell_frames: int = 0
    prolonged_frames: int = 0
    recovery_frames: int = 0
    fall_active: bool = False  # fallen 상태 이벤트 이미 발생했는지


# ------------------------------------------------------------------------------
# MQTT 설정
# ------------------------------------------------------------------------------
def create_mqtt_client(user_data) -> mqtt.Client:
    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.user_data_set(user_data)

    def on_connect(c, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"[MQTT] Connected to {MQTT_BROKER}:{MQTT_PORT} as {MQTT_CLIENT_ID}")
            c.subscribe(MQTT_TOPIC_CAPTURE)
            print(f"[MQTT] Subscribed to {MQTT_TOPIC_CAPTURE}")
        else:
            print(f"[MQTT] Connect failed with code {rc}")

    def on_message(c, userdata, msg):
        # /system/camera/capture 수신 → 수동 캡처 요청
        if msg.topic != MQTT_TOPIC_CAPTURE:
            return

        payload_raw = msg.payload.decode("utf-8", errors="ignore")
        meta = None
        try:
            meta = json.loads(payload_raw)
            print(f"[MQTT] Capture trigger (JSON) received: {meta}")
        except json.JSONDecodeError:
            if payload_raw.strip():
                meta = {"raw_payload": payload_raw}
                print(f"[MQTT] Capture trigger (text) received: {payload_raw}")
            else:
                meta = {}

        userdata.request_manual_snapshot(meta)

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
    client.loop_start()
    return client


# ------------------------------------------------------------------------------
# Hailo 유틸: main person & landmarks
# ------------------------------------------------------------------------------
def select_main_person(roi):
    """가장 큰 person detection 1명 + 그 landmarks 반환."""
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    if len(detections) == 0:
        return None, None

    best_det = None
    best_area = 0.0
    for det in detections:
        try:
            if det.get_label() != "person":
                continue
        except Exception:
            continue
        bbox = det.get_bbox()
        area = bbox.width() * bbox.height()
        if area > best_area:
            best_area = area
            best_det = det

    if best_det is None:
        return None, None

    lms = best_det.get_objects_typed(hailo.HAILO_LANDMARKS)
    if not lms:
        return None, None

    points = lms[0].get_points()
    bbox = best_det.get_bbox()
    return points, bbox


def global_y(point, bbox):
    """landmark point + bbox → 프레임 기준 정규화 y좌표(0~1)."""
    return bbox.ymin() + point.y() * bbox.height()


# ------------------------------------------------------------------------------
# 낙상 상태 업데이트 로직
# ------------------------------------------------------------------------------
def update_fall_state(points, bbox, state: PersonState, frame_idx: int):
    """
    한 프레임에 대해:
      - hip y → velocity 계산
      - keypoint y-spread → posture 판단
      - 상태 머신 업데이트
      - 새 낙상 이벤트 / 회복 이벤트 발생 여부와 메타데이터 반환

    meta["event"]:
      - "fall"       : 새 낙상 확정
      - "recovered"  : fallen → standing 회복
      - "state"      : 그 외 상태 업데이트
    """
    now = time.time()

    if not points:
        # 사람 없으면 그냥 상태만 기록
        state.last_ts = now
        state.velocity = 0.0
        meta = {
            "event": "state",
            "reason": None,
            "frame": frame_idx,
            "timestamp": datetime.datetime.now().isoformat(),
            "velocity": 0.0,
            "hip_y": None,
            "smoothed_vertical_spread": state.smoothed_vertical_spread,
            "is_horizontal": False,
            "status": state.status,
            "h_dwell_frames": state.h_dwell_frames,
            "prolonged_frames": state.prolonged_frames,
            "recovery_frames": state.recovery_frames,
        }
        return False, meta

    # 1) 힙 Y (left hip=11, right hip=12 평균)
    LEFT_HIP_IDX = 11
    RIGHT_HIP_IDX = 12
    hip_ys = []
    for idx in (LEFT_HIP_IDX, RIGHT_HIP_IDX):
        if idx < len(points):
            hip_ys.append(global_y(points[idx], bbox))

    if hip_ys:
        hip_y = sum(hip_ys) / len(hip_ys)
    else:
        hip_y = state.last_hip_y if state.last_hip_y is not None else 0.5

    # 2) 수직 속도
    vel = state.velocity
    if state.last_ts is not None and state.last_hip_y is not None:
        dt = now - state.last_ts
        if dt > 0:
            vel = (hip_y - state.last_hip_y) / dt
    state.velocity = vel
    state.last_hip_y = hip_y
    state.last_ts = now

    # 3) posture / spread
    candidate_idxs = [5, 6, 11, 12, 13, 14, 15, 16]
    y_list = []
    for idx in candidate_idxs:
        if idx < len(points):
            y_list.append(global_y(points[idx], bbox))

    if y_list:
        vertical_spread = max(y_list) - min(y_list)
    else:
        vertical_spread = state.smoothed_vertical_spread

    ema = EMA_ALPHA * vertical_spread + (1.0 - EMA_ALPHA) * state.smoothed_vertical_spread
    state.smoothed_vertical_spread = ema
    is_horizontal = ema < VERTICAL_SPREAD_THRESHOLD

    # 4) 상태 머신
    status = state.status
    fall_triggered = False
    reason = None

    # 수평 프레임 카운트 (천천히 누운 경우용)
    if is_horizontal:
        state.prolonged_frames += 1
    else:
        state.prolonged_frames = 0

    if status == "standing":
        state.h_dwell_frames = 0
        state.recovery_frames = 0
        if vel > VELOCITY_THRESHOLD:
            status = "monitoring"

    elif status == "monitoring":
        if is_horizontal:
            state.h_dwell_frames += 1
            if state.h_dwell_frames >= POST_FALL_DWELL_FRAMES:
                status = "fallen"
                reason = "sudden_fall"
        else:
            if vel <= 0:
                status = "standing"
                state.h_dwell_frames = 0

    elif status == "fallen":
        if not is_horizontal:
            state.recovery_frames += 1
            if state.recovery_frames >= RECOVERY_FRAMES_THRESHOLD:
                status = "standing"
                state.h_dwell_frames = 0
                state.recovery_frames = 0
                state.fall_active = False
        else:
            state.recovery_frames = 0

    # 천천히 누운 경우
    if status != "fallen" and state.prolonged_frames >= PROLONGED_LIE_DOWN_FRAMES:
        status = "fallen"
        reason = "prolonged_horizontal"

    # 이전 상태와 현재 상태 비교 (회복 감지용)
    prev_status = state.status
    state.status = status

    # fall_active / 재탐지 플래그
    if status == "fallen" and not state.fall_active:
        state.fall_active = True
        fall_triggered = True
    if status != "fallen":
        state.fall_active = False

    # 회복 이벤트 여부
    recovered = prev_status == "fallen" and status == "standing"

    # meta.event 결정
    if recovered:
        event_name = "recovered"
        meta_reason = "recovered_after_fall"
    elif fall_triggered:
        event_name = "fall"
        meta_reason = reason
    else:
        event_name = "state"
        meta_reason = reason

    meta = {
        "event": event_name,
        "reason": meta_reason,
        "frame": frame_idx,
        "timestamp": datetime.datetime.now().isoformat(),
        "velocity": vel,
        "hip_y": hip_y,
        "smoothed_vertical_spread": ema,
        "is_horizontal": is_horizontal,
        "status": status,
        "h_dwell_frames": state.h_dwell_frames,
        "prolonged_frames": state.prolonged_frames,
        "recovery_frames": state.recovery_frames,
        "prev_status": prev_status,
    }

    return fall_triggered, meta


# ------------------------------------------------------------------------------
# 콜백용 클래스 (스냅샷 + 상태 + MQTT 연계)
# ------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self, mqtt_client: mqtt.Client):
        super().__init__()
        self.mqtt_client = mqtt_client
        self.person_state = PersonState()
        self.start_time = time.time()
        self.last_debug_time = 0.0

        self.current_frame_bgr: Optional[np.ndarray] = None
        self.snapshot_counter = 0

        # MQTT로 들어오는 manual capture 요청
        self._manual_snapshot_requested = False
        self._manual_snapshot_meta = None

    # MQTT on_message에서 호출
    def request_manual_snapshot(self, meta: Optional[dict]):
        self._manual_snapshot_requested = True
        self._manual_snapshot_meta = meta or {}

    def pop_manual_snapshot_request(self):
        if self._manual_snapshot_requested:
            self._manual_snapshot_requested = False
            meta = self._manual_snapshot_meta or {}
            self._manual_snapshot_meta = None
            return True, meta
        return False, None

    # 스냅샷 + job JSON 생성 (fall / manual)
    def save_snapshot_and_job(self, trigger_type: str, meta: dict):
        """
        trigger_type: "fall" / "manual"
        meta: 낙상 알고리즘 메타데이터 또는 MQTT payload 등
        """
        if self.current_frame_bgr is None:
            print("[SNAPSHOT] No current frame available, skip.")
            return

        now = datetime.datetime.now()
        ts_str = now.strftime("%Y%m%d_%H%M%S")
        img_name = f"{trigger_type}_snapshot_{ts_str}.jpg"
        img_path = os.path.join(SNAPSHOT_DIR, img_name)

        # 이미지 저장
        try:
            cv2.imwrite(img_path, self.current_frame_bgr)
            self.snapshot_counter += 1
            print(f"[SNAPSHOT] Saved image: {img_path}")
        except Exception as e:
            print(f"[SNAPSHOT] Failed to save image: {e}")
            return

        # job JSON 구성
        job = {
            "event_type": "fall" if trigger_type == "fall" else "manual_capture",
            "trigger": trigger_type,
            "created_at": now.isoformat(),
            "snapshot_path": img_path,
            "meta": meta,
        }

        job_name = f"job_{trigger_type}_{ts_str}.json"
        job_path = os.path.join(ANALYSIS_QUEUE_DIR, job_name)

        try:
            with open(job_path, "w", encoding="utf-8") as f:
                json.dump(job, f, ensure_ascii=False, indent=2)
            print(f"[JOB] Saved job file: {job_path}")
        except Exception as e:
            print(f"[JOB] Failed to save job file: {e}")

    # 회복 이벤트용 JSON만 생성 (스냅샷 없음)
    def save_recovery_job(self, meta: dict):
        """
        fallen → standing 회복 시 JSON 로그만 남김.
        텔레그램 alert_sender 쪽에서 event_type == 'recovered' 로 구분 가능.
        """
        now = datetime.datetime.now()
        ts_str = now.strftime("%Y%m%d_%H%M%S")

        job = {
            "event_type": "recovered",
            "trigger": "recovered",
            "created_at": now.isoformat(),
            "snapshot_path": None,
            "meta": meta,
        }

        job_name = f"job_recovered_{ts_str}.json"
        job_path = os.path.join(ANALYSIS_QUEUE_DIR, job_name)

        try:
            with open(job_path, "w", encoding="utf-8") as f:
                json.dump(job, f, ensure_ascii=False, indent=2)
            print(f"[JOB] Saved recovery job file: {job_path}")
        except Exception as e:
            print(f"[JOB] Failed to save recovery job file: {e}")


# ------------------------------------------------------------------------------
# GStreamer / Hailo 콜백
# ------------------------------------------------------------------------------
def app_callback(pad, info, user_data: user_app_callback_class):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    frame_idx = user_data.get_count()

    # numpy frame 추출 (RGB → BGR)
    try:
        fmt, width, height = get_caps_from_pad(pad)
        if not (fmt and width and height):
            return Gst.PadProbeReturn.OK
        frame_rgb = get_numpy_from_buffer(buffer, fmt, width, height)
        if frame_rgb is None:
            return Gst.PadProbeReturn.OK
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        user_data.current_frame_bgr = frame_bgr
    except Exception as e:
        print(f"[WARN] Failed to get frame from buffer: {e}")
        return Gst.PadProbeReturn.OK

    # Hailo ROI로부터 포즈 정보 추출
    try:
        roi = hailo.get_roi_from_buffer(buffer)
    except Exception as e:
        print(f"[WARN] Failed to get ROI from buffer: {e}")
        return Gst.PadProbeReturn.OK

    points, bbox = select_main_person(roi)
    fall_triggered = False
    meta = None

    if points is not None and bbox is not None:
        fall_triggered, meta = update_fall_state(points, bbox, user_data.person_state, frame_idx)
    else:
        # 사람 없는 경우도 상태 업데이트 형태로 관리
        fall_triggered, meta = update_fall_state([], None, user_data.person_state, frame_idx)

    # 디버그 로그 (5초 간격)
    now = time.time()
    if now - user_data.last_debug_time >= DEBUG_LOG_INTERVAL:
        user_data.last_debug_time = now
        ps = user_data.person_state
        print(
            f"[DEBUG] frame={frame_idx} status={ps.status} "
            f"vel={ps.velocity:.4f} spread={ps.smoothed_vertical_spread:.4f} "
            f"h_dwell={ps.h_dwell_frames} prolonged={ps.prolonged_frames} "
            f"recovery={ps.recovery_frames}"
        )

    # 1) 낙상으로 인한 자동 스냅샷 + job 생성
    if fall_triggered and meta is not None and meta.get("event") == "fall":
        print(f"[FALL] Detected fall at frame={frame_idx}, reason={meta.get('reason')}")
        user_data.save_snapshot_and_job(trigger_type="fall", meta=meta)

    # 1.5) 회복 이벤트 감지 → JSON만 생성
    if meta is not None and meta.get("event") == "recovered":
        print(
            f"[RECOVERY] Person recovered at frame={frame_idx} "
            f"(prev_status={meta.get('prev_status')}, status={meta.get('status')})"
        )
        user_data.save_recovery_job(meta)

    # 2) MQTT에서 들어온 manual capture 요청 처리
    manual_req, manual_meta = user_data.pop_manual_snapshot_request()
    if manual_req:
        combined_meta = {"source": "mqtt_capture"}
        if manual_meta:
            combined_meta["mqtt_payload"] = manual_meta
        # 현재 알고리즘 상태도 같이 넣음 (fall / state / recovered 등 그대로)
        combined_meta["algo_state"] = meta or {}
        user_data.save_snapshot_and_job(trigger_type="manual", meta=combined_meta)

    return Gst.PadProbeReturn.OK


# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == "__main__":
    Gst.init(None)
    print("[INIT] fall_detector_all_in_one starting...")
    print(f"[CONFIG] MQTT broker={MQTT_BROKER}:{MQTT_PORT}, topic={MQTT_TOPIC_CAPTURE}")
    print(
        f"[CONFIG] VELOCITY_THRESHOLD={VELOCITY_THRESHOLD}, "
        f"VERTICAL_SPREAD_THRESHOLD={VERTICAL_SPREAD_THRESHOLD}"
    )

    # user_data 먼저 만들고, 그걸 MQTT user_data로 넘김
    dummy_client = mqtt.Client()  # 임시
    user_data = user_app_callback_class(dummy_client)
    mqtt_client = create_mqtt_client(user_data)
    user_data.mqtt_client = mqtt_client  # 실제 클라이언트로 교체

    app = GStreamerPoseEstimationApp(app_callback, user_data)
    try:
        app.run()
    finally:
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except Exception:
            pass
        print(f"[EXIT] Total snapshots saved: {user_data.snapshot_counter}")
