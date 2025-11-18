#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Subscribes to PersonGatedWithIDArray (in MAP frame) and publishes worker list to MQTT as JSON.

- In : /dx/person_gated_with_id_map  (dx_msgs/PersonGatedWithIDArray)
- Out: MQTT topic (default: /worker/data)

Payload example:
[
  {"name":"작업자A","track_id":12,"x":-5.0,"y":-1.8,"helmet":true,"vest":true,"safe":true,"stamp":1731489478000000000},
  {"name":"작업자B","track_id":7, "x":-3.5,"y":-1.1,"helmet":false,"vest":true,"safe":false,"stamp":1731489478000000000},
  {"name":"미상인원","track_id":21,"x":-1.0,"y":0.2,"helmet":false,"vest":false,"safe":false,"stamp":1731489478000000000}
]

자리 규칙:
- names 파라미터로 슬롯 이름 정의: ["작업자A","작업자B","작업자C","작업자D"]
- 슬롯 수 = len(names)
- 각 슬롯은 한 시점에 하나의 track_id만 가짐
- 이전 프레임에 있던 track_id가 이번 프레임에 없으면 → 해당 슬롯을 비움
- 새 track_id가 들어왔을 때
    - 빈 슬롯이 있으면 → 그 슬롯에 배정
    - 모든 슬롯이 차 있으면 → name="미상인원"으로 전송
"""

import json
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

# 커스텀 메시지 (패키지 이름/경로는 네 환경에 맞춰서)
from dx_msgs.msg import PersonGatedWithIDArray


class WorkerMqttPublisher(Node):
    def __init__(self):
        super().__init__('worker_mqtt_publisher')

        # ---------- Parameters ----------
        self.declare_parameter('pose_topic', '/dx/person_gated_with_id_map')
        self.declare_parameter('mqtt_host', '192.168.0.74')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', '/worker/data')
        self.declare_parameter('mqtt_qos', 1)
        # 슬롯 이름 (길이가 곧 슬롯 개수)
        self.declare_parameter('names', ['작업자A', '작업자B', '작업자C'])
        self.declare_parameter('publish_rate_limit_hz', 20.0)

        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.mqtt_host = self.get_parameter('mqtt_host').get_parameter_value().string_value
        self.mqtt_port = int(self.get_parameter('mqtt_port').get_parameter_value().integer_value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
        self.mqtt_qos = int(self.get_parameter('mqtt_qos').get_parameter_value().integer_value)
        self.names: List[str] = [
            str(s) for s in self.get_parameter('names').get_parameter_value().string_array_value
        ]
        self.rate_limit_hz = float(
            self.get_parameter('publish_rate_limit_hz').get_parameter_value().double_value
        )

        self.num_slots = len(self.names)

        # ---------- 슬롯 관리 구조 ----------
        # slot_index (0..N-1) → track_id (또는 None)
        self.slot_track_ids: List[Optional[int]] = [None] * self.num_slots
        # track_id → slot_index
        self.track_to_slot: Dict[int, int] = {}

        # ---------- MQTT Client ----------
        if mqtt is None:
            self.get_logger().error('paho-mqtt not installed. Install with: pip3 install paho-mqtt')
            raise RuntimeError('paho-mqtt missing')

        self.client = mqtt.Client()
        self.client.enable_logger()
        try:
            self.client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.client.loop_start()
            self.get_logger().info(
                f"Connected to MQTT {self.mqtt_host}:{self.mqtt_port}, publish → {self.mqtt_topic}"
            )
        except Exception as e:
            self.get_logger().warn(f"MQTT initial connect failed: {e}")

        # ---------- ROS I/O ----------
        self.sub = self.create_subscription(
            PersonGatedWithIDArray, self.pose_topic, self.person_cb, qos_profile_sensor_data
        )
        self._last_pub_ns = 0

    # -------------------- Utils --------------------
    def _rate_limited(self, now_ns: int) -> bool:
        if self.rate_limit_hz <= 0:
            return False
        min_interval_ns = int(1e9 / self.rate_limit_hz)
        if now_ns - self._last_pub_ns < min_interval_ns:
            return True
        self._last_pub_ns = now_ns
        return False

    def _assign_slot_for_track(self, track_id: int) -> Optional[int]:
        """track_id에 슬롯을 배정/복구. 없으면 새 슬롯, 꽉 찼으면 None."""
        if track_id in self.track_to_slot:
            return self.track_to_slot[track_id]

        # 빈 슬롯 탐색
        for idx in range(self.num_slots):
            if self.slot_track_ids[idx] is None:
                # 이 슬롯을 새 track_id에 할당
                self.slot_track_ids[idx] = track_id
                self.track_to_slot[track_id] = idx
                return idx

        # 빈 슬롯 없음 → 미상인원 처리 예정
        return None

    def _cleanup_slots(self, alive_track_ids: List[int]):
        """이번 프레임에 없는 track_id는 슬롯에서 제거."""
        alive_set = set(alive_track_ids)

        # slot_track_ids를 순회하면서 죽은 트랙 제거
        for idx in range(self.num_slots):
            tid = self.slot_track_ids[idx]
            if tid is not None and tid not in alive_set:
                # 슬롯 반환
                self.slot_track_ids[idx] = None
                if tid in self.track_to_slot:
                    del self.track_to_slot[tid]

    # -------------------- Callback --------------------
    def person_cb(self, msg: PersonGatedWithIDArray):
        now_ns = self.get_clock().now().nanoseconds
        if self._rate_limited(now_ns):
            return

        persons = msg.persons
        # 이번 프레임에 실제 존재하는 track_id 목록
        alive_ids: List[int] = []

        items = []

        for p in persons:
            tid = int(p.track_id)
            # 유효하지 않은 track_id(예: 0xFFFFFFFF) 체크는 필요하면 추가
            alive_ids.append(tid)

            slot_idx = self._assign_slot_for_track(tid)
            if slot_idx is not None:
                name = self.names[slot_idx]
            else:
                name = "미상인원"

            # safe는 msg 안에 있으면 그대로 사용, 없으면 helmet & vest로 계산
            safe = bool(getattr(p, "safe", (p.helmet and p.vest)))

            item = {
                "name": name,
                "track_id": tid,
                "x": float(p.pose.position.x),
                "y": float(p.pose.position.y),
                "helmet": bool(p.helmet),
                "vest": bool(p.vest),
                "safe": safe,
                "stamp": int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec),
            }
            items.append(item)

        # 이번 프레임에 없는 track_id는 슬롯에서 해제
        self._cleanup_slots(alive_ids)

        # MQTT 전송
        try:
            payload = json.dumps(items, ensure_ascii=False, separators=(',', ':'))
            result = self.client.publish(self.mqtt_topic, payload=payload, qos=self.mqtt_qos, retain=False)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f"MQTT publish rc={result.rc}")
        except Exception as e:
            self.get_logger().warn(f"MQTT publish failed: {e}")


def main():
    rclpy.init()
    node = WorkerMqttPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.client.loop_stop()
            node.client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
