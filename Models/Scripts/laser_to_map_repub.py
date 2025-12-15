#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Republish PersonGatedWithIDArray from laser frame → map frame using TF2.

- In : /dx/person_gated_with_id        (frame_id = 'laser')
- Out: /dx/person_gated_with_id_map    (frame_id = 'map')

Only transforms pose. All other fields (track_id, helmet, vest, safe) stay same.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from dx_msgs.msg import PersonGatedWithID, PersonGatedWithIDArray

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose


class PersonWithIDToMap(Node):
    def __init__(self):
        super().__init__('person_with_id_to_map')

        # Parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('topic_in', '/dx/person_gated_with_id')
        self.declare_parameter('topic_out', '/dx/person_gated_with_id_map')

        self.target_frame = self.get_parameter('target_frame').value
        self.topic_in = self.get_parameter('topic_in').value
        self.topic_out = self.get_parameter('topic_out').value

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub = self.create_subscription(
            PersonGatedWithIDArray,
            self.topic_in,
            self.cb,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            PersonGatedWithIDArray,
            self.topic_out,
            10
        )

        self.pub_posearray = self.create_publisher(
            PoseArray,
            '/dx/person_gated_map',   # RViz에서 볼 토픽 이름
            10
        )

        self.get_logger().info(
            f"[PersonWithID → MAP] listening on {self.topic_in}, publishing to {self.topic_out}"
        )

    # -------------------------
    # Callback
    # -------------------------
    def cb(self, msg: PersonGatedWithIDArray):
        source_frame = msg.header.frame_id or 'laser'
        stamp = msg.header.stamp

        # TF lookup
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.3)
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup fail {source_frame}→{self.target_frame}: {e}"
            )
            return

        # Output messages
        out = PersonGatedWithIDArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.target_frame

        for p in msg.persons:
            try:
                # p.pose: geometry_msgs.msg.Pose
                pose_map = do_transform_pose(p.pose, trans)  # Pose → Pose

                # RViz용 PoseArray에 추가
                pose_array.poses.append(pose_map)

                # with_id_map용 메시지
                q = PersonGatedWithID()
                q.header.stamp = stamp
                q.header.frame_id = self.target_frame
                q.pose = pose_map
                q.track_id = p.track_id
                q.helmet = p.helmet
                q.vest = p.vest
                q.safe = p.safe

                out.persons.append(q)

            except Exception as ex:
                self.get_logger().warn(f"Transform fail: {ex}")
                continue

        # 퍼블리시
        self.pub.publish(out)
        self.pub_posearray.publish(pose_array)

        # 디버그용
        self.get_logger().debug(
            f"Published {len(pose_array.poses)} poses to /dx/person_gated_map"
        )



def main():
    rclpy.init()
    node = PersonWithIDToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
