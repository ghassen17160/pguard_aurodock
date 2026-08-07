#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class CameraFrameFix(Node):

    def __init__(self):
        super().__init__('camera_frame_fix')

        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('input_camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_image_topic', '/camera_fixed/image_raw')
        self.declare_parameter('output_camera_info_topic', '/camera_fixed/camera_info')
        self.declare_parameter('correct_frame_id', 'camera_link_optical')

        in_image = self.get_parameter('input_image_topic').value
        in_info = self.get_parameter('input_camera_info_topic').value
        out_image = self.get_parameter('output_image_topic').value
        out_info = self.get_parameter('output_camera_info_topic').value
        self.correct_frame_id = self.get_parameter('correct_frame_id').value

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.image_pub = self.create_publisher(Image, out_image, qos)
        self.info_pub = self.create_publisher(CameraInfo, out_info, qos)

        self.create_subscription(Image, in_image, self.on_image, qos)
        self.create_subscription(CameraInfo, in_info, self.on_info, qos)

        self.get_logger().info(
            f"camera_frame_fix actif : '{in_image}'/'{in_info}' -> "
            f"'{out_image}'/'{out_info}' avec frame_id corrige en "
            f"'{self.correct_frame_id}'."
        )

    def on_image(self, msg: Image):
        msg.header.frame_id = self.correct_frame_id
        self.image_pub.publish(msg)

    def on_info(self, msg: CameraInfo):
        msg.header.frame_id = self.correct_frame_id
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFrameFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
