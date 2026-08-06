#!/usr/bin/env python3
"""
camera_frame_fix.py
Package pguard_autodock

Contournement d'un comportement de libgazebo_ros_camera.so (Gazebo Classic) :
malgre <frameName>camera_link</frameName> correctement renseigne dans le SDF
genere, le plugin publie /camera/image_raw et /camera/camera_info avec un
header.frame_id errone (constate : 'base_footprint', vraisemblablement le
lien canonique du modele plutot que le lien reel de la camera). Le rendu de
l'image lui-meme reste correct (la camera est bien physiquement simulee a la
pose de camera_link) -- seule l'ETIQUETTE dans le header est fausse.

Consequence : apriltag_ros calcule la pose du tag dans la convention optique
de la camera, mais la publie via TF comme si son parent etait directement le
frame_id recu (donc 'base_footprint', sans le veritable offset/rotation de
la camera) -- d'ou des positions/orientations de tag totalement incoherentes
une fois recomposees dans base_link.

Ce noeud relaie /camera/image_raw et /camera/camera_info en corrigeant
uniquement le header.frame_id vers le VRAI repere optique de la camera
(camera_link_optical, cf. patch xacro correspondant), sans autre
transformation des donnees. apriltag_ros doit ensuite etre pointe sur les
topics corriges (/camera_fixed/image_raw, /camera_fixed/camera_info) via les
arguments image_topic/camera_info_topic du launch.
"""
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