import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

       
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            1)
        self.subscription  # prevent unused variable warning

    def handle_turtle_pose(self, msg):
        odom_tf = TransformStamped()
        base_link_tf = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = 'map'
        odom_tf.child_frame_id = 'odom'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        odom_tf.transform.translation.x = 0.0
        odom_tf.transform.translation.y = 0.0
        odom_tf.transform.translation.z = 0.0

        # No orientation on the map
        q = quaternion_from_euler(0, 0, 0)
        odom_tf.transform.rotation.x = q[0]
        odom_tf.transform.rotation.y = q[1]
        odom_tf.transform.rotation
        self.tf_broadcaster.sendTransform(odom_tf)


        # Read message content and assign it to
        # corresponding tf variables
        base_link_tf.header.stamp = self.get_clock().now().to_msg()
        base_link_tf.header.frame_id = 'odom'
        base_link_tf.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        base_link_tf.transform.translation.x = msg.x
        base_link_tf.transform.translation.y = msg.y
        base_link_tf.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        base_link_tf.transform.rotation.x = q[0]
        base_link_tf.transform.rotation.y = q[1]
        base_link_tf.transform.rotation.z = q[2]
        base_link_tf.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(base_link_tf)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()