"""Convert FAST_LIO's camera_init/body pose to Nav2's odom/base_link pose.

This is a rigid-body conversion, not a frame-name replacement.  FAST_LIO
estimates T_camera_init_body; Nav2 needs T_odom_base.  With odom initially
coincident with camera_init, the latter is:

    T_odom_base = T_camera_init_body * inverse(T_base_body)
"""

import math
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FastLioOdomAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_adapter")
        self.declare_parameter("input_odom_topic", "/Odometry")
        self.declare_parameter("output_odom_topic", "/fastlio/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # Measured/derived base_link -> FAST_LIO IMU body transform.  The
        # defaults derive from base->livox=(0.31,0,0.365,yaw=pi) and the
        # current mid360.yaml lidar->IMU extrinsic.
        self.declare_parameter("base_to_body_xyz", [0.299, -0.02329, 0.32088])
        self.declare_parameter("base_to_body_rpy", [0.0, 0.0, math.pi])
        input_topic = self.get_parameter("input_odom_topic").value
        output_topic = self.get_parameter("output_odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        xyz = self.get_parameter("base_to_body_xyz").value
        rpy = self.get_parameter("base_to_body_rpy").value
        self.t_base_body = self.pose_matrix(xyz, rpy)
        self.t_body_base = np.linalg.inv(self.t_base_body)
        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, input_topic, self.on_odom, 50)
        self.get_logger().info(
            f"Rigidly converting {input_topic} camera_init/body to "
            f"{output_topic} {self.odom_frame}/{self.base_frame}"
        )

    @staticmethod
    def pose_matrix(xyz, rpy):
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr],
        ])
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = xyz
        return result

    @staticmethod
    def quaternion_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        result = np.eye(4)
        result[:3, :3] = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        return result

    @staticmethod
    def matrix_quaternion(rotation):
        trace = np.trace(rotation)
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return ((rotation[2, 1]-rotation[1, 2])/s,
                    (rotation[0, 2]-rotation[2, 0])/s,
                    (rotation[1, 0]-rotation[0, 1])/s, 0.25*s)
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = math.sqrt(1.0+rotation[0, 0]-rotation[1, 1]-rotation[2, 2])*2
            return (0.25*s, (rotation[0, 1]+rotation[1, 0])/s,
                    (rotation[0, 2]+rotation[2, 0])/s,
                    (rotation[2, 1]-rotation[1, 2])/s)
        if index == 1:
            s = math.sqrt(1.0+rotation[1, 1]-rotation[0, 0]-rotation[2, 2])*2
            return ((rotation[0, 1]+rotation[1, 0])/s, 0.25*s,
                    (rotation[1, 2]+rotation[2, 1])/s,
                    (rotation[0, 2]-rotation[2, 0])/s)
        s = math.sqrt(1.0+rotation[2, 2]-rotation[0, 0]-rotation[1, 1])*2
        return ((rotation[0, 2]+rotation[2, 0])/s,
                (rotation[1, 2]+rotation[2, 1])/s, 0.25*s,
                (rotation[1, 0]-rotation[0, 1])/s)

    def on_odom(self, message):
        from nav_msgs.msg import Odometry
        source = self.quaternion_matrix(message.pose.pose.orientation)
        source[:3, 3] = [message.pose.pose.position.x,
                         message.pose.pose.position.y,
                         message.pose.pose.position.z]
        converted = source @ self.t_body_base
        qx, qy, qz, qw = self.matrix_quaternion(converted[:3, :3])
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = float(converted[0, 3])
        output.pose.pose.position.y = float(converted[1, 3])
        output.pose.pose.position.z = float(converted[2, 3])
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.pose.covariance = message.pose.covariance
        rotation_base_body = self.t_base_body[:3, :3]
        linear_body = np.array([message.twist.twist.linear.x,
                                message.twist.twist.linear.y,
                                message.twist.twist.linear.z])
        angular_body = np.array([message.twist.twist.angular.x,
                                 message.twist.twist.angular.y,
                                 message.twist.twist.angular.z])
        angular_base = rotation_base_body @ angular_body
        linear_base = (rotation_base_body @ linear_body
                       + np.cross(angular_base, -self.t_base_body[:3, 3]))
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = linear_base
        output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = angular_base
        output.twist.covariance = message.twist.covariance
        self.publisher.publish(output)
        if not self.publish_tf:
            return
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
