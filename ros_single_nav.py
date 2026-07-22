#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy
# 定义一个 sensor QoS
qos_profile_reliable = QoSProfile(depth=2)
qos_profile_reliable.reliability = ReliabilityPolicy.RELIABLE
import zmq


context = zmq.Context()
socket = context.socket(zmq.PUB)
#socket.connect("tcp://192.168.100.1:5557")
socket.connect("tcp://127.0.0.1:5557")#临时修改
print("socket connected(?) to port 5557")

# TF
import tf2_ros
from tf_transformations import quaternion_matrix, translation_matrix, concatenate_matrices

# 注意：ROS2 的 message_filters 需要安装对应版本：ros-foxy-message-filters
import message_filters

import numpy as np
from argparse import Namespace
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
# 其他依赖
import threading
import open3d as o3d
import open3d.visualization.gui as gui
from multiprocessing import Process, Queue

# 用户自定义的文件
from arguments import get_args
from agents.ros2_single_agent import ROS_Agent
from utils.vis_gui import ReconstructionWindow
from utils.explored_map_utils import Global_Map_Proc, detect_frontier
import utils.visualization as vu

FORWARD_KEY="w"
LEFT_KEY="a"
RIGHT_KEY="d"
FINISH="f"
import cv2

def get_pose(trans, rot):
    """
    将 TF 中的 Translation / Rotation 转为 4x4 矩阵
    （可根据你实际需要调整坐标轴）
    """
    # for Jackal
    trans_mat = translation_matrix([trans.x, trans.y, trans.z])
    rot_mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    transform_mat = np.dot(trans_mat, rot_mat)
    
    # trans_mat = translation_matrix([0.20882221049280814,
    #                                 -0.00118275956714797,
    #                                 -0.07903620394463086])
    # rot_mat = quaternion_matrix([-0.49352434446717836,
    #                              0.4995478883158239,
    #                              -0.5045718106204675,
    #                              0.5022876831305211])
    
    trans_mat = translation_matrix([0.20626980772421397,
                                    -0.00528269584842609,
                                    -0.015826777380052434])
    rot_mat = quaternion_matrix([0.5130067729487394,
                                    -0.5014588509249872,
                                    0.4940995610242004,
                                    -0.49115037975492243])
    T_lidar_camera = np.dot(trans_mat, rot_mat)
    
    return transform_mat @ T_lidar_camera


class FspNode(Node):
    def __init__(self, args, send_queue, receive_queue):
        super().__init__('fsp_node')

        self.args = args
        self.send_queue = send_queue
        self.receive_queue = receive_queue
        self.point_sum = o3d.geometry.PointCloud()
        self.map_process = Global_Map_Proc(args)

        # 创建 agent
        self.agent = ROS_Agent(self.args, 0, send_queue, receive_queue)

        # 存放观测数据
        self.obs = {}

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 同步订阅
        # 这里的订阅话题名称与原 ROS1 版保持一致，可以根据实际情况修改
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw', qos_profile=qos_profile_reliable)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile_reliable)

        # 同步器
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.rgbd_callback)

        # 普通订阅：CameraInfo
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # 发布器：动作
        self.velocity_publisher = self.create_publisher(Twist, '/robot_action', 10)

        # 定时器：相当于原先 while not rospy.is_shutdown() + rospy.Rate(2)
        # 每 0.5 秒执行一次
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 记录深度图获取的时间戳，用于做 TF lookup
        self.global_timestamp = None

        # Preferred TF child frame for RGB-D pose. Will be auto-selected.
        self.pose_child_frame = None

        # Track last observed RGB-D size for diagnostics.
        self.last_rgb_shape = None
        self.last_depth_shape = None

        self.get_logger().info("FspNode init done.")

    def rgbd_callback(self, rgb_msg, depth_msg):
        """
        message_filters 同步回调
        """
        rgb_h = rgb_msg.height
        rgb_w = rgb_msg.width
        depth_h = depth_msg.height
        depth_w = depth_msg.width

        self.last_rgb_shape = (rgb_h, rgb_w)
        self.last_depth_shape = (depth_h, depth_w)

        # RGB
        rs_rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_h, rgb_w, 3)
        self.obs['rgb'] = rs_rgb

        # Depth
        # Support common encodings:
        # - 16UC1: uint16 in millimeters
        # - 32FC1: float32 in meters (convert to uint16 mm)
        if getattr(depth_msg, 'encoding', '') == '32FC1':
            rs_depth_m = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_h, depth_w)
            rs_depth = np.clip(rs_depth_m * 1000.0, 0, np.iinfo(np.uint16).max).astype(np.uint16)
        else:
            rs_depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_h, depth_w)

        # RealSense aligned depth can occasionally change resolution at runtime.
        # Instead of dropping frames (which makes GUI look stuck), resize depth to RGB size.
        #if (rgb_h != depth_h) or (rgb_w != depth_w):
        #    self.get_logger().warn(
        #        f"RGB/Depth size mismatch: rgb={rgb_w}x{rgb_h}, depth={depth_w}x{depth_h}. "
        #        f"Resizing depth -> rgb for this frame."
        #    )
        #    rs_depth = cv2.resize(rs_depth, (rgb_w, rgb_h), interpolation=cv2.INTER_NEAREST)
        #self.obs['depth'] = rs_depth
        # Use depth resolution as the canonical resolution.
# This avoids artificially upsampling depth from 640x480 to 1280x720.
        if (rgb_h != depth_h) or (rgb_w != depth_w):
            self.get_logger().warn(
                f"RGB/Depth size mismatch: rgb={rgb_w}x{rgb_h}, depth={depth_w}x{depth_h}. "
                f"Resizing RGB -> depth size."
            )

            rs_rgb = cv2.resize(
                rs_rgb,
                (depth_w, depth_h),
                interpolation=cv2.INTER_AREA
            )

            rgb_h, rgb_w = depth_h, depth_w

        self.obs['rgb'] = rs_rgb
        self.obs['depth'] = rs_depth


        # 记录时间戳
        self.global_timestamp = depth_msg.header.stamp

        # print("received rgbd images")

    def camera_info_callback(self, msg):
        """
        订阅相机内参
        """
        # 这里 msg.K 一般是 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # 你先取出需要的 fx, fy, cx, cy
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        camera_matrix = {'cx': cx, 'cy': cy, 'fx': fx, 'fy': fy}
        self.obs['cam_K'] = Namespace(**camera_matrix)
        self.obs['cam_size'] = (msg.height, msg.width)

    def timer_callback(self):
        """
        定时执行，相当于原先 while+rate 循环里的主要逻辑
        """
        # 确保拿到了图像数据
        if 'rgb' not in self.obs or 'depth' not in self.obs or 'cam_K' not in self.obs:
            return
        if self.global_timestamp is None:
            return

        # 获取 TF
        try:
            # Prefer using the RGB-D sensor frame if TF is available;
            # otherwise fall back to base_link.
            if self.pose_child_frame is None:
                candidates = [
                    "camera_link",
                    "camera_color_frame",
                    "camera_color_optical_frame",
                    "camera_depth_frame",
                    "camera_depth_optical_frame",
                    "base_link",
                ]
            else:
                candidates = [self.pose_child_frame]

            transform = None
            last_err = None
            for child_frame in candidates:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "odom",
                        child_frame,
                        rclpy.time.Time(),  # use latest TF
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    if self.pose_child_frame != child_frame:
                        self.pose_child_frame = child_frame
                        self.get_logger().info(f"Using TF child frame: {child_frame}")
                    break
                except tf2_ros.TransformException as e:
                    last_err = e
                    continue

            if transform is None:
                raise last_err
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # 转换成 4x4 矩阵
        trans = transform.transform.translation
        rot = transform.transform.rotation
        self.obs['pose'] = get_pose(trans, rot)
        
        # self.point_sum += self.agent.point_sum

        # 让 agent 做 mapping / act
        action = self.agent.mapping(self.obs)

        vel_msg = Twist()
        if self.agent.l_step < 35:
            socket.send(b"speedctl speed|0.0|0.0|0.5")
        elif action == 1:
            socket.send(b"speedctl speed|0.4|0.0|0.0")
        elif action == 2:
            socket.send(b"speedctl speed|0.0|0.0|0.5")
        elif action == 3:
            socket.send(b"speedctl speed|0.0|0.0|-0.5")
            
        
        # 发布动作
        msg = Int32()
        msg.data = action
        self.velocity_publisher.publish(vel_msg)

       
        # 仅做示例日志
        self.get_logger().info(f"Action published: {action}")


def visualization_thread(args, send_queue, receive_queue):
    """
    Open3D GUI 线程
    """
    app = gui.Application.instance
    app.initialize()
    mono = app.add_font(gui.FontDescription(gui.FontDescription.MONOSPACE))

    app_win = ReconstructionWindow(args, mono, send_queue, receive_queue)
    app.run()  # 阻塞式，直到窗口退出


def main():
    # 1. 初始化 ROS2
    rclpy.init()

    # 2. 获取用户自定义参数
    args = get_args()

    # 3. 建立进程 / 线程间通信队列
    send_queue = Queue()
    receive_queue = Queue()

    # 4. 启动可视化线程
    visualization = threading.Thread(target=visualization_thread, args=(args, send_queue, receive_queue))
    visualization.start()

    # 5. 创建并运行节点
    node = FspNode(args, send_queue, receive_queue)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # 6. 等待可视化线程结束
    visualization.join()
    print("you successfully navigated to destination point")


if __name__ == "__main__":
    main()
