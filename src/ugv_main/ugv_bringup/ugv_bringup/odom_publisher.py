#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import tf2_ros
import math
import os
import numpy as np


ODOM_POSE_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                             0, 1e-3, 1e-9, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = ODOM_POSE_COVARIANCE
ODOM_TWIST_COVARIANCE_STOP = ODOM_POSE_COVARIANCE_STOP

# The raw encoder feed refreshes at only ~10 Hz while odom/odom_raw messages arrive at
# 20 Hz, so during roughly half of steady motion, load-dependent, samples alternate
# zero-delta and double-delta. A single zero sample used to publish the STOP covariance
# (twist[0] = 1e-9), which robot_localization fuses with gain 1.0 as "velocity exactly
# zero": BEAST-01 on 2026-09-07 measured fused displacement at 0.59 x wheel. Debouncing
# the covariance switch here is the fix; the EKF tuning is not at fault.
STOP_COVARIANCE_AFTER_SAMPLES = 5


class OdomPublisher(Node):

    def __init__(self):
        super().__init__("odom_publisher")

        # parameters
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("pub_odom_tf", False)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_footprint_frame = self.get_parameter("base_footprint_frame").value
        self.pub_odom_tf = self.get_parameter("pub_odom_tf").value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # subscribers
        self.create_subscription(Imu, "imu/data", self.handle_imu, 20)
        self.create_subscription(Float32MultiArray, "odom/odom_raw", self.handle_odom, 20)

        # publisher
        self.odom_pub = self.create_publisher(Odometry, "odom_wheel", 20)

        # wheel base from env
        model = os.getenv("UGV_MODEL", "ugv_rover")
        if model == "ugv_rover":
            self.wheel_base = 0.17452
        elif model == "ugv_beast":
            self.wheel_base = 0.143864
        elif model == "rasp_rover":
            self.wheel_base = 0.12960
        else:
            self.wheel_base = 0.17452

        # state variables
        self.is_initialized = False
        self.imu_valid = False

        self.init_left = 0.0
        self.init_right = 0.0

        self.pre_left = 0.0
        self.pre_right = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw_imu = 0.0
        self.yaw_odom = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vw = 0.0

        self.zero_delta_samples = 0

        self.last_time = self.get_clock().now()

        self.get_logger().info("Python odom_publisher started.")

    # Convert quaternion to yaw
    @staticmethod
    def quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # IMU callback
    def handle_imu(self, msg: Imu):
        yaw = self.quat_to_yaw(msg.orientation)
        self.yaw_imu = yaw
        self.imu_valid = True

    # odom raw callback
    def handle_odom(self, msg: Float32MultiArray):

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        left = msg.data[0]
        right = msg.data[1]

        # First-time encoder initialization
        if not self.is_initialized:
            self.init_left = left
            self.init_right = right
            self.is_initialized = True

        left -= self.init_left
        right -= self.init_right

        dleft = left - self.pre_left
        dright = right - self.pre_right
        self.pre_left = left
        self.pre_right = right

        dxy = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_base

        # Count on the raw deltas: a zero delta is a duplicate encoder reading regardless of dt.
        if dxy == 0.0 and dth == 0.0:
            self.zero_delta_samples += 1
        else:
            self.zero_delta_samples = 0

        self.vx = dxy / dt
        self.vw = dth / dt

        if dxy != 0.0:
            dx = math.cos(dth / 2) * dxy
            dy = math.sin(dth / 2) * dxy

            self.x += math.cos(self.yaw) * dx - math.sin(self.yaw) * dy
            self.y += math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        if dth != 0.0:
            self.yaw_odom += dth

        self.yaw = self.yaw_imu if self.imu_valid else self.yaw_odom

        self.publish_odom(now)

    # Publish odom + tf
    def publish_odom(self, stamp: Time):

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame

        # position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # quaternion
        half = self.yaw * 0.5
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        # covariance
        if self.zero_delta_samples >= STOP_COVARIANCE_AFTER_SAMPLES:
            odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
            odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vw

        self.odom_pub.publish(odom)

        # tf
        if self.pub_odom_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_footprint_frame

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0

            tf.transform.rotation.z = math.sin(half)
            tf.transform.rotation.w = math.cos(half)

            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
