#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros

WHEEL_DIAMETER = 0.085  # meters
TICKS_PER_REV = 240
WHEEL_BASE = 0.32  # meters
WHEEL_CIRC = math.pi * WHEEL_DIAMETER
TICK_DISTANCE = WHEEL_CIRC / TICKS_PER_REV

# Encoder pins (BCM mode)
ENC_A = 6
ENC_B = 5

class HybridOdometryPublisher(Node):
    def __init__(self):
        super().__init__('hybrid_odometry_publisher')

        # Encoder state
        self.right_ticks = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=self.encoder_callback)

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity from cmd_vel
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0

        # ROS
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odometry)

    def encoder_callback(self, channel):
        a = GPIO.input(ENC_A)
        b = GPIO.input(ENC_B)
        if a == b:
            self.right_ticks += 1
        else:
            self.right_ticks -= 1

    def cmd_vel_callback(self, msg):
        self.linear_cmd = msg.linear.x
        self.angular_cmd = msg.angular.z

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Calculate right wheel distance
        right_distance = self.right_ticks * TICK_DISTANCE
        self.right_ticks = 0

        # Estimate left wheel distance from cmd_vel
        if self.angular_cmd == 0.0:
            left_distance = right_distance
        else:
            v_r = (self.linear_cmd + self.angular_cmd * WHEEL_BASE / 2)
            v_l = (self.linear_cmd - self.angular_cmd * WHEEL_BASE / 2)
            if v_r != 0.0:
                left_distance = right_distance * (v_l / v_r)
            else:
                left_distance = right_distance

        # Average distances
        d = (right_distance + left_distance) / 2
        delta_theta = (right_distance - left_distance) / WHEEL_BASE

        # Integrate position
        self.theta += delta_theta
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        q = quaternion_from_euler(0, 0, self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear.x = self.linear_cmd
        odom.twist.twist.angular.z = self.angular_cmd
        self.odom_pub.publish(odom)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HybridOdometryPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

