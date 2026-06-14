#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
from nav_msgs.msg import Odometry
import tf_transformations

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit')
        # TODO: create ROS subscribers and publishers
        self.waypoints = np.loadtxt('/home/yoon/sim_ws/log/wp-2025-05-14-09-31-38.csv', delimiter=',')
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        self.pose_subscriber = self.create_subscription(
            Odometry, 
            odom_topic,
            self.pose_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        self.L=0.5

    def current_pose(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation)
        current_pose = np.array([x,y,yaw])
        # print(current_pose)
        return current_pose
    
    def goal_point(self,msg):
        current_pose = self.current_pose(msg)
        xy = self.waypoints[:, :2]
        yaw = current_pose[2]

        dist = np.linalg.norm(xy - current_pose[:2], axis=1)

        goal_dist = np.inf
        goal_idx = None

        for i in range(len(dist)):
            if dist[i] < self.L or dist[i] >= goal_dist:
                continue

            dx = xy[i][0] - current_pose[0]
            dy = xy[i][1] - current_pose[1]
            xxx = np.cos(yaw) * dx + np.sin(yaw) * dy#전방점

            if xxx <= 0:
                continue

            goal_dist = dist[i]
            goal_idx = i

        return xy[goal_idx]


    
    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture

        curr = self.current_pose(pose_msg)
        goal=self.goal_point(pose_msg)
        g_x, g_y = goal[0], goal[1]
        c_x, c_y, yaw = curr[0], curr[1], curr[2]
        dx = g_x - c_x
        dy = g_y - c_y

        x_r = np.cos(yaw) * dx + np.sin(yaw) * dy
        y_r = -np.sin(yaw) * dx + np.cos(yaw) * dy
        # print(self.goal_point(pose_msg))
        # print(self.waypoints)
        # TODO: transform goal point to vehicle frame of reference
        gamma = 2*y_r / (self.L**2)
        # print(gamma)
        angle = np.arctan(self.L*gamma)
        max_steering = 0.4189  # ≈ 24 degrees
        angle = np.clip(angle, -max_steering, max_steering)
        # TODO: calculate curvature/steering angle
        drive_msg = AckermannDriveStamped()
        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 1.0

        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)

    pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()