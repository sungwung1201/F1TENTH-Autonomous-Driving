"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf_transformations
from PIL import Image
import yaml
import os


class TREENode(object):
    def __init__(self,x=None,y=None,parent=None,cost=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
        self.is_root = False

class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.waypoints = np.loadtxt('/home/yoon/sim_ws/log/waypoint.csv', delimiter=',')
        self.goal_idx = 0
        odom_topic = '/ego_racecar/odom'
        scan_topic = '/scan'
        drive_topic= '/drive'
        marker_topic = '/rrt/tree_markers'
        grid_topic = '/rrt/map'
        self.current_pose=None

        self.min_speed = 0.3
        self.max_speed = 1.0
        self.speed = self.max_speed
        self.goal_slow_threshold = 3.0

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg() 
        grid_msg.header.frame_id = 'map'   
        grid_msg.info.resolution=0.4
        grid_msg.info.width=75
        grid_msg.info.height=60
        grid_msg.info.origin.position.x=-16.0
        grid_msg.info.origin.position.y=-8.0
        grid_msg.info.origin.orientation.w=1.0
        grid_msg.data = [0] * (grid_msg.info.width * grid_msg.info.height)
        self.occupancy_grid = grid_msg

        self.tree = []

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            10
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            grid_topic,
            map_qos
        )

        map_yaml_path = '~/sim_ws/src/f1tenth_gym_ros/maps/levine.yaml'
        map_yaml_path = os.path.expanduser(map_yaml_path)
        self.load_map_from_yaml(map_yaml_path)

    def load_map_from_yaml(self, yaml_path):
        import scipy.ndimage

        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)

        pgm_path = os.path.join(os.path.dirname(yaml_path), map_info['image'])
        resolution = map_info['resolution']
        origin = map_info['origin']

        img = Image.open(pgm_path).convert('L')
        img_data = np.array(img)
        img_data = np.flipud(np.fliplr(img_data))
        img_data = np.fliplr(img_data)

        road_mask = img_data > 250
        non_road_mask = ~road_mask
        expanded_non_road = scipy.ndimage.binary_dilation(non_road_mask, iterations=2)

        data = []
        for y in range(img_data.shape[0]):
            for x in range(img_data.shape[1]):
                if road_mask[y, x]:
                    data.append(0)
                elif expanded_non_road[y, x]:
                    data.append(100)
                else:
                    data.append(0)

        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = resolution
        grid_msg.info.width = img_data.shape[1]
        grid_msg.info.height = img_data.shape[0]
        grid_msg.info.origin.position.x = origin[0]
        grid_msg.info.origin.position.y = origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = data

        self.occupancy_grid = grid_msg
        self.map_publisher.publish(grid_msg)

    def marker(self):
        marker_array=MarkerArray()
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "rrt_nodes"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.1
        points_marker.scale.y = 0.1
        points_marker.color.r = 1.0
        points_marker.color.g = 1.0
        points_marker.color.b = 1.0
        points_marker.color.a = 1.0

        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "rrt_edges"
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for node in self.tree:
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            points_marker.points.append(p)

            if node.parent is not None:
                parent_p = Point()
                parent_p.x = node.parent.x
                parent_p.y = node.parent.y
                parent_p.z = 0.0
                line_marker.points.append(parent_p)
                line_marker.points.append(p)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)
        self.marker_publisher.publish(marker_array)

    def scan_callback(self, scan_msg):
        car_x, car_y, car_yaw = self.current_pose
        grid_msg = self.occupancy_grid
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)

        obs_range = 5  # 세이프티 버블 반경 (셀 단위)

        for i in range(len(ranges)):
            dist = ranges[i]
            if np.isinf(dist) or np.isnan(dist):
                continue
            angle = angle_min + angle_increment * i
            obs_x = dist * np.cos(angle)
            obs_y = dist * np.sin(angle)

            world_obs_x = car_x + np.cos(car_yaw) * obs_x - np.sin(car_yaw) * obs_y
            world_obs_y = car_y + np.sin(car_yaw) * obs_x + np.cos(car_yaw) * obs_y

            grid_obs_x = int((world_obs_x - grid_msg.info.origin.position.x) / grid_msg.info.resolution)
            grid_obs_y = int((world_obs_y - grid_msg.info.origin.position.y) / grid_msg.info.resolution)

            for dx in range(-obs_range, obs_range + 1):
                for dy in range(-obs_range, obs_range + 1):
                    nx = grid_obs_x + dx
                    ny = grid_obs_y + dy
                    if 0 <= nx < grid_msg.info.width and 0 <= ny < grid_msg.info.height:
                        idx = ny * grid_msg.info.width + nx
                        grid_msg.data[idx] = 100  # 장애물 + 버블 처리

        self.map_publisher.publish(grid_msg)

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = np.array([x, y, yaw])

        if len(self.tree) == 0:
            root = TREENode(x=x, y=y, parent=None, cost=0)
            root.is_root = True
            self.tree.append(root)

        goal_x, goal_y = self.waypoints[self.goal_idx][:2]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        goal_threshold = 1.0

        if dist_to_goal < goal_threshold:
            self.get_logger().info(f"Goal {self.goal_idx} reached. Moving to next waypoint.")
            self.goal_idx += 1
            if self.goal_idx >= len(self.waypoints):
                self.goal_idx = 0
            self.tree = []
            self.path = []
            return

        if dist_to_goal < self.goal_slow_threshold:
            ratio = dist_to_goal / self.goal_slow_threshold
            self.speed = self.min_speed + (self.max_speed - self.min_speed) * ratio
        else:
            self.speed = self.max_speed

        sampled_point = self.sample()
        if sampled_point:
            nearest_idx = self.nearest(self.tree, sampled_point)
            nearest_node = self.tree[nearest_idx]
            new_node = self.steer(nearest_node, sampled_point)

            if not self.check_collision(nearest_node, new_node):
                near_nodes = self.near(self.tree, new_node)
                min_cost = self.cost(self.tree, nearest_node) + self.line_cost(nearest_node, new_node)
                best_parent = nearest_node

                for node in near_nodes:
                    if not self.check_collision(node, new_node):
                        cost = self.cost(self.tree, node) + self.line_cost(node, new_node)
                        if cost < min_cost:
                            min_cost = cost
                            best_parent = node

                new_node.parent = best_parent
                new_node.cost = min_cost
                self.tree.append(new_node)

                for node in near_nodes:
                    if node == new_node or node == new_node.parent:
                        continue
                    if not self.check_collision(new_node, node):
                        new_cost = new_node.cost + self.line_cost(new_node, node)
                        if new_cost < self.cost(self.tree, node):
                            node.parent = new_node
                            node.cost = new_cost
                if self.is_goal(new_node, goal_x, goal_y):
                    self.path = self.find_path(self.tree, new_node)
                else:
                    self.publish_stop()

        self.marker()

        if hasattr(self, 'path') and len(self.path) > 1:
            self.follow_path_control()

        return None

    def follow_path_control(self):
        car_x, car_y, car_yaw = self.current_pose
        dists = [np.hypot(node.x - car_x, node.y - car_y) for node in self.path]
        closest_idx = np.argmin(dists)
        target_idx = min(closest_idx + 1, len(self.path) - 1)
        target_node = self.path[target_idx]

        path_dx = target_node.x - car_x
        path_dy = target_node.y - car_y
        path_yaw = np.arctan2(path_dy, path_dx)

        angle_diff = path_yaw - car_yaw
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        max_steering_angle = 0.34
        steering_angle = max(-max_steering_angle, min(max_steering_angle, angle_diff))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle

        self.drive_publisher.publish(drive_msg)

    def sample(self):
        car_x, car_y, car_yaw = self.current_pose

        x_min=self.occupancy_grid.info.origin.position.x
        y_min=self.occupancy_grid.info.origin.position.y
        x_max = x_min + self.occupancy_grid.info.width * self.occupancy_grid.info.resolution
        y_max = y_min + self.occupancy_grid.info.height * self.occupancy_grid.info.resolution

        for _ in range(100):
            ran_x=random.uniform(x_min,x_max)
            ran_y=random.uniform(y_min,y_max)
            dx=ran_x-car_x
            dy=ran_y-car_y
            forward=np.cos(car_yaw) * dx + np.sin(car_yaw)*dy

            if forward>0:
                grid_x = int((ran_x - x_min) / self.occupancy_grid.info.resolution)
                grid_y = int((ran_y - y_min) / self.occupancy_grid.info.resolution)
                index = grid_y * self.occupancy_grid.info.width + grid_x
                if self.occupancy_grid.data[index] == 0:
                    return (ran_x, ran_y)
        return None

    def nearest(self, tree, sampled_point):
        min_dist = float('inf')
        sample_x,sample_y = sampled_point
        for i in range(len(tree)):
            node = tree[i]
            dist = np.sqrt((node.x - sample_x)**2+(node.y - sample_y)**2)
            if dist<min_dist:
                min_dist=dist
                nearest_idx = i
        return nearest_idx

    def steer(self, nearest_node, sampled_point):
        near_x=nearest_node.x
        near_y=nearest_node.y
        sample_x,sample_y = sampled_point
        dx = sample_x - near_x
        dy = sample_y - near_y
        yaw = np.arctan2(dy,dx)
        dist = np.sqrt(dx**2+dy**2)
        max_dist = 0.5

        new_x = near_x + min(max_dist,dist) * np.cos(yaw)
        new_y = near_y + min(max_dist,dist) * np.sin(yaw)

        new_node = TREENode(x=new_x,y=new_y,parent=nearest_node)

        return new_node

    def check_collision(self, nearest_node, new_node):
        near_x=nearest_node.x
        near_y=nearest_node.y
        new_x=new_node.x
        new_y=new_node.y
        grid_msg = self.occupancy_grid

        dx = (new_x - near_x)/10
        dy= (new_y - near_y)/10
        for i in range(1,11):
            map_a=int((near_x+dx*i-grid_msg.info.origin.position.x) / grid_msg.info.resolution)
            map_b=int((near_y+dy*i-grid_msg.info.origin.position.y) / grid_msg.info.resolution)
            if map_a < 0 or map_a >= grid_msg.info.width or map_b < 0 or map_b >= grid_msg.info.height:
                return True
            index = map_b * grid_msg.info.width + map_a
            if grid_msg.data[index]==100:
                return True

        return False    

    def is_goal(self, latest_added_node, goal_x, goal_y):
        dx=goal_x - latest_added_node.x
        dy=goal_y - latest_added_node.y
        dist = np.sqrt(dx**2 + dy**2)
        if dist <2.0:
            return True
        return False

    def find_path(self, tree, latest_added_node):
        path =[]
        tree.append(latest_added_node)

        node = latest_added_node
        while node is not None:
            path.append(node)
            node = node.parent

        path.reverse()

        return path

    def publish_stop(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_publisher.publish(drive_msg)

    def cost(self, tree, node):
        cost = 0.0
        current = node
        while current.parent is not None:
            dx = current.x - current.parent.x
            dy = current.y - current.parent.y
            cost += np.hypot(dx, dy)
            current = current.parent
        return cost

    def line_cost(self, n1, n2):
        dx = n1.x - n2.x
        dy = n1.y - n2.y
        return np.hypot(dx, dy)

    def near(self, tree, node):
        radius = 2.0
        neighborhood = []
        for other_node in tree:
            dist = np.hypot(other_node.x - node.x, other_node.y - node.y)
            if dist <= radius:
                neighborhood.append(other_node)
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
