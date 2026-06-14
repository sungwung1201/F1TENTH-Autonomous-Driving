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


# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class TREENode(object):
    def __init__(self,x=None,y=None,parent=None,cost=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        # topics, not saved as attributes
        super().__init__('rrt_node')
        # TODO: grab topics from param file, you'll need to change the yaml file
        odom_topic = '/ego_racecar/odom'
        scan_topic = '/scan'
        drive_topic= '/drive'
        marker_topic = '/rrt/tree_markers'
        grid_topic = '/rrt/map'
        self.current_pose=None
        

        #트리
        self.tree = []
        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.
        map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # TODO: create subscribers
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

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
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
        # class attributes
        # TODO: maybe create your occupancy grid here

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            grid_topic,
            map_qos
        )

        map_yaml_path = '~/sim_ws/src/f1tenth_gym_ros/maps/levine.yaml'
        map_yaml_path = os.path.expanduser(map_yaml_path)
        self.load_map_from_yaml(map_yaml_path)

    def load_map_from_yaml(self, yaml_path):
        # yaml 파일 읽기
        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)
        
        pgm_path = os.path.join(os.path.dirname(yaml_path), map_info['image'])
        resolution = map_info['resolution']
        origin = map_info['origin']  # [x, y, yaw]
        
        # PGM 이미지 읽기 (흑백)
        img = Image.open(pgm_path)
        img = img.convert('L')
        img_data = np.array(img)
        img_data = np.flipud(np.fliplr(img_data))
        img_data = np.fliplr(img_data)

        # OccupancyGrid 메시지 초기화
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = resolution
        grid_msg.info.width = img_data.shape[1]
        grid_msg.info.height = img_data.shape[0]
        grid_msg.info.origin.position.x = origin[0]
        grid_msg.info.origin.position.y = origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # 여기서 도로까지 모두 장애물 처리
        # img_data는 0~255 밝기인데, 맵에서 도로는 밝은 부분(255), 벽은 어두운 부분(0) 보통임
        # 따라서 전체 맵 데이터를 모두 장애물(100)로 강제 설정

        # 모든 셀을 100으로 하여 맵 전체 장애물 처리
        data = []
        for row in img_data:
            for pixel in row:
                if pixel > 250:
                    data.append(0)     # 도로
                else:
                    data.append(100)   # 도로 외부
        grid_msg.data = data           # <-- 이 줄은 반복문 밖!
        self.occupancy_grid = grid_msg
        self.map_publisher.publish(grid_msg)
        
    def marker(self):#rivz마커

        marker_array=MarkerArray()
        # 노드 Marker (POINTS 타입)
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

        # 엣지 Marker (LINE_LIST 타입)
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
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        car_x, car_y, car_yaw = self.current_pose
        #grid 생성
        grid_msg = self.occupancy_grid
        # scan_msg
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)
        # print(start_scan_idx,end_scan_idx)

        for i in range(len(ranges)):
            #차량기준좌표
            dist = ranges[i]
            if np.isinf(dist) or np.isnan(dist):
                continue  
            angle = angle_min+angle_increment*i
            obs_x = dist*np.cos(angle)
            obs_y = dist*np.sin(angle)

            #월드좌표로 변경
            world_obs_x = car_x + np.cos(car_yaw) * obs_x - np.sin(car_yaw) * obs_y
            world_obs_y = car_y + np.sin(car_yaw) * obs_x + np.cos(car_yaw) * obs_y

            #그리드기준좌표 변경
            grid_obs_x = int((world_obs_x - grid_msg.info.origin.position.x) / grid_msg.info.resolution)
            grid_obs_y = int((world_obs_y - grid_msg.info.origin.position.y) / grid_msg.info.resolution)

            if 0 <= grid_obs_x < grid_msg.info.width and 0 <= grid_obs_y < grid_msg.info.height:
                idx = grid_obs_y*grid_msg.info.width+grid_obs_x
                grid_msg.data[idx] = 100
        #그리드만들기
        self.map_publisher.publish(grid_msg)

    def odom_callback(self, odom_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        #현재 x,y,yaw 저장
        x=odom_msg.pose.pose.position.x
        y=odom_msg.pose.pose.position.y
        orientation = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation)
        self.current_pose = np.array([x,y,yaw])
        print(self.current_pose)

    
        #트리에 시작점 추가
        if len(self.tree)==0:
            root = TREENode()
            root.x=x
            root.y=y
            root.is_root = True
            root.parent = None
            root.cost = 0
            self.tree.append(root)

        goal_x, goal_y = 4.0, 8.0

        sampled_point = self.sample()
        if sampled_point is not None:
            nearest_idx = self.nearest(self.tree, sampled_point)
            nearest_node = self.tree[nearest_idx]
            new_node = self.steer(nearest_node, sampled_point)
            if not self.check_collision(nearest_node, new_node):
                self.tree.append(new_node)
                if self.is_goal(new_node, goal_x, goal_y):
                    self.path = self.find_path(self.tree, new_node)

        self.marker()

        # 경로 따라 주행 제어 (path가 있으면)
        if hasattr(self, 'path') and len(self.path) > 1:
            # 현재 위치, 각도
            car_x, car_y, car_yaw = self.current_pose

            # 다음 목표 노드 (현재 위치 기준 가장 가까운 경로 노드 뒤의 노드)
            # 현재 위치에 가장 가까운 경로 노드 인덱스 찾기
            dists = [np.hypot(node.x - car_x, node.y - car_y) for node in self.path]
            closest_idx = np.argmin(dists)
            target_idx = min(closest_idx + 1, len(self.path) - 1)
            target_node = self.path[target_idx]

            # 목표점 방향 각도
            path_dx = target_node.x - car_x
            path_dy = target_node.y - car_y
            path_yaw = np.arctan2(path_dy, path_dx)

            # 각도 차이 ([-pi, pi])
            angle_diff = path_yaw - car_yaw
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi

            # 간단한 비례 제어기 (스티어링 각도)
            max_steering_angle = 0.34  # 예시 최대 조향각도 (rad)
            steering_angle = max(-max_steering_angle, min(max_steering_angle, angle_diff))

            # 속도 (고정값 or 거리차에 따라 조절 가능)
            speed = 1.0  # m/s

            # AckermannDriveStamped 메시지 작성 및 퍼블리시
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = "base_link"
            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = steering_angle

            self.drive_publisher.publish(drive_msg)

        return None

    def sample(self):#월드기준 샘플점
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        car_x, car_y, car_yaw = self.current_pose

        x_min=self.occupancy_grid.info.origin.position.x
        y_min=self.occupancy_grid.info.origin.position.y
        x_max = x_min + self.occupancy_grid.info.width * self.occupancy_grid.info.resolution
        y_max = y_min + self.occupancy_grid.info.height * self.occupancy_grid.info.resolution
        

        for _ in range(100):
            #랜덤한 점 가져오기
            ran_x=random.uniform(x_min,x_max)
            ran_y=random.uniform(y_min,y_max)
            
            dx=ran_x-car_x
            dy=ran_y-car_y

            #전방의 점만 가져오기
            forward=np.cos(car_yaw) * dx + np.sin(car_yaw)*dy

            if forward>0:
                grid_x = int((ran_x - x_min) / self.occupancy_grid.info.resolution)
                grid_y = int((ran_y - y_min) / self.occupancy_grid.info.resolution)
                index = grid_y * self.occupancy_grid.info.width + grid_x
                #월드좌표
                if self.occupancy_grid.data[index] == 0:
                    # print(ran_x, ran_y)
                    return (ran_x, ran_y)
                    
        return None

    def nearest(self, tree, sampled_point):
        """
        이 메서드는 샘플링된 포인트에 가장 가까운 트리상의 노드를 반환해야 합니다.

        인자:
            tree ([]): 현재 RRT 트리
            sampled_point (tuple of (float, float)): 자유 공간에서 샘플링된 포인트

        반환값:
            nearest_node (int): 트리에서 샘플링된 포인트와 가장 가까운 노드의 인덱스
        """
        min_dist = float('inf')
        sample_x,sample_y = sampled_point #월드기준 점
        for i in range(len(tree)): #트리노드 순회
            node = tree[i] 
            dist = np.sqrt((node.x - sample_x)**2+(node.y - sample_y)**2)#거리재봄
            if dist<min_dist:
                min_dist=dist
                nearest_idx = i
        return nearest_idx

    def steer(self, nearest_node, sampled_point):
        """
        이 메서드는 샘플링된 점보다 nearest_node에 더 가까운 지점을
        탐색 가능한 공간 안에서 반환해야 합니다.

        인자:

        nearest_node (Node): 트리에서 샘플링된 점에 가장 가까운 노드

        sampled_point (tuple of (float, float)): 샘플링된 좌표 (x, y)

        반환값:

        new_node (Node): steering 과정을 통해 생성된 새로운 노드
        """
        near_x=nearest_node.x
        near_y=nearest_node.y
        sample_x,sample_y = sampled_point
        dx = sample_x - near_x
        dy = sample_y - near_y
        yaw = np.arctan2(dy,dx)
        dist = np.sqrt(dx**2+dy**2)
        max_dist = 1.0

        new_x = near_x + min(max_dist,dist) * np.cos(yaw)
        new_y = near_y + min(max_dist,dist) * np.sin(yaw)

        new_node = TREENode(x=new_x,y=new_y,parent=nearest_node)

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        이 메서드는 nearest 노드와 new_node 사이의 경로가 충돌이 없는지 반환해야 합니다.

    인자:
        nearest (Node): 트리에서 가장 가까운 노드
        new_node (Node): 조향(steering)으로 생성된 새 노드

    반환값:
        collision (bool): 두 노드 사이 경로가 Occupancy Grid와 충돌하는지 여부
        """
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
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        dx=goal_x - latest_added_node.x
        dy=goal_y - latest_added_node.y
        dist = np.sqrt(dx**2 + dy**2)
        if dist <2.0:
            return True
        return False

    def find_path(self, tree, latest_added_node):
        """
        이 메서드는 최신에 추가된 노드가 목표에 충분히 가까워졌을 때,
        시작점에서 목표까지 연결된 노드들의 리스트로 경로를 반환합니다.

        인자:
            tree ([]): 노드들의 리스트로 구성된 현재 트리
            latest_added_node (Node): 트리에 최신으로 추가된 노드

        반환값:
            path ([]): 유효한 경로로서의 노드 리스트
            """
        path =[]
        tree.append(latest_added_node)

        node = latest_added_node
        while node is not None:
            path.append(node)
            node = node.parent

        path.reverse()

        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
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