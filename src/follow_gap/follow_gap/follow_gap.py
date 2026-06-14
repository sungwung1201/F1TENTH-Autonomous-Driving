import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class FollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('follow_gap')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        # TODO: Publish to drive
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        #추가
        self.bubble_radius = 10

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        max_dist = 3.0

        ranges = np.array(ranges)
        angle_range = np.deg2rad(60)  # 60도 → rad
        angle_min = np.deg2rad(-135)
        angle_increment = np.deg2rad(0.25)

        start_idx = int(( -angle_range - angle_min) / angle_increment)
        end_idx   = int((  angle_range - angle_min) / angle_increment)

        ranges =ranges[start_idx:end_idx]



        # NaN 또는 inf 값을 max_distance로 대체
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), max_dist, ranges)
        # max_distance보다 큰 값은 잘라냄
        ranges[ranges > max_dist] = max_dist

        #sliding filter
        kernel_size = 5
        kernel = np.ones(kernel_size) / kernel_size
        padded = np.pad(ranges, (kernel_size//2, kernel_size//2), mode='edge')
        ranges = np.convolve(padded, kernel, mode='valid')

        proc_ranges = ranges
        return proc_ranges

    def set_bubble(self, ranges, closest_index):
        ranges = ranges.copy()
        start = max(0, closest_index - self.bubble_radius)
        end = min(len(ranges), closest_index + self.bubble_radius)
        ranges[start:end] = 0.0
        proc_ranges = ranges
        self.get_logger().info(f"Bubble set from {start} to {end}")
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_i, end_i = 0, 0
        max_gap = 0
        start = None

        # 1 이상의 값만을 고려하도록 필터링
        # free_space_ranges = np.where(free_space_ranges >= 1.0, free_space_ranges, 0)


        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0:
                if start is None:
                    start = i
            else:
                if start is not None:
                    gap = i - start
                    if gap > max_gap:
                        max_gap = gap
                        start_i = start
                        end_i = i
                    start = None

        # 마지막까지 열린 구간 처리
        if start is not None:
            gap = len(free_space_ranges) - start
            if gap > max_gap:
                start_i = start
                end_i = len(free_space_ranges)
        print(start_i , end_i)
        return start_i, end_i
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        best_index = int((start_i + end_i)/2)  # 중간값 선택
        return best_index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        nonzero_indices = np.where(proc_ranges > 0)[0]
        if len(nonzero_indices) > 0:
            closest_index = nonzero_indices[np.argmin(proc_ranges[nonzero_indices])]
        else:
            closest_index = len(proc_ranges) // 2  # 예외 처리
        #버블만들기
        proc_ranges = self.set_bubble(proc_ranges, closest_index)
        #Find max length gap 
        start_i, end_i = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        best_index = self.find_best_point(start_i, end_i, proc_ranges)
        angle = -np.deg2rad(60)+best_index * data.angle_increment
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        speed = drive_msg.drive.speed   # 속도 설정
        drive_msg.drive.steering_angle = float(angle)
        angle_deg = np.rad2deg(angle)
        if 0 <= abs(angle_deg) <= 10 :
            drive_msg.drive.speed = 1.5
        elif 10 < abs(angle_deg) <= 20:
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    follow_gap = FollowGap()
    rclpy.spin(follow_gap)

    follow_gap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()