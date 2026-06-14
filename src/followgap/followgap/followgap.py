import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Followgap(Node):
    """ 
    차량에서 벽 추적을 구현합니다.
    이 코드는 템플릿이며, 자유롭게 구현할 수 있습니다!
    """
    def __init__(self):
        super().__init__('follow_gap')
        # 토픽 및 구독자, 발행자 설정
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: LiDAR 데이터를 구독
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )
        # TODO: 차량 제어 메시지를 발행
        drive_msg = AckermannDriveStamped()
        self.drive_publisher.publish(drive_msg)

    def preprocess_lidar(self, ranges):
        """ LiDAR 스캔 배열을 전처리합니다. 고급 구현 방식은 다음과 같습니다:
            1. 각 값을 일정 범위에서 평균값으로 설정
            2. 너무 큰 값 (예: 3m 이상) 제거
        """

        ranges = np.array(ranges)
        if np.isnan(ranges) or np.isinf(ranges) or ranges > 3.0:
            return 3.0
        kernel_size = 5  
        ranges = np.convolve(ranges, np.ones(kernel_size)/kernel_size, mode='same')
        proc_ranges = ranges
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ free_space_ranges에서 가장 긴 간격(빈 공간)의 시작 및 끝 인덱스를 반환합니다.
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """ start_i와 end_i는 가장 긴 간격의 시작 및 끝 인덱스입니다.
            해당 범위 내에서 가장 좋은 포인트의 인덱스를 반환합니다.
	        단순한 방법: 해당 범위 내에서 가장 먼 지점을 선택하고 그곳으로 이동
        """
        return None

    def lidar_callback(self, data):
        """ Follow Gap 알고리즘을 사용하여 각 LiDAR 스캔을 처리하고 
            AckermannDriveStamped 메시지를 발행합니다.
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        # LiDAR에서 가장 가까운 점 찾기

        # '버블' 내부의 모든 포인트 제거 (값을 0으로 설정)

        # 가장 긴 간격 찾기

        # 간격 내에서 최적의 포인트 찾기

        # 차량 제어 메시지 발행


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    follow_gap_node = Followgap()
    rclpy.spin(follow_gap_node)

    follow_gap_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
