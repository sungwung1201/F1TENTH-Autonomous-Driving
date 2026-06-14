#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

import numpy as np
# TODO: 필요한 ROS 메시지 타입 헤더와 라이브러리 포함
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    """
    비상 제동을 처리하는 클래스입니다.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        하나의 퍼블리셔가 /drive 토픽에 AckermannDriveStamped 드라이브 메시지를 퍼블리시해야 합니다.

        또한 /scan 토픽을 구독하여 LaserScan 메시지를 받고,
        /ego_racecar/odom 토픽을 구독하여 차량의 현재 속도를 얻어야 합니다.

        구독자들은 제공된 odom_callback과 scan_callback 메서드를 콜백 함수로 사용해야 합니다.

        참고: odom의 linear velocity의 x 구성 요소는 차량의 속도를 나타냅니다.
        """
        
        self.speed = 0.  # 현재 속도 초기화

        # TODO: ROS 구독자와 퍼블리셔 생성
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        

    def odom_callback(self, odom_msg):
        # TODO: 현재 속도 업데이트
        self.speed = odom_msg.twist.twist.linear.x#앞으로 가는 속도

    def scan_callback(self, scan_msg):
        # TODO: TTC(충돌 시간) 계산
	    
        scan_object = min(scan_msg.ranges)#스캔 할 때의 빔중 최소 거리
        arr = np.array(scan_msg.ranges)#빔의 배열
        range_index = np.argmin(arr)#빔의 index
        if self.speed != float('inf'):
            if self.speed !=0:
                angle_max=scan_msg.angle_max#스캔 하는 최대 각도
                angle_min=scan_msg.angle_min#스캔 하는 최소 각도
                angle_increment=scan_msg.angle_increment#스캔 사이의 각도
                #num_of_beams = int((angle_max - angle_min) / angle_increment) +1#빔의갯수
		
                TTC_R=self.speed*math.cos(angle_min + (range_index * angle_increment))
                if TTC_R != 0 :
                    TTC = scan_object / TTC_R
                    if -1 < TTC < 1:
                        brake = AckermannDriveStamped()
                        brake.drive.speed = 0.0
                        self.drive_publisher.publish(brake)
            
        #r=self.speed*cos(angle_min + (i * angle_increment))
        #angle_max - angle_min = 전체각도 / angle_increment +1

        #num of beams == (angle_max - angle_min)/angle_increment == num of ranges
        # 제일 가까이 있는 걸 기준으로 TTC 계산해야 함
        # 가장 가까이 있는 range의 index를 알면 angle_increment에 곱할 i도 나옴.
        # scan_msg.index(ranges)
        #-----------------
        #numpy를 써서 할 경우의 코딩
        #num_of_beams = ((angle_max - angle_min) / angle_increment) +1
        #range_index_arr = np.num_of_beams

        # TODO: 제동 명령 퍼블리시
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # 노드를 명시적으로 소멸시킵니다.
    # (선택사항 - 그렇지 않으면 가비지 컬렉터가 노드 객체를 파괴할 때 자동으로 수행됩니다)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
