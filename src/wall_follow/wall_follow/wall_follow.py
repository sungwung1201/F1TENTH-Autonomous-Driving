import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    차량에서 벽 추적을 구현합니다.
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan' 
        drive_topic = '/drive'

        # TODO: 구독자와 발행자 생성
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
        # TODO: PID 이득 설정
        self.kp = 1 # 파라미터
        self.kd = 0.001
        self.ki = 0.005

        # TODO: 기록 저장
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: 필요한 다른 값들을 저장
        
        
    def get_range(self, range_data, angle):
        """
        주어진 각도에서 해당하는 거리 측정을 반환하는 간단한 도우미 함수입니다.
        NaN과 무한대 값을 처리하는 것을 확실히 하세요.

        Args:
            range_data: LiDAR의 단일 거리 배열
            angle: LiDAR의 angle_min과 angle_max 사이의 각도 (degrees)

        Returns:
            range: 주어진 각도에서의 거리 측정값 (미터 단위)
        """
        
        angle_increment = range_data.angle_increment
        ranges = np.array(range_data.ranges)

        idx = int((np.deg2rad(angle) - range_data.angle_min)/angle_increment)

        if idx <0 or idx >=len(ranges):
            return 3.0

        distance = ranges[idx]
        #print(f"{angle}, {distance}")
        if np.isnan(distance) or np.isinf(distance) or distance > 10:
            return 10.0
        #print(f"{angle}, {distance}")
        #TODO: 구현
        return distance

    def get_error(self, range_data, dist):
        """
        벽과의 오차를 계산합니다. 왼쪽 벽을 따라 (Levine 루프에서 반시계 방향으로) 추적합니다.
        get_range()를 사용할 수 있습니다.

        Args:
            range_data: LiDAR의 단일 거리 배열
            dist: 벽과의 원하는 거리

        Returns:
            error: 계산된 오차
        """
        #e(t)=-(y+L*sin(theta)) #theta:70 y는 벽과의 떨어진 거리 1로놓자
        a=self.get_range(range_data,35)
        b=self.get_range(range_data,90)
        # print("\n",a)
        # print(b)
        L=1
        #TODO: 구현
        alpha = np.arctan((a * np.cos(np.deg2rad(55)) - b) / (a * np.sin(np.deg2rad(55))))
        wall_dist = b * np.cos(alpha)
        after_dist = wall_dist + L*np.sin(alpha)
        error = dist - after_dist
        # print("alpha:",alpha)
        
        # print("walldist :",wall_dist)
        # print("afterdist :",after_dist)
        # print("error :",error)
        return error

    def pid_control(self, error, velocity):
        """
        계산된 오차를 바탕으로 차량 제어를 발행합니다.

        Args:
            error: 계산된 오차
            velocity: 원하는 속도

        Returns:
            None
        """
        # TODO: kp, ki & kd를 사용하여 PID 제어기를 구현
        dt=0.01
        self.integral += self.prev_error * dt
        derivative = (error - self.prev_error) / dt
        angle = self.kp*error + self.kd*derivative + self.ki*self.integral
        self.prev_error = error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        print(angle)
        
        # if 0 <= angle <= 10:
        #     drive_msg.drive.speed = 1.5
        # elif 10 < angle <= 20:
        #     drive_msg.drive.speed = 1.0
        # else:
        #     drive_msg.drive.speed = 0.5

        
        # TODO: drive 메시지를 채우고 발행
        self.drive_publisher.publish(drive_msg)
        
        

    def scan_callback(self, msg):
        """
        LaserScan 메시지에 대한 콜백 함수입니다. 오차를 계산하고 이 함수에서 drive 메시지를 발행합니다.

        Args:
            msg: 수신된 LaserScan 메시지

        Returns:
            None
        """
        error = self.get_error(msg, 0.75) # TODO: get_error()로 계산된 오차로 교체
        angle = self.kp * error + self.kd * ((error - self.prev_error) / 0.01) + self.ki * self.integral
        print(angle)
        # 조향각을 기반으로 속도 조절
        if 0 < abs(angle) <= np.deg2rad(10):
            velocity = 1.5
        elif np.deg2rad(10) < abs(angle) <= np.deg2rad(20):
            velocity = 1.0
        else:
            velocity = 0.5
        #TODO: 오차를 기반으로 원하는 차량 속도를 계산
        self.pid_control(error, velocity) # TODO: PID로 차량 제어

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # 노드를 명시적으로 제거
    # (선택 사항 - 그렇지 않으면 가비지 컬렉터가 자동으로 노드를 제거할 것입니다)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
