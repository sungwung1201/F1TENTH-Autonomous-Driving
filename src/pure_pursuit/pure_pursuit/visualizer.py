import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('visualizer')

        self.publisher = self.create_publisher(Marker, '/waypoints_marker', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # CSV 파일에서 웨이포인트 불러오기 (x, y, theta, speed)
        self.waypoints = np.loadtxt('/home/yoon/sim_ws/log/wp-2025-05-14-09-31-38.csv', delimiter=',')

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"  # 또는 "odom", 사용 환경에 맞게
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # 선 굵기
        marker.scale.y = 0.2
        marker.color.r = 0.3
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0

        for wp in self.waypoints:
            pt = Point()
            pt.x = wp[0]
            pt.y = wp[1]
            pt.z = 0.0
            marker.points.append(pt)

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    visualizer = WaypointVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
