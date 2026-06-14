import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from tf_transformations import euler_from_quaternion
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA

from nav_msgs.msg import Odometry

class MakeWayPoint(Node):

    def __init__(self):
        super().__init__('make_waypoint')

        odom_topic = '/ego_racecar/odom'
        timer_period = 0.5
        self.timer=self.create_timer(timer_period, self.timer_callback)
        self.pose_subscriber = self.create_subscription(
            Odometry,
            odom_topic,
            self.pose_callback,
            10
        )
        home = expanduser("~")
        self.file = open(strftime(home+'/sim_ws/log/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        atexit.register(self.shutdown)
    def shutdown(self):
        self.file.close()
        self.get_logger().info("Waypoint file saved and closed.")

    def pose_callback(self,msg):
        self.wp_msg=msg
    def timer_callback(self):
        if self.wp_msg is None:
            return
        msg=self.wp_msg
        quaternion = np.array([msg.pose.pose.orientation.x, 
                               msg.pose.pose.orientation.y, 
                               msg.pose.pose.orientation.z, 
                               msg.pose.pose.orientation.w])
        # print(quaternion)
        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([msg.twist.twist.linear.x, 
                                  msg.twist.twist.linear.y, 
                                  msg.twist.twist.linear.z]),2)
        if msg.twist.twist.linear.x > 1:
            self.file.write('%f, %f, %f, %f\n' % (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                euler[2],
                speed))



def main(args=None):
    rclpy.init(args=args)
    print("MakeWayPoint Initialized")
    make_waypoint = MakeWayPoint()
    rclpy.spin(make_waypoint)

    make_waypoint.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


