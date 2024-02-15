import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np
import time


class ContainerChaser(Node):
    def __init__(self):
        super().__init__('container_chaser')
        self.subscription_midpoint = self.create_subscription(Point, 'blue_container_midpoint', self.listener_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("desired_midpoint_x", 310)
        self.declare_parameter("desired_midpoint_y", 145)
        self.declare_parameter("max_linear_speed", 0.07)
        self.declare_parameter("max_angular_speed", 0.07)

        # Default values for parameters
        self.desired_midpoint_x = self.get_parameter('desired_midpoint_x').get_parameter_value().integer_value
        self.desired_midpoint_y = self.get_parameter('desired_midpoint_y').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        print("init")

    def listener_callback(self, msg):
        cmd_vel_msg = Twist()
        if not ((self.desired_midpoint_x - 5) <= msg.x <= (self.desired_midpoint_x + 5)):
            cmd_vel_msg.angular.z = -(np.sign(msg.x - self.desired_midpoint_x)) * self.max_angular_speed
        else:
            cmd_vel_msg.angular.z = 0.0

        if not ((self.desired_midpoint_y - 5) <= msg.y <= (self.desired_midpoint_y + 5)):
            cmd_vel_msg.linear.x = (np.sign(msg.y - self.desired_midpoint_y)) * self.max_linear_speed
        else:
            cmd_vel_msg.linear.x = 0.0
            
        if ((self.desired_midpoint_x - 5) <= msg.x <= (self.desired_midpoint_x + 5)) and ((self.desired_midpoint_y - 5) <= msg.y <= (self.desired_midpoint_y + 5)):
            print("end")
            cmd_vel_msg.linear.x = 0.05
            cmd_vel_msg.angular.z = 0.0
            self.publisher_cmd_vel.publish(cmd_vel_msg)
            time.sleep(20)
            cmd_vel_msg.linear.x = 0.0
            self.publisher_cmd_vel.publish(cmd_vel_msg)
            rclpy.shutdown()
        
        print(msg.x)
        print(msg.y)
        print(cmd_vel_msg)
        self.publisher_cmd_vel.publish(cmd_vel_msg)



def main(args=None):
    rclpy.init(args=args)
    container_chaser = ContainerChaser()
    print("start")
    rclpy.spin(container_chaser)
    print("end")
    container_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

