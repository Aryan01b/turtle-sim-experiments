#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Create publisher to control the turtle
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Create subscriber to get the turtle's pose
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        
        # Store current pose
        self.pose = Pose()
        
        # Create a timer that calls the move_turtle function every 0.1 seconds
        self.timer = self.create_timer(0.1, self.move_turtle)
        
        self.get_logger().info('Turtle controller has been started')
    
    def pose_callback(self, msg):
        self.pose = msg
        
    def move_turtle(self):
        # Create a new Twist message
        twist = Twist()
        
        # Simple movement pattern - make the turtle move in a circle
        twist.linear.x = 1.0
        twist.angular.z = 0.5
        
        # Publish the message
        self.publisher_.publish(twist)
        self.get_logger().info(f'Turtle position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, theta={self.pose.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    controller = TurtleController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the turtle before shutting down
        twist = Twist()
        controller.publisher_.publish(twist)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
