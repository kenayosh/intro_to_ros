#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import random

class TutorialPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher") #Node name
        self.publisher = self.create_publisher(
            Vector3, #Type of message that's boreadcasted
            "/tutorial/vector3", #Topic name
            10
        )
        self.publisher_timer = self.create_timer(
            1.0, self.run_node #Every second, runs the run_node() method
        )
        self.get_logger().info("starting publisher node") #Log in the terminal "starting publisher node"

    def run_node(self):
        msg = Vector3()                         #Initialize Vector3 (the message)
        msg.x = random.uniform(-10.0, 10.0)     #Generate random values for the Vector3
        msg.y = random.uniform(-10.0, 10.0)
        msg.z = random.uniform(-10.0, 10.0) 
        self.publisher.publish(msg)             #publish the message
        self.get_logger().info(f"Vector3\n\tx: {msg.x}\ty: {msg.y}\tz: {msg.z}")
        
    def main(args=None):
        rclpy.init(args=args)           # starts the ROS2 Python3 client
        node = TutorialPublisher()      # creates an instance of the TutorialPublisher class

        try:
            rclpy.spin(node)            # keeps node running until there is an exception
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received, shutting down...")
        finally:
            node.destroy_node()         # destroys node at the end of the program's lifespan
            if rclpy.ok():
                rclpy.shutdown()        # closes the ROS2 Python3 client if it is still active

    if __name__=="__main__":
        main()
        
