import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
#import numpy as np
from geometry_msgs.msg import Twist


class SimpleSubscriber(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('simple_subscriber')
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))  # is the most used to read LaserScan data and some sensor data.
        
    
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    integrator = 0
    
    def listener_callback(self, laser_msg):
        Kp = 0.1
        Ki = 0.07
        offset = 2.0
        # denne koden løser oppgaven
        distance_from_wall = min(laser_msg.ranges[:round(len(laser_msg.ranges)/2)])
        self.get_logger().info(str(distance_from_wall))
        
        twist_msg = Twist()
        if min(laser_msg.ranges[:10]) < 0.25:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -2.0
            self.get_logger().info("obstacle")
        else:
            # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
            twist_msg.linear.x = 0.05
            # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
            if distance_from_wall < 0.4:
                self.integrator = self.integrator + (distance_from_wall*10.0-offset) * Ki
                if 0.5 < self.integrator:
                    self.integrator = 0.5
                elif -0.5 > self.integrator:
                    self.integrator = -0.5
                    
                twist_msg.angular.z = (distance_from_wall*10.0-offset) * Kp + self.integrator
            elif distance_from_wall:
                twist_msg.angular.z = (0.4*10.0-offset) * Kp
            
        # Publish the message to the Topic
        self.publisher_.publish(twist_msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % twist_msg)
        
        #pass
        


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_subscriber = SimpleSubscriber()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_subscriber)
    # Explicity destroy the node
    simple_subscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()