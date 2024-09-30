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
        
        # Gjennomsnittig avstand fra veggen
        self.y0 = (30-17.8/2)/2 + 17.8/2 # ca. 19.45 cm
        
        # Konstante verdier
        self.Ku = 0.1
        self.Tu = 2.5
        self.Kp = 0.6 * self.Ku
        self.Ki = 1.2 * self.Ku
        self.Kd = 0.075 * self.Ku * self.Tu
        
        self.I_max = 0.539999999999 # fra testen
        
        # Verdier som skal oppdateres
        self.P = 0 # Propposjonal led
        self.I = 0 # Integral led
        self.D = 0 # Derrivasjons led
        self.e_t = None # avvik i tiden t
        
        
        # Første utkast
        # self.Kp = 0.1
        # self.Ki = 0.07
        # self.integrator = 0
        # self.offset = 0.2
        
    
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))  # is the most used to read LaserScan data and some sensor data.
        
    
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


    def P_ledd(self):
        self.P = self.e_t * self.Kp
        
    def I_ledd(self):
        self.I = self.I + self.e_t * self.Ki
        
        # Begrens I når den er for stor/liten
        if self.I_max < self.I:
            self.I = self.I_max
        elif -self.I_max > self.I:
            self.I = -self.I_max
            
            
    def D_ledd(self):
        
        # Mean filter
        # current mean error
        # previous mean error
        # delta_t 
        pass
            
        
    
        
    def listener_callback(self, laser_msg):
        
        y = min(laser_msg.ranges[:round(len(laser_msg.ranges)/2)]) # distance from the wall
        self.get_logger().info(str(y))
        
        twist_msg = Twist() # pådrag u
        # if min(laser_msg.ranges[:10]) < 0.25:
        #     twist_msg.linear.x = 0.0
        #     twist_msg.angular.z = -2.0
        #     self.get_logger().info("obstacle")
        # else:
        self.e_t = y - self.y0
        # self.e_t = (distance_from_wall*10.0-self.offset)
        # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
        twist_msg.linear.x = 0.05 
        
        # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
        if y < 0.3:
            self.I_ledd()
            self.P_ledd()
            self.D_ledd()
            twist_msg.angular.z = self.P + self.I *  + self.D
            
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