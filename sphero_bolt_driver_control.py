#!/usr/bin/env python3

"""sphero_bolt_driver_node.py: a wrapper to 
operate a Sphero BOLT robot with the Robot 
Operating System (ROS) and the spherov2 
Python library.
"""

### ROS libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

### Sphero libraries
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

### Other libraries
import math
import random

### Credits
__author__     = "David Garzón Ramos"
__copyright__  = "Copyright 2023, IRIDIA - Université libre de Bruxelles"
__credits__    = ["David Garzón Ramos", "Florian Noussa Yao", "Mauro Birattari"]
__license__    = "MIT"
__version__    = "0.1"
__maintainer__ = "David Garzón Ramos"
__email__      = "david.garzon.ramos@ulb.be"
__status__     = "Prototype"

class SpheroBolt():

    ### Constants

    PERIOD_PUBLISH_CALLBACK: float = 0.1
    PERIOD_CONTORL_CALLBACK: float = 0.25
    MAX_BOLT_SPEED = 1.5 # m/s
    MAX_BOLT_HEADING = math.pi # rad

    ### Init

    def __init__(self) -> None:

        # init ROS node
        rospy.init_node("sphero_bolt_driver_node")

        self.target_speed   = 0
        self.target_heading   = 0
        self.led_color_front  = Color(0, 0, 0)
        self.led_color_back   = Color(0, 0, 0)
        self.br_step_count = 0
        self.matrix_color = Color(0, 0, 0)

        self.setup_sphero_parameters()

    ### Setup BOLT 

    def setup_sphero_parameters(self) -> None: 

        self.boundary_speed     = float(rospy.get_param('speed_limit', 0.2))
        self.target_max_speed   = min(self.MAX_BOLT_SPEED, 
                                 max(-self.MAX_BOLT_SPEED, self.boundary_speed))  
        
        self.boundary_heading   = float(rospy.get_param('turning_limit', math.pi/2))
        self.target_max_heading = min(self.MAX_BOLT_HEADING, 
                                 max(-self.MAX_BOLT_HEADING, self.boundary_heading))
        
        init_led_front_rgb = rospy.get_param('led_front_rgb', [0, 0, 0])
        self.led_front_rgb = Color(init_led_front_rgb[0], init_led_front_rgb[1], init_led_front_rgb[2])

        init_led_back_rgb  = rospy.get_param('led_back_rgb', [0, 0, 0])
        self.led_back_rgb  = Color(init_led_back_rgb[0], init_led_back_rgb[1], init_led_back_rgb[2])

        self.set_sphero_leds()

        self.create_ros_publishers()
        self.create_ros_subscribers()

        # Publishing timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_PUBLISH_CALLBACK), self.publisher_callback)

        # Control update timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_CONTORL_CALLBACK), self.control_loop_callback)

    ### Create subscribers

    def create_ros_subscribers(self) -> None:

        # Velocity 
        rospy.Subscriber('sphero/cmd_vel', Twist, 
                         self.cmd_vel_callback, queue_size=1)
        # Front LED
        rospy.Subscriber('sphero/led_front_rgb', ColorRGBA, 
                         self.led_front_rgb_callback, queue_size=1)      
        # Back LED 
        rospy.Subscriber('sphero/led_back_rgb', ColorRGBA,
                         self.led_back_rgb_callback, queue_size=1)
        
        #acceleration
        rospy.Subscriber('sphero/acceleration', Imu,
                          self.acceleration_callback, queue_size=1)

    ### Subscribers

    def cmd_vel_callback(self, msg) -> None:
        self.target_speed = self.ms_to_speed(msg.linear.x)
        self.target_heading = self.rad_to_heading(msg.angular.z)

    def led_front_rgb_callback(self, msg) -> None:
        self.led_front_rgb = Color(round(msg.r), round(msg.g), round(msg.b))

    def led_back_rgb_callback(self, msg) -> None:
        self.led_back_rgb = Color(round(msg.r), round(msg.g), round(msg.b))

    def acceleration_callback(self,msg):
        if (abs(msg.linear_acceleration.x)>1):
            self.target_heading = random.uniform(0, math.pi)
            self.br_step_count = 0
            self.matrix_color = Color(255, 255, 0)

    ### Create publishers

    def create_ros_publishers(self) -> None:

        # Ambient light
        self.pub_illuminance = rospy.Publisher('sphero/illuminance', 
                                               Illuminance, queue_size=1)
        self.pub_target_velocity = rospy.Publisher('sphero/cmd_vel', 
                                                Twist, queue_size = 1)
        self.pub_led_color_front = rospy.Publisher('sphero/led_color_front', 
                                                ColorRGBA, queue_size = 1)
        self.pub_led_color_back = rospy.Publisher('sphero/led_color_back',
                                                 ColorRGBA, queue_size = 1)
    
        
    ### Publishers

    def publish_illuminance(self):
        illumniance_msg = Illuminance()
        illumniance_msg.header.stamp = rospy.Time.now()
        illumniance_msg.illuminance = droid.get_luminosity()['ambient_light']
        self.pub_illuminance.publish(illumniance_msg)

    def publisher_callback(self, event=None):
        self.publish_illuminance()   
    
    ### Controls 

    def brownian_move(self):
        if (not (_br_step_count %4)):
            self.target_heading = random.uniform(0, math.pi)


        self.vector3_msg.z = _self.target_heading
        self.twist_msg.angular = _self.vector3_msg
        self.vector3_msg2.x = 0.2
        self.twist_msg.linear  = _self.vector3_msg2
        self.pub_target_velocity.publish(_twist_msg) 
        
        _br_step_count += 1

    ### Utils

    def ms_to_speed(self, linear_x):
        bounded_speed = min(max(0, linear_x), self.target_max_speed) 
        sphero_speed = round((bounded_speed * 255)/self.MAX_BOLT_SPEED)  
        return sphero_speed 
        
    def rad_to_heading(self, angular_z):
        bounded_heading = min(self.target_max_heading, max(-self.target_max_heading, angular_z))
        sphero_heading = (round(math.degrees(bounded_heading)) + 360) % 360
        return sphero_heading

### Main             
        
if __name__ == "__main__":

    
