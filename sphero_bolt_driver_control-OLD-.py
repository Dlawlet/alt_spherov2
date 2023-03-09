#!/usr/bin/env python3

# ROS libraries
import random
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

# Sphero libraries
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

# Sphero utils
import sb_utils

# Other libraries
import math

# Pre-defined global variables

_br_step_count = 1

_b_speed = 0.2
_b_heading = math.pi

_target_max_speed   = min(sb_utils.MAX_BOLT_VEL, max(-sb_utils.MAX_BOLT_VEL, _b_speed))  # Max BOLT speed 1.5 m/s
_target_max_heading = min(sb_utils.MAX_BOLT_HEADING, max(-sb_utils.MAX_BOLT_HEADING, _b_heading)) # Bounding target heading to [-pi, pi] rad

_target_speed   = 0
_target_heading = 0
_matrix_color = Color(0, 0, 0)
_led_color_front = Color(0, 0, 0)
_led_color_back  = Color(0, 0, 0)

_illumniance_msg = Illuminance()
_imu_msg = Imu()
_vector3_msg = Vector3()
_vector3_msg2 = Vector3()
_twist_msg = Twist()

# Data should be moved to a robot state
def update_state():
    # Check if past speed is the same, if so, do not update
    # Heading might require continuous restart
    # Implement updates with a FSM
    return 

# Cleans data after being processed
def clean_input():
    # Check if past heading or past vel is the same, if so, do not update
    return 

# Define callbacks and functions to process incoming messages
def cmd_vel_callback(msg):
    global _target_speed, _target_heading
    _target_speed = ms_to_speed(msg.linear.x)
    _target_heading = rad_to_heading(msg.angular.z)
    return

def led_color_front_callback(msg):
    global _led_color_front
    _led_color_front = Color(round(msg.r), round(msg.g), round(msg.b))
    return

def led_color_back_callback(msg):
    global _led_color_back
    _led_color_back = Color(round(msg.r), round(msg.g), round(msg.b))
    return

def acceleration_callback(msg):
    global _target_heading, _br_step_count, _matrix_color
    if (abs(msg.linear_acceleration.x)>1):
        _target_heading = random.uniform(0, math.pi)
        _br_step_count = 0
        _matrix_color = Color(255, 0, 0)

    return    

# Util methods 

def ms_to_speed(linear_x):
    bound_speed = min(max(0, linear_x), _target_max_speed) 
    sphero_speed = round((bound_speed * 255)/sb_utils.MAX_BOLT_VEL)  
    return sphero_speed 
    
def rad_to_heading(angular_z):
    bound_heading = min(_target_max_heading, max(-_target_max_heading, angular_z))
    sphero_heading = round(math.degrees(bound_heading))
    return sphero_heading
    
# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('sphero_bolt_driver_master')

    # Subscribe to the topics and associate the corresponding callback functions
    sub_target_acceleration = rospy.Subscriber('robot_05/sphero/acceleration', Imu, acceleration_callback)

    # Publish messages
    pub_target_velocity = rospy.Publisher('robot_05/sphero/cmd_vel', Twist, queue_size = 1)
    pub_led_color_front = rospy.Publisher('robot_05/sphero/led_front_rgb', ColorRGBA, queue_size = 1)
    pub_led_color_back = rospy.Publisher('robot_05/sphero/led_back_rgb', ColorRGBA, queue_size = 1)
    pub_matrix_color_back = rospy.Publisher('robot_05/sphero/led_main_rgb', ColorRGBA, queue_size = 1)
    
    rate=rospy.Rate(1)

    while not rospy.is_shutdown():
        
        #Brownian move 
        if (not (_br_step_count %4)):
            _target_heading = random.uniform(0, math.pi)


        _vector3_msg.z = _target_heading
        _twist_msg.angular = _vector3_msg
        _vector3_msg2.x = 1
        _twist_msg.linear  = _vector3_msg2
        pub_target_velocity.publish(_twist_msg) 

        matrix_msg = ColorRGBA()
        matrix_msg.r, matrix_msg.g, matrix_msg.b = _matrix_color[0], _matrix_color[1], _matrix_color[2]
        pub_matrix_color_back.publish(matrix_msg)


        _br_step_count += 1

        #global var reinitialization
        _target_heading = 0
        _matrix_color = Color(0, 0, 0)
        rate.sleep()
