#!/usr/bin/env python3
import odrive
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool, Empty,SetBoolResponse
import time
import math
from odrive_driver.msg import Channel_values, Status

class Odrive_Driver():

    def __init__(self) -> None:
        rospy.loginfo("Finding an odrive...")
        self.my_drive = odrive.find_any()
        rospy.loginfo("Succesfully found")
        self.wheelbase = rospy.get_param('wheelbase', default = 0.365)
        self.radius = rospy.get_param('wheel_radius', default = 0.085)
        self.max_rpm = rospy.get_param('max_rpm', default = 260)
        self.rate = rospy.get_param('frequency', default = 20)
        rospy.Service('Odrive_reboot', Empty, handler=self.reboot)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
        self.shadow_counts = rospy.Publisher('shadow_counts', Channel_values, queue_size= 10)
        self.status_pub = rospy.Publisher('status', Status, queue_size= 10)
        self.counts = Channel_values()
        self.status = Status()
    def driver_status(self):
        """ Encoder ticks publishing"""
        self.counts.right = self.my_drive.axis0.encoder.shadow_count
        self.counts.left = - self.my_drive.axis1.encoder.shadow_count 
        self.shadow_counts.publish(self.counts)
    
        """ Driver Status publishing"""
        self.status.right_error = self.my_drive.axis0.error
        self.status.left_error = self.my_drive.axis1.error
        self.status.battery_voltage = self.my_drive.vbus_voltage
        self.status_pub.publish(self.status)
    
    def cmd_callback(self, msg):
        self.my_drive.axis0.controller.input_vel = self.calculate_right_speed(msg.linear.x, msg.angular.z)   # turn/s
        self.my_drive.axis1.controller.input_vel = self.calculate_left_speed(msg.linear.x, msg.angular.z)    # turn/s
        
    def calculate_right_speed(self, x, z):
        speed = (2 * x + z * self.wheelbase) / (2 * self.radius * 2 * math.pi)
        return self.check_speed_limit(speed)

    def calculate_left_speed(self, x, z):
        speed = -(2 * x - z * self.wheelbase) / (2 * self.radius * 2 * math.pi) 
        return self.check_speed_limit(speed)
    
    def check_speed_limit(self, speed):
        if (abs(speed) > (self.max_rpm / 60)):
            speed = (self.max_rpm / 60) * (speed / abs(speed))
        
        return speed
    
    def reboot(self, command):
        
        return "Reboot"
    
if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('Odrive_Driver_Node', anonymous=True)
    rate = rospy.Rate(20)
    # Calling Class
    Odrive = Odrive_Driver()
    while not rospy.is_shutdown():
        Odrive.driver_status()
        rate.sleep()
    rospy.spin()














