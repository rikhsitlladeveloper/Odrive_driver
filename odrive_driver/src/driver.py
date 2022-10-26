#!/usr/bin/env python3
import odrive
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Odrive_Driver():

    def __init__(self) -> None:
        rospy.loginfo("Finding an odrive...")
        self.my_drive = odrive.find_any()
        rospy.loginfo("Succesfully found")
        rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
        #self.odometry_publisher = rospy.Publisher("odometry", Odometry)
        self.wheelbase = 0.5
        self.radius = 0.1651 / 2        


    def cmd_callback(self, msg):
        self.my_drive.axis0.controller.input_vel = self.calculate_right_speed(msg.linear.x, msg.angular.z)   # turn/s
        self.my_drive.axis1.controller.input_vel = self.calculate_left_speed(msg.linear.x, msg.angular.z)    # turn/s
    
    def calculate_right_speed(self, x, z):
        return (2 * x + z * self.wheelbase) / (2 * self.radius * 2 * 3.14) 

    def calculate_left_speed(self, x, z):
        return (2 * x - z * self.wheelbase) / (2 * self.radius * 2 * 3.14) 

    def odometry_calculation(self):
        init_pos = self.my_drive.axis0.encoder.shadow_count
        print(init_pos)
if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('Odrive_Driver_Node', anonymous=True)
    # Calling Class
    Odrive = Odrive_Driver()
    #while True:
    #    Odrive.odometry_calculation()
    #    rospy.spin()
    #rate control
    rospy.spin()














