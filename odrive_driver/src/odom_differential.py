#!/usr/bin/env python3
import odrive
import rospy
from tf import TransformBroadcaster
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from odrive_driver.msg import Channel_values
import math
import time
class Odrive_Odometry():

    def __init__(self) -> None:
        self.publish_tf = rospy.get_param('publish_tf', default = True)
        self.odom_frame = rospy.get_param('odom_frame', default = 'odom')
        self.base_frame = rospy.get_param('base_frame', default = 'base_link')
        self.wheelbase = rospy.get_param('wheelbase', default = 0.365)
        self.radius = rospy.get_param('wheel_radius', default = 0.085)
        self.ppr = rospy.get_param('ppr', default = 1024)
        self.last_time = time.time()
        self.ticks_meter = (self.ppr * 4) / (2 * math.pi * self.radius)
        self.init = False
        self.enc_left = 0
        self.enc_right = 0
        self.x_final = 0
        self.y_final = 0
        self.theta_final = 0

        rospy.Subscriber('shadow_counts', Channel_values, self.encoder_callback)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size = 10)
        
    def encoder_callback(self, msg):
        """ 
        Odometry calculation from encoder tick counts
        recives : (msg.left(Int32) , msg.right(Int32))
        publishes: Odometry topic and TF
        
        """
        self.current_time = time.time() # getting current time in secs
        left = msg.left                 # left encoder tick counts 
        right = msg.right               # right encoder tick counts
        
        # If callback runs initially it takes encoder values
        if (self.init == False):
            self.enc_left = left
            self.enc_right = right
            self.init = True
        
        # Calculation of delta values of motors in meters / dt
        d_left = (left - self.enc_left) / self.ticks_meter
        d_right = (right - self.enc_right) / self.ticks_meter

        # Equalizing encoder values for last encoder value
        self.enc_left = left
        self.enc_right = right

        # Distance and angle passed in dt 
        d = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.wheelbase
        
        # elapsed time or dt
        elapsed = self.current_time - self.last_time
        
        # calculation of velocity 
        dx = d / elapsed
        dr = th / elapsed

        # Odometry displacement calculation
        x = math.cos(th) * d
        y = math.sin(th) * d

        self.x_final = self.x_final + ( math.cos(self.theta_final) * x - math.sin(self.theta_final) * y)
        self.y_final = self.y_final + ( math.sin(self.theta_final) * x + math.cos(self.theta_final) * y)

        self.theta_final = self.theta_final + th
        
        odom_quad = Quaternion()
        odom_quad.x = 0.0
        odom_quad.y = 0.0
        odom_quad.z = math.sin( self.theta_final / 2)
        odom_quad.w = math.cos( self.theta_final / 2)

        if(self.publish_tf):
            odom_trans = TransformStamped()
            odom_broadcaster = TransformBroadcaster()
            odom_trans.header.stamp = rospy.Time.now()
            odom_trans.header.frame_id = self.odom_frame
            odom_trans.child_frame_id = self.base_frame

            odom_trans.transform.translation.x = self.x_final
            odom_trans.transform.translation.y = self.y_final
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quad

            odom_broadcaster.sendTransformMessage(odom_trans)

        odom = Odometry()
        #odom.header.stamp = self.current_time
        odom.header.frame_id = self.odom_frame

        odom.pose.pose.position.x = self.x_final
        odom.pose.pose.position.y = self.y_final
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quad

        odom.child_frame_id = self.base_frame
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dr

        self.odom_pub.publish(odom)

        self.last_time = self.current_time

if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('Odrive_Odometry_Node', anonymous=True)
    # Calling Class
    Odrive = Odrive_Odometry()
    #rate control
    rospy.spin()
