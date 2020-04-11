#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('robot_mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    ##### get input
    print("let's make this turtlebot move")
    speeds=[[10,20,30,2 ],[20,30,30,2],[20,20,30,2],[10,10,30,2]]
    for speed in speeds:
        vel_msg.linear.x = abs(speed[0])
        vel_msg.linear.y = abs(speed[1])
        vel_msg.linear.z = 0
        vel_msg.angular.x = abs(speed[2])
        vel_msg.angular.y = 0
        distance=abs(speed[3])       
        while not rospy.is_shutdown():
           t0 = rospy.Time.now().to_sec()
           total_time = 0
    
           #Loop to move the turtle in an specified distance
           while(t0==total_time):
               #Publish the velocity
               velocity_publisher.publish(vel_msg)
               #Takes actual time to velocity calculus
               t1=rospy.Time.now().to_sec()
               #Calculates distancePoseStamped
               total_time=distance/math.sqrt(speed[0]^2 + speed[1]^2)
           #After the loop, stops the robot
           vel_msg.linear.x = 0
           #Force the robot to stop
           velocity_publisher.publish(vel_msg) vel_msg.angular.z = 0

    rospy.spin()

