#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

def move():
    # Starts a new node
    rospy.init_node('turtlebot', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    vel_msg = Twist()

    #Since we are moving just in xy-axis
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

		points = [ [-4000.000000, -3000.000000], [-3899.902941, -2977.264734], [-3797.887326, -2965.901293], [-3697.790267, -2943.166027], [-3595.774652, -2931.802585], [-3441.014230, -2777.042163], [-3286.253807, -2622.281741], [-3186.156748, -2599.546475], [-3084.141133, -2588.183033], [-2929.380711, -2433.422611], [-2774.620288, -2278.662189], [-2619.859866, -2123.901766], [-2465.099444, -1969.141344], [-2310.339021, -1814.380922], [-2210.241962, -1791.645655], [-2013.273303, -1887.065380], [-1911.257688, -1875.701939], [-1811.160629, -1852.966673], [-1614.191970, -1948.386398], [-1417.223311, -2043.806123], [-1220.254651, -2139.225848], [-1023.285992, -2234.645573], [-826.317333, -2330.065298], [-629.348674, -2425.485023], [-432.380014, -2520.904748], [-378.062906, -2608.002195], [-333.858725, -2700.642920] ]

		thetas = [ 45.000000, 334.152542, 45.000000, 334.152542, 45.000000, 45.000000, 45.000000, 334.152542, 45.000000, 45.000000, 45.000000, 45.000000, 45.000000, 45.000000, 334.152542, 334.152542, 45.000000, 334.152542, 334.152542, 334.152542, 334.152542, 334.152542, 334.152542, 334.152542, 334.152542, 263.305085, 334.152542 ]

		for i,point in enumerate(points):
			if i==0: continue  # skip first point because there is no previous to calculate deltas
			theta = thetas[i]
			prev_point = points[i-1]

			t=0
			dt=0.1
			# First convert distances mm to meters, then take a tenth of that for running ten times
			dx = ((point[0]-prev_point[0])/1000)/10
			dy = ((point[1]-prev_point[1])/1000)/10

			#Setting the current time for distance calculus
			#t0 = rospy.Time.now().to_sec()

			while(t<1):
				vel_msg.linear.x = dx
				vel_msg.linear.y = dy

				#Publish the velocity
				velocity_publisher.publish(vel_msg)
				#Takes actual time to velocity calculus
				#t1=rospy.Time.now().to_sec()

				t+=dt

        #After the loop, stop the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
