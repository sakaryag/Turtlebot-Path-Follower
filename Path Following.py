#!/usr/bin/env python
##GORKEM SAKARYA - 150140063
##
##     I used PD control for following the path and I adapted values with some 
##     values for smoothness. My robot can follow Route 1 - Route 2 and Route 3. 
##	But it is not good for Route 4, maybe it can finished Route 4 but it could 
##	take too much time. 
##
##	**Route 3 is totaly works, if it is too slow in the beginning of pink waypoints,
##	  Please Wait a bit, it will continue :)
##
##	Sometimes it could select wrong way to turn I had not got enough time for correcting it 
##	If it do something wrong please wait again :) I tried Route 1-2 and 3 all of them are finishing.
##
##
##
##  EXTRAS:
##      - Route 3
##      - Proportional Derivative Control
##
##
##	All other comments are given below, Thank you.
##
##
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math
waypoint=None
angle_previous = 0
dist_previous = 0
#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

       
        
	motor_command=Twist()
        
	#GORKEM_COMMENT# calculate error (distance to go)
	distance_x = waypoint.translation.x - translation[0]
	distance_y = waypoint.translation.y - translation[1]

	#GORKEM_COMMENT# calculate rotated waypoint -> error multiplied by rotation matrix
        rotated_x = distance_x * math.cos(robot_theta) + distance_y * math.sin(robot_theta)
	rotated_y = distance_y * math.cos(robot_theta) - distance_x * math.sin(robot_theta)
        
        #GORKEM_COMMENT# arctangent formula for calculating angle
	angle = math.atan2(rotated_y, rotated_x)
	#GORKEM_COMMENT# Euclidian distance for calculating linear x command
        dist = math.sqrt(rotated_x*rotated_x + rotated_y*rotated_y);
	
	#GORKEM_COMMENT# current x- old x calculation for PD Control
	change_linear = dist - dist_previous
	change_angular= angle - angle_previous
	
	#GORKEM_COMMENT# values are adapted for too close position
	if distance_x > 0.0001 or distance_y > 0.0001 :

		#GORKEM_COMMENT# PD Control for linear x
		motor_command.linear.x= 0.5 * dist + 0.02 * change_linear
		
		#GORKEM_COMMENT# values are adapted for too close angles
		if angle>0.01 or angle <-0.01:
			
			#GORKEM_COMMENT# PD Control for angular z
			motor_command.angular.z=0.4 * angle + 0.02 * change_angular
		else:		
			#GORKEM_COMMENT# PD Control for angular z
			motor_command.angular.z=5 * angle + 0.02 * change_angular
	else:
		
		#GORKEM_COMMENT# PD Control for linear x
		motor_command.linear.x= 0.3 * dist + 0.02 * change_linear
		#GORKEM_COMMENT# values are adapted for too close angles
		if angle>0.01 or angle < -0.01:
			#GORKEM_COMMENT# PD Control for angular z			
			motor_command.angular.z=0.4 * angle + 0.02 * change_angular
		else:
			#GORKEM_COMMENT# PD Control for angular z
			motor_command.angular.z=5 * angle + 0.02 * change_angular
	

	
        motor_command_publisher.publish(motor_command)

	#GORKEM_COMMENT# assign current values as old values
        angle_previous = angle
        dist_previous = dist


            

        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
