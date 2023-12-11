#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 0.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.(1.5)(0.5)
desired_distance = 0.6	# distance from the wall (in m). (defaults to right wall)(0.9)
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
c=0.6

leftcone=[[-2.3,0],[-2.4,4],[-2.3,7.9],[-2.3,11.9],[-2.3,15.6],[-2.3,19.8],[-2.3,23.8],[-2.3,27.8],[-2.3,32.1],[-2.3,37.1],[-1,43.3],[7.5,44.4],[14.2,45],[20,41.3],[20.7,35],[20.7,29.9],[20.9,24.6],[20.7,20],[20.9,15.9],[20.6,12.2],[20.9,7.7],[20.8,4],[20.8,1],[20.47,-2.65],[19.5,-5.4],[13.7,-6.4],[9.4,-6.3],[5.6,-6.2],[1.9,-6.2],[-2.3,-4.2]]
rightcone=[[2.3,0],[1.9,4.1],[2.1,7.8],[2.1,12.1],[2.1,15.7],[2.1,19.6],[1.9,23.7],[2.3,27.5],[2.3,32],[2.2,36.2],[3,40],[7.9,40.4],[13.5,40.4],[15.3,37.2],[16.9,34.9],[16.1,29.5],[16.2,24.2],[15.9,20],[16,15.6],[16,11.7],[16.1,7.5],[16.1,4],[15.8,0.7],[16,-2.5],[16,-2.5],[13.8,-2.7],[9.1,-2.8],[5,-2.8],[2.1,-2.7],[2.1,-2.7]]

car_coordinate=[0,0]


# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('/err', pid_input, queue_size=10)
'''
def call(data):
	print(len(data.ranges))
	size=len(data.ranges)/360
	for i in range(272,818):
		#print("degree= ",i*0.33," dist= ",data.ranges[i])
'''
	
	
	

def callback(data):

	
	car_coordinate[0]=data.pose.position.x
	car_coordinate[1]=data.pose.position.y

	global roll, pitch, yaw
	orientation_q = data.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

	"""
	print(car_cordinate[0])
	print(car_cordinate[1])
	print(roll)
	print(pitch)
	print(math.degrees(yaw))
	"""
	print("--------------------")

	midpoints=[]
	
	for i in range(2):
		mid=[]
		min_left= leftcone[i]
		min_right= rightcone[i]
		mid.append((min_left[0]+min_right[0])/2)
		mid.append((min_left[1]+min_right[1])/2)
		midpoints.append(mid)
	
	for i in range(2):
		for j in range(2):
			print(midpoints[i][j])	
	print("yaw=",math.degrees(yaw))


	num=midpoints[1][1]-midpoints[0][1]
	den=(midpoints[1][0]-midpoints[0][0])
	
	
	'''
	slope= abs(num/den)
	ideal_orientation=math.atan(slope)
	'''
	dist_midpoints=abs(math.sqrt(num**2 + den**2))
	ideal=math.acos(den/dist_midpoints)
	
	val=midpoints[1][1]-midpoints[0][1]
	if(val<0):
		ideal=ideal*(-1)
	print("ideal= ",math.degrees(ideal))
	


	A=midpoints[0][1]-midpoints[1][1]
	B=midpoints[1][0]-midpoints[0][0]
	C=(midpoints[0][0]*midpoints[1][1])-(midpoints[1][0]*midpoints[0][1])
	err=(abs((A*car_coordinate[0])+(B*car_coordinate[1]+C))/math.sqrt((A**2)+(B**2)))
	print("err=",err)

	dist_car_left= math.sqrt(((leftcone[1][0]-car_coordinate[0])**2)+((leftcone[1][1]-car_coordinate[1])**2))
	dist_car_right= math.sqrt(((rightcone[1][0]-car_coordinate[0])**2)+((rightcone[1][1]-car_coordinate[1])**2))
	
    
	Alpha= yaw-ideal
	print("alpha=",math.degrees(Alpha))
	if(dist_car_left<=dist_car_right):
		error=forward_projection*math.sin(Alpha)+err
	else:
		error=forward_projection*math.sin(Alpha)-err
	print("error=",error)
	



	#(dist_car_left+dist_car_right)>1.71 and (dist_car_left+dist_car_right)<1.75
	A1=leftcone[1][1]-rightcone[1][1]
	B1=rightcone[1][0]-leftcone[1][0]
	C1=(leftcone[1][0]*rightcone[1][1])-(rightcone[1][0]*leftcone[1][1])
	d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))

	if(d>0 and d<0.2):
		leftcone.pop(0)
		rightcone.pop(0)
		
	'''
	print(dist_car_left+dist_car_right)
	
	print(leftcone)
	'''
	#print(rightcone)
	

	
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error	
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)
	
	


if __name__ == '__main__':
	c=0
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	#rospy.Subscriber("/scan",LaserScan,call)
	rospy.Subscriber("/gt_pose",PoseStamped, callback)
	rospy.spin()