#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped

left_cone_coordinates=[]
right_cone_coordinates=[]

min_distance=10000
min_left_coordinate=[-1,-1]
min_right_coordinate=[-1,-1]

def distance(x,y):
	d=math.sqrt(((x[0]-y[0])**2)+((y[1]-y[1])**2))
	return d

def call(data):
	global car_coordinate 
	car_coordinate=[0,0]
	car_coordinate[0]=data.pose.position.x
	car_coordinate[1]=data.pose.position.y


def callback(cones):

	if(len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0):
		reference_x=car_coordinate[0]
		reference_y=car_coordinate[1]
	else:
		reference_x=(left_cone_coordinates[-1][0]+right_cone_coordinates[-1][0])/2
		reference_y=(left_cone_coordinates[-1][1]+right_cone_coordinates[-1][1])/2
	reference=[reference_x,reference_y]

	if(len(left_cone_coordinates)!=0 and len(right_cone_coordinates)!=0):
		for i in cones:
			if(distance(left_cone_coordinates[-1],i)<0.6 or distance(right_cone_coordinates[-1],i)<0.6):
				cones.remove(i)

	for i in range(0,len(cones)):
		for j in range(i+1,len(cones)):
			mid_x=(cones[i][0]+cones[j][0])/2
			mid_y=(cones[i][1]+cones[j][1])/2
			mid=[mid_x,mid_y]
			minimum=distance(reference,mid)
			if(minimum<min_distance):
				min_distance=minimum
				#the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
				#be the position of right cone and cones[j] will ve the position of left cone
				min_left_coordinate=cones[j]
				min_right_coordinate=cones[i]
	
	if(distance(reference,car_coordinate)<0.4):
		left_cone_coordinates.append(min_left_coordinate)
		right_cone_coordinates.append(min_right_coordinate)
		min_distance=10000
		min_left_coordinate=[-1,-1]
		min_right_coordinate=[-1,-1]



if __name__ == '__main__':
	print("SLAM")
	rospy.init_node('slam',anonymous = True)
	rospy.Subscriber("/perception_to_slam",LaserScan,callback)
	rospy.Subscriber("/gt_pose",PoseStamped, call)
	rospy.spin()