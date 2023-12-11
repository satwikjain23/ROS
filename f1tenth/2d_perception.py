#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

coordinates=[]

def final_cone_coodinate(i):
	#print("dddddddddddddddddddddddddddd")
	angle=(i[2]*0.33)-180
	#print("Angle=",angle)
	orientation=math.degrees(yaw)
	#print("orie=",orientation)
	theta=math.radians(angle + orientation)
	#print("cone=",theta)
	length=i[1]+0.47  #radius of cone is 0.94m
	
	#print(math.cos(theta))
	#print(math.sin(theta))
	x_coordinate=car_coordinate[0]+length* math.cos(theta)
	y_coordinate=car_coordinate[1]+length* math.sin(theta)
	c=[x_coordinate,y_coordinate]
	return c


def call(data):
	global car_coordinate 
	car_coordinate=[0,0]
	car_coordinate[0]=data.pose.position.x
	car_coordinate[1]=data.pose.position.y

	global roll, pitch, yaw
	orientation_q = data.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	
	
	

def cone_confirmation(data,cones):
	print("length=",len(cones))
	global final
	final=[]
	for i in cones:
		l=len(i[0])-1
		a=i[0][0]
		b=i[0][l]
		c=i[1]
		#print(car_coordinate,math.degrees(yaw))
		#print("angle of cone=",i[2]*0.33)
		if((a>c)and(b>c)):
			co=final_cone_coodinate(i)
			final.append(co)
	print(final)



def callback(data):
	cones=[]
	prev=0
	arr=[]
	mini=2000
	min_angle=0
	for i in range(272,821):
		dist=data.ranges[i]
		#print("degree= ",i*0.33," dist= ",dist)
		if((abs(dist-prev)<0.5) and (dist<6)):
			arr.append(dist)
			if(dist<mini):
				mini=dist
				min_angle=i
			#print("min=",mini)
		elif(((dist-prev)>1)and(abs(prev-mini)<1)):
			a=[arr,mini,min_angle]
			#print(a)
			cones.append(a)
			arr=[]
			min_angle=0
			mini=2000
			
		prev=dist
		#print("prev=",prev)
	#print(cones)
	cone_confirmation(data,cones)


if __name__ == '__main__':
	c=0
	print("PERCEPTION")
	rospy.init_node('2d_perception',anonymous = True)
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.Subscriber("/gt_pose",PoseStamped, call)
	rospy.spin()