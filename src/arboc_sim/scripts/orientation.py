#!/usr/bin/env python

gyro_angle_x=gyro_angle_y=gyro_angle_z=0
PI = 3.14159265358979323846
abs_angle=0

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
#from pykalman import KalmanFilter
import numpy as np
from tf.transformations import euler_from_quaternion

def callback(msg, topic):
	eulerangles(msg, topic)

def publishit(data, topic):
    pub = rospy.Publisher(topic, Float64, queue_size=10)
    pub.publish(data)

def eulerangles(msg, topic):
	quaternions = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
	euler = euler_from_quaternion(quaternions)

	a = (int) (topic.split('_')[1][1])
	if(a%2==0):
		actuated_angle = euler[2]
	else:
		actuated_angle = euler[2]

	#rospy.loginfo(actuated_angle)
	publishit(actuated_angle, topic)


def subscribe():
	rospy.init_node('kalman_filter', anonymous=True)
	rospy.Subscriber('/arboc/imu/imu_01', Imu, callback, callback_args =  "/arboc/link/link_01_angle")
	rospy.Subscriber('/arboc/imu/imu_02', Imu, callback, callback_args =  "/arboc/link/link_02_angle")
	rospy.Subscriber('/arboc/imu/imu_03', Imu, callback, callback_args =  "/arboc/link/link_03_angle")
	rospy.Subscriber('/arboc/imu/imu_04', Imu, callback, callback_args =  "/arboc/link/link_04_angle")
	rospy.Subscriber('/arboc/imu/imu_05', Imu, callback, callback_args =  "/arboc/link/link_05_angle")
	rospy.Subscriber('/arboc/imu/imu_06', Imu, callback, callback_args =  "/arboc/link/link_06_angle")
	rospy.Subscriber('/arboc/imu/imu_07', Imu, callback, callback_args =  "/arboc/link/link_07_angle")
	rospy.Subscriber('/arboc/imu/imu_08', Imu, callback, callback_args =  "/arboc/link/link_08_angle")
	rospy.spin()

if __name__ == '__main__':
	subscribe()



'''

def gyroAndAcc(msg, topic):
	raw_ang_x = msg.angular_velocity.x
	raw_ang_y = msg.angular_velocity.y
	raw_ang_z = msg.angular_velocity.z

	gyro_x, gyro_y, gyro_z = convertingRawValues(raw_x=raw_ang_x, raw_y=raw_ang_y, raw_z=raw_ang_z)

	lin_a_x = msg.linear_acceleration.x
	lin_a_y = msg.linear_acceleration.y
	lin_a_z = msg.linear_acceleration.z

	acc_y = convertingRawAccValues(lin_a_x, lin_a_y, lin_a_z)
	angle_y = computeAngle(gyro_y, acc_y)
	#rospy.loginfo(topic+": angular_velocity: x:{}, y:{}, z:{} linear_acceleration: x:{}, y:{}, z:{}".format(ang_x, ang_y, ang_z, lin_a_x, lin_a_y, lin_a_z))
	rospy.loginfo(topic+": Orientation: y:{}*".format(angle_y))
	publishit(gyro_y)
	
def computeAngle(gyro_angle, acc_angle):
	global abs_angle
	AA=1
	abs_angle=AA*(gyro_angle) +(1 - AA) * acc_angle

	return abs_angle

def convertingRawValues(raw_x, raw_y, raw_z):
	
	global gyro_angle_x, gyro_angle_y, gyro_angle_z
	global PI

	DT=0.005

	gyro_angle_x += raw_x * DT
	gyro_angle_y += raw_y * DT
	gyro_angle_z += raw_z * DT

	return gyro_angle_x*180/PI, gyro_angle_y*180/PI, gyro_angle_z*180/PI

def convertingRawAccValues(x, y, z):
	AccYangle = np.arctan2(z,x)+PI
	return AccYangle*180/PI
'''