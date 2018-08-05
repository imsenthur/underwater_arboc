#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np

feed_angle=0.0
actual_angle=0.0
error={'link_01':0.0,
	   'link_02':0.0,
	   'link_03':0.0,
	   'link_04':0.0,
	   'link_05':0.0,
	   'link_06':0.0,
	   'link_07':0.0,
	   'link_08':0.0}

def computeError():
	rospy.init_node('PID', anonymous=True)
	rospy.Subscriber('/arboc/joint_01_pc/command', Float64, gaits_callback, callback_args =  "/arboc/joint_01_pc/command")
	rospy.Subscriber('/arboc/link/link_01_angle', Float64, orient_callback, callback_args =  "/arboc/link/link_01_angle")
	rospy.spin()

def gaits_callback(msg, topic):
	global feed_angle
	feed_angle = msg.data
	#rospy.loginfo(topic+":"+str(feed_angle))

def orient_callback(msg, topic):
	global actual_angle
	global error
	actual_angle = msg.data
	#rospy.loginfo(topic+":"+str(actual_angle))
	a = (topic.split('/')[3][0:7])
	error[a] = feed_angle - actual_angle
	#rospy.loginfo(error[a])
	publishit(error[a], topic=('/arboc/error/'+str(a)))

def publishit(data, topic):
    pub = rospy.Publisher(topic, Float64, queue_size=10)
    pub.publish(data)

if (__name__=="__main__"):
	computeError()

