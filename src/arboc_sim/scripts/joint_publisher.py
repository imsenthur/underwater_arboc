#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self.joint_01 = rospy.Publisher('/arboc/joint_01_pc/command', Float64, queue_size=1)
        self.joint_02 = rospy.Publisher('/arboc/joint_02_pc/command', Float64, queue_size=1)
        self.joint_03 = rospy.Publisher('/arboc/joint_03_pc/command', Float64, queue_size=1)
        self.joint_04 = rospy.Publisher('/arboc/joint_04_pc/command', Float64, queue_size=1)
        self.joint_05 = rospy.Publisher('/arboc/joint_05_pc/command', Float64, queue_size=1)
        self.joint_06 = rospy.Publisher('/arboc/joint_06_pc/command', Float64, queue_size=1)
        self.joint_07 = rospy.Publisher('/arboc/joint_07_pc/command', Float64, queue_size=1)
        
        self.publishers_array.append(self.joint_01)
        self.publishers_array.append(self.joint_02)
        self.publishers_array.append(self.joint_03)
        self.publishers_array.append(self.joint_04)
        self.publishers_array.append(self.joint_05)
        self.publishers_array.append(self.joint_06)
        self.publishers_array.append(self.joint_07)

    def move_joints(self, joints_array):
   
        i=0
        for publisher in self.publishers_array:
          joint_value = Float64()
          joint_value.data=joints_array[i]
          rospy.loginfo(str(joint_value)+" on "+str((publisher.name).split('/')[2]) + " @rate: " + str(rate_value))    
          self.joint_01.publish(joint_value)
          self.joint_02.publish(joint_value)
          publisher.publish(joint_value)
          i = i+1
        rospy.loginfo("*************************************************************************************") 


    def start_loop(self, rate_value = 2.0):
        rospy.loginfo("Starting Control")
        pos1 = [-1.0,1.0,-1.0,1.0,-1.0,1.0,-1.0]
        pos2 = [1.0,-1.0,1.0,-1.0,1.0,-1.0,1.0]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()


if __name__=="__main__":

  try:
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()

    #** tweak the publication rate **#
    rate_value = 0.5
    joint_publisher.start_loop(rate_value)

  except rospy.ROSInterruptException:
    pass