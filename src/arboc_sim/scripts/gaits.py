#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np

ui_prev = 0.0
e_prev = 0.0

kp=1.0
ki=0.0
kd=0.0

feedback={'link_01':0.0,
       'link_02':0.0,
       'link_03':0.0,
       'link_04':0.0,
       'link_05':0.0,
       'link_06':0.0,
       'link_07':0.0,
       'link_08':0.0}

class JointCmds:

    def __init__( self, num_mods ) :
        
        self.num_modules = num_mods
        self.jnt_cmd_dict = {}        
        self.joints_list = []
        self.t = 0.0
        for i in range(1,self.num_modules) :
            leg_str='joint_'
            if i < 10 :
                leg_str += '0' + str(i)
            else :
                leg_str += str(i)
            self.joints_list += [leg_str]

    def update( self, dt ) :

        self.t += dt
        a = 0.3*np.pi
        b = 2*np.pi
        c = 0.0*np.pi

        kp=1.0

        rospy.Subscriber('/arboc/link/link_01_angle', Float64, callback, callback_args =  "/arboc/link/link_01_angle")
        #rospy.loginfo("***")

        num_segments = 8
        gamma=-c/num_segments
        beta=b/num_segments
        alpha=a
        omega=5

        for i, jnt in enumerate(self.joints_list):
            if (i%2 == 0):
                feed = alpha*np.sin(self.t*omega + i*beta)+gamma
                self.jnt_cmd_dict[jnt] = feed 
                #pid_controller(y=feedback['link_0'+str(i+1)], yc=feed, h=dt, i=i)
                #rospy.loginfo(pid_controller(y=feedback['link_0'+str(i+1)], yc=feed, h=dt, i=i))
                #self.jnt_cmd_dict[jnt] = alpha*np.sin(self.t*omega + i*beta)+gamma + kp*error['link_0'+str(i+1)]
                #rospy.loginfo((alpha*np.sin(self.t*omega + i*beta)+gamma)*180/np.pi)
            #else:
            	#pass
                #self.jnt_cmd_dict[jnt] = alpha*np.sin(self.t*np.pi + i*beta)+gamma
                #rospy.loginfo(str(self.t))
    
        #rospy.loginfo(self.jnt_cmd_dict['joint_01'])
        return self.jnt_cmd_dict

def callback(msg, topic):
    global feedback
    a = (topic.split('/')[3][0:7])
    feedback[a] = msg.data
    rospy.loginfo(feedback[a])

def pid_controller(y, yc, i, h=1):

    global ui_prev
    global e_prev

    e = yc - y
    publishit(e, topic=('/arboc/error/link_0'+str(i)))

    up = kp*e
    ui = ui_prev + ki * h * e
    ud = kd * (e - e_prev)/h

    e_prev = e
    ui_prev = ui

    u = up + ui + ud

    return u

def publishit(data, topic):
    pub = rospy.Publisher(topic, Float64, queue_size=10)
    pub.publish(data)     


def publish_commands( num_modules, hz ):
    pub={}
    ns_str = '/arboc'
    cont_str = 'pc'
    for i in range(1,num_modules) :
        leg_str='joint_'
        if i < 10 :
            leg_str += '0' + str(i)
        else :
            leg_str += str(i)
        pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
                                        + cont_str + '/command',
                                        Float64, queue_size=10 )

    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(num_mods=num_modules)

    for jnt in ['joint_01', 'joint_02', 'joint_03', 'joint_04', 'joint_05', 'joint_06', 'joint_07']:
        pub[jnt].publish(0.0)

    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1./hz)        
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish( jnt_cmd_dict[jnt] )
        rate.sleep()


if __name__ == "__main__":
    try:
        num_modules = 8       
        hz = 50
        publish_commands( num_modules, hz )
    except rospy.ROSInterruptException:
        pass
