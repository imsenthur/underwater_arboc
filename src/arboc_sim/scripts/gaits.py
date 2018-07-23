#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np

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
        a = 0.5*np.pi
        b = 2*np.pi
        c = -0.25*np.pi

        num_segments = 8
        gamma=-c/num_segments
        beta=b/num_segments
        alpha=a
        omega=5

        for i, jnt in enumerate(self.joints_list):
            if i%2==0:
                self.jnt_cmd_dict[jnt] = alpha*np.sin(self.t*omega + i*beta)+gamma
            else:
                pass
                #self.jnt_cmd_dict[jnt] = alpha*np.sin(self.t*np.pi - 0.5*(i+1)*beta)+gamma
                #rospy.loginfo(str(self.t))
    
        #rospy.loginfo(self.jnt_cmd_dict['joint_01'])
        return self.jnt_cmd_dict


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
