#! /usr/bin/env python

import rospy

import actionlib

from internship_antonin.msg import *
from get_people_pose import *
#from internship_antonin.get_people_pose import *

class get_people_poseAction(object):
    # create messages that are used to publish feedback/result
    _feedback = internship_antonin.msg.get_people_poseFeedback()
    _result = internship_antonin.msg.get_people_poseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, internship_antonin.msg.get_people_poseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        data = Data_Storage()
        print goal
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, following human with the camera for %i seconds' % (self._action_name, goal.duration))
        
        # start executing the action
        for i in range(1, 4):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
           
            r.sleep()
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        end = rospy.Time.now()
        #print end.secs
        #print data.start.secs
        #print end.secs - data.start.secs
        #print goal.duration
        while goal.duration > end.secs - data.start.secs :
            data.move_condition(goal.angle_max, goal.angle_min, goal.distance_max, goal.pan_vel)
            end = rospy.Time.now()
        print('Time is running out, resetting the camera position')
        data.move_camera(0,10,30,10)
        
if __name__ == '__main__':
    rospy.init_node('follow_people')
    
    #print type(data.start)
    #print data.start
    server = get_people_poseAction(rospy.get_name())
    rospy.spin()
