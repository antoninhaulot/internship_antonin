#!/usr/bin/env python

import rospy
import actionlib
from math import pi
from bayes_people_tracker.msg import PeopleTracker
from scitos_ptu.msg import PtuGotoAction, PtuGotoGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *


class Data_Storage :
    def __init__(self):
        self.human_pose = []
        self.robot_pose = Pose()
        self.velocity = []
        self.average_velocity = [0,0]
        self.human_angle = 0
        self.human_distance = 0
        self.camera_angle = 0
        self.human_uuid = ''
        self.human_index = 0
        self.move = False
        self.start = rospy.Time.now()
        rospy.Subscriber("people_tracker/positions",PeopleTracker, self.get_data)
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped, self.get_robot_pose)
        

    def end_program(self):
        print 'PROGRAM IS SHUTTING DOWN'
        self.move_camera(0,10,30,10)
    
    def rotate_robot(self):       
        
        #CALCULATE THE ANGLE OF THE ROBOT IN THE WORLD
        angle_robot = euler_from_quaternion([self.robot_pose.orientation.x,
                                            self.robot_pose.orientation.y,
                                            self.robot_pose.orientation.z,
                                            self.robot_pose.orientation.w])
        #print "ANGLE ROBOT : " + str(angle_robot[2]*180/pi)
        angle_human = angle_robot[2]+(self.human_angle*pi/180)
        #print "ANGLE HUMAIN : " + str(angle*180/pi)
        quaternion = quaternion_from_euler(0,0,angle_human)
        #print quaternion
        
        #STARTING THE ROTATION OF THE ROBOT
        goal = self.moveToGoal(self.robot_pose.position.x,
                               self.robot_pose.position.y,
                               quaternion)
        if goal == True:
            return True
    
    def move_robot(self):
    
        angle_robot = euler_from_quaternion([self.robot_pose.orientation.x,
                                            self.robot_pose.orientation.y,
                                            self.robot_pose.orientation.z,
                                            self.robot_pose.orientation.w])
        #print "ANGLE ROBOT : " + str(angle_robot[2]*180/pi)
        angle_human = angle_robot[2]+(self.human_angle*pi/180)
        #print "ANGLE HUMAIN : " + str(angle*180/pi)
        quaternion = quaternion_from_euler(0,0,angle_human)
        #print quaternion


        goal = self.moveToGoal(self.human_pose[0],
                               self.human_pose[1],
                               quaternion)
        if goal == True:
            return True
        
    
    def get_robot_pose(self, data):
        
        self.robot_pose = data.pose.pose 
        #print self.robot_pose
    
    
    def moveToGoal(self, x_goal,y_goal, quaternion):

        ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        
        
        while(not ac.wait_for_server(rospy.Duration.from_sec(0.5))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        
        
        goal.target_pose.pose.position = Point(x_goal,y_goal,0)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        
        #print goal
        rospy.loginfo("Sending goal location")
        ac.send_goal(goal)
        
        
        
        ac.wait_for_result(rospy.Duration(1))

        if(ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("Destination reached")
            self.move = False
            return True
        else:
            rospy.loginfo("Fail to reach the destination")
            return False

    def move_camera(self, pan, tilt, pan_vel, tilt_vel):
    
        camera_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        camera_client.wait_for_server(rospy.Duration(60))
        
        camera_goal = PtuGotoGoal()
        camera_goal.pan = pan
        camera_goal.tilt = tilt
        camera_goal.pan_vel = pan_vel
        camera_goal.tilt_vel = tilt_vel
        
        self.camera_angle = camera_goal.pan
        
        camera_client.send_goal(camera_goal)
        camera_client.wait_for_result()
        
    def shutdown(self):
        self.move_camera(0,0,10,10)
    
    def calculate_futur_position(self) :
    
        time = 10       # Time in seconds 
        self.futur_position = [self.human_pose[0]+self.average_velocity[0]*time,
                               self.human_pose[1]+self.average_velocity[1]*time]
        #print 'next position : ' + str(futur_position)
    
    def calculate_average_velocity(self):

        self.average_velocity = [0,0]
        for i in range(0,len(self.velocity)) :
            self.average_velocity[0] += self.velocity[i][0]
            self.average_velocity[1] += self.velocity[i][1]
        self.average_velocity[0] = self.average_velocity[0] / len(self.velocity)
        self.average_velocity[1] = self.average_velocity[1] / len(self.velocity)
        #print 'average velocity : ' + str(average_velocity)
    

    def get_data(self,data):
        
        #print data
        if data.uuids != [] :
            # INITIALISATION OF THE HUMAN UUID THAT BETTY WILL FOLLOW
            if self.human_uuid == '':
                self.human_uuid = data.uuids[0]
            
            # GETTING & UPDATING THE INDEX OF THE HUMAN'S UUID    
            tracker = False
            for i in range(0, len(data.uuids)):
                if data.uuids[i] == self.human_uuid:
                    self.human_index = i
                    tracker = True
                    
                # IF THE HUMAN IS NO MORE IN THE SIGHT OF THE CAMERA, UPDATE UUID & INDEX WITH THE FIRST HUMAN OF THE LIST  ------ IN TEST
                if i == len(data.uuids) and tracker == False :
                    self.human_uuid = data.uuids[0]
                    selslackf.human_index = 0
    
            # GETTING ALL THE DATA
            self.human_pose = [data.poses[self.human_index].position.x,
                               data.poses[self.human_index].position.y]
            self.velocity.append([data.velocities[self.human_index].x,
                                  data.velocities[self.human_index].y])
            self.human_angle = data.angles[self.human_index]*180/pi
            self.human_distance = data.distances[self.human_index]
            if len(self.velocity) > 20 :
                self.velocity.pop(0)
                self.calculate_average_velocity()
                #print 'current human position : ' + str(self.human_pose)
                #print 'current robot ' + str(self.robot_pose)
                #print 'current human angle : ' + str(self.human_angle)
                #print 'current camera angle : ' + str(self.camera_angle)
                #print 'current distance : ' + str(self.human_distance)
                self.calculate_futur_position()
                
                                
        #rospy.sleep(0.5)

    def move_condition(self, max_angle, min_angle, max_distance, pan_vel):
        if abs(self.human_angle) < max_angle and abs(self.human_angle-self.camera_angle) > min_angle :
            self.move_camera(self.human_angle,10,pan_vel,10)
        if self.human_distance > max_distance :
            print 'FOLLOWING THE HUMAN'
            self.move = True
            self.move_robot()
        if abs(self.human_angle) > max_angle :
            print 'ROTATING THE ROBOT'
            self.move = True
            self.rotate_robot()


#def start_get_people_pose:
if __name__ == '__main__':
    max_angle = 80
    min_angle = 3
    max_distance = 2.5
    
    rospy.init_node("get_people_pose_test")
    data_use = Data_Storage()
    data_use.move_camera(0,10,30,10)
    
    while not rospy.is_shutdown():
        data_use.move_condition(max_angle, min_angle, max_distance, 30)
    rospy.on_shutdown(data_use.end_program)
    rospy.spin()
    start = rospy.Time.now()
    print start
