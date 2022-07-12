#!/usr/bin/env python

# -------------------------------------------------------
# used for moving the robot wrist across the target area
# and saving the changes in distance (the changes in 
# distance are from the robot wrist to the point chosen) 
# -------------------------------------------------------

import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from datetime import date

class Get_distance(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_contact_detector=fp.ContactDetector(get_wrist_pitch,pitch_contact_fn,move_increment=.02)
        self.wrist_contact_detector.get_lift_pos=get_lift_pose

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        self.wrist_contact_detector.update(self.joint_states,self.stop_the_robot_service)

    # Move the stretch across the point chosen and save the changes in distance as the robot wrist moves
    def collect_data(self,request):
        try:

            # p = self.joint_states.name.index('head_pan')
            # current_effort_pitch = self.joint_states.effort[p]
            # listener.waitForTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            # listener.waitForTransform('camera_link','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans,rot)=listener.lookupTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0))
            # (trans,rot)=listener.lookupTransform('camera_link','link_aruco_top_wrist',rospy.Time(0.0))
            # rospy.loginfo("tag to robot wrist: " + str((trans,rot)))
            
            distances = []
            index = self.joint_states.name.index('wrist_extension') #put inside
            # rospy.loginfo("first index" + str(self.joint_states.position[index]))
            # pose={'wrist_extension':0.45}
            # self.move_to_pose(pose)
            # index = self.joint_states.name.index('wrist_extension')
            # rospy.loginfo("second index" + str(self.joint_states.position[index]))
            while(self.joint_states.position[index] < 0.43):
                listener.waitForTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
                (trans,rot)=listener.lookupTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0))
                distances.append(trans)
                rospy.loginfo("wrist to point: " + str((trans,rot)))
                current_pose = self.joint_states.position[index]
                new_pose = {'wrist_extension': current_pose + 0.01}
                self.move_to_pose(new_pose)
                # index = self.joint_states.name.index('wrist_extension')
                # pose={'wrist_extension':0.45}
                # self.move_to_pose(pose)

            # save distances from robot wrist to point
             
            today = date.today()  
            file = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/distances_measured_"+str(today)+".txt", "w")

            # for item in range(len(distances)):
            #     file.write(distances[item] + '\n')

            for item in distances:
                file.write(str(item)+ "\n")
                
            print("this is the list")
            print(distances)



            # listener.waitForTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans,rot)=listener.lookupTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0))
            # rospy.loginfo("wrist to point: " + str((trans,rot)))

            

            # listener.waitForTransform('camera_link','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans2,rot2)=listener.lookupTransform('camera_link','sel_point',rospy.Time(0.0))
            # rospy.loginfo("tag to selected point: " + str((trans2,rot2)))
            # self.finaltrans=[abs(trans[0])+trans2[0],abs(trans[1])+trans2[1],abs(trans[2])-trans2[2]]
            # rospy.loginfo("robot to point:" + str(self.finaltrans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

    

        # pose={'wrist_extension':0.45}
        # self.move_to_pose(pose)
        # rospy.sleep(5)
        # (x,y,z)=(self.finaltrans[0],self.finaltrans[1],self.finaltrans[2])
        # with self.joint_states_lock:
        #     i = self.joint_states.name.index('joint_lift')
        #     j = self.joint_states.name.index('wrist_extension')
        #     current_pos_lift = self.joint_states.position[i]
        #     current_pos_wrist = self.joint_states.position[j]
        # pose={'translate_mobile_base':abs(x)+.28}
        # rospy.loginfo("x: "+str(pose))
        # self.move_to_pose(pose)
        # rospy.sleep(2.0)
        # pose={'wrist_extension':current_pos_wrist+y+.08}
        # rospy.loginfo("y: "+str(pose))
        # self.move_to_pose(pose)
        # rospy.sleep(2.0)
        # p = self.joint_states.name.index('joint_wrist_pitch')
        # current_effort_pitch = self.joint_states.effort[p]
        # if (current_effort_pitch<.8):
        #         self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
        # rospy.sleep(1.0)
        return TriggerResponse(
            success=True,
            message='Collected data'
        )
    
    def main(self):
        hm.HelloNode.main(self, 'get_wrist_trans_to_point', 'get_wrist_trans_to_points', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        # register trigger services
        self.trigger_write_hello_service = rospy.Service('collect_data',
                                                         Trigger,
                                                         self.collect_data)
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

#referenced existing hello robot code for these in stretch_ros/funmap and stretch_ros/hello_misc.py
def get_wrist_pitch(joint_states):
    joint_name = 'joint_wrist_pitch'
    i = joint_states.name.index(joint_name)
    pitch_pos = joint_states.position[i]
    pitch_velocity = joint_states.velocity[i]
    pitch_effort = joint_states.effort[i]
    return [pitch_pos,pitch_velocity,pitch_effort]

def get_lift_pose(joint_states):
    joint_name = 'joint_lift'
    i = joint_states.name.index(joint_name)
    lift_pos = joint_states.position[i]
    return lift_pos

def pitch_contact_fn(effort, av_effort):
    single_effort_threshold = 1
    av_effort_threshold = 1.1

    if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold))

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__=='__main__':
    rospy.init_node("get_wrist_trans_to_point")

    listener=tf.TransformListener()
    
    node = Get_distance()
    node.main()