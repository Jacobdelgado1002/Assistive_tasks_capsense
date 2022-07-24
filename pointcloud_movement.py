#!/usr/bin/env python

# -------------------------------------------------------
# used for moving the robot wrist across the target area
# and saving the changes in distance (the changes in 
# distance are from the robot wrist to the point chosen) 
# -------------------------------------------------------

import rospy
import os
# import tf2_ros
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from datetime import date
# from execute_trajectory import ExecuteTrajectoryNode
import execute_trajectory as tra

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
    def collect_data_task1(self,request):
        try:

            # p = self.joint_states.name.index('head_pan')
            # current_effort_pitch = self.joint_states.effort[p]
            # listener.waitForTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            # listener.waitForTransform('camera_link','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans,rot)=listener.lookupTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0))
            # (trans,rot)=listener.lookupTransform('camera_link','link_aruco_top_wrist',rospy.Time(0.0))
            # rospy.loginfo("tag to robot wrist: " + str((trans,rot)))
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []
            index = self.joint_states.name.index('wrist_extension') #put inside
            
            # rospy.loginfo("first index" + str(self.joint_states.position[index]))
            # pose={'wrist_extension':0.45}
            # self.move_to_pose(pose)
            # index = self.joint_states.name.index('wrist_extension')
            # rospy.loginfo("second index" + str(self.joint_states.position[index]))

            # index = self.joint_states.name.index('electrode6')
            # print(self.joint_states.position[index])
            # exit()

            # while(self.joint_states.position[index] < 0.43):
            #     trans_iteration = []
            #     rot_iteration = []
            #     try:
            #         listener.waitForTransform('electrode1','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            #         (trans1,rot1)=listener.lookupTransform('electrode1','sel_point',rospy.Time(0.0))
            #         # print(trans1, rot1)
            #         trans_iteration.append(trans1)
            #         rot_iteration.append(rot1)
            #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #         rospy.loginfo("tf exception")

            while(self.joint_states.position[index] < 0.43):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)
                    # rotations6.append(rot6)
                
                    # translations.append(trans_iteration)
                    # rotations.append(rot_iteration)

                    # listener.waitForTransform('root_tool', 'sel_point', rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans,rot)=listener.lookupTransform('root_tool', 'sel_point', rospy.Time(0.0))

                    # rospy.loginfo("wrist to point: " + str((trans,rot)))
                    # current_pose = self.joint_states.position[index]
                    # new_pose = {'wrist_extension': current_pose + 0.01}
                    # self.move_to_pose(new_pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                current_pose = self.joint_states.position[index]
                new_pose = {'wrist_extension': current_pose + 0.005}
                self.move_to_pose(new_pose)


            # while(self.joint_states.position[index] < 0.43):
            #     listener.waitForTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            #     (trans,rot)=listener.lookupTransform('link_aruco_top_wrist','sel_point',rospy.Time(0.0))
            #     translations.append(trans)
            #     rospy.loginfo("wrist to point: " + str((trans,rot)))
            #     current_pose = self.joint_states.position[index]
            #     new_pose = {'wrist_extension': current_pose + 0.01}
            #     self.move_to_pose(new_pose)
                # index = self.joint_states.name.index('wrist_extension')
                # pose={'wrist_extension':0.45}
                # self.move_to_pose(pose)

            # save translations from robot wrist to point
             
            today = date.today()  
            # file_trans = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+".txt", "w")
            # file_rot = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+".txt", "w")

            # for item in range(len(distances)):
            #     file.write(distances[item] + '\n')
            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

            # # for sublists in translations:
            # #     count = 1
            # #     print(sublists)
            # #     for item in sublists:
            # #         print(item)
            # #         count = count + 1
            
            # if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)):
            #     os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today))
                
            # file_rot1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations1.txt", "w")
            # for sublists in rotations1:
            #     for item in sublists:
            #         file_rot1.write(str(item))
            #     file_rot1.write("\n")

            # file_rot2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations2.txt", "w")
            # for sublists in rotations2:
            #     for item in sublists:
            #         file_rot2.write(str(item))
            #     file_rot2.write("\n")
            
            # file_rot3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations3.txt", "w")
            # for sublists in rotations3:
            #     for item in sublists:
            #         file_rot3.write(str(item))
            #     file_rot3.write("\n")

            # file_rot4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations4.txt", "w")
            # for sublists in rotations4:
            #     for item in sublists:
            #         file_rot4.write(str(item))
            #     file_rot4.write("\n")

            # file_rot5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations5.txt", "w")
            # for sublists in rotations5:
            #     for item in sublists:
            #         file_rot5.write(str(item))
            #     file_rot5.write("\n")
            
            # file_rot6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/rotations"+str(today)+"/dataset"+str(dataset)+"/rotations6.txt", "w")
            # for sublists in rotations6:
            #     for item in sublists:
            #         file_rot6.write(str(item))
            #     file_rot6.write("\n")

            # for item in translations:
            #     file_trans.write(str(item)+ "\n")
            
            # for item in rotations:
            #     file_rot.write(str(item)+ "\n")
                
            # print("this is the list")
            # print(translations)



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

    # Move the stretch across the point chosen and save the changes in distance as the robot wrist moves
    def collect_data_task2(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []

            

            index = self.joint_states.name.index('joint_lift') #put inside

            while(self.joint_states.position[index] > 0.86):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                current_pose = self.joint_states.position[index]
                new_pose = {'joint_lift': current_pose - 0.005}
                self.move_to_pose(new_pose)


             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        
        return TriggerResponse(
            success=True,
            message='Collected data'
        )

    def collect_data_task3(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []

            counter = 0.0

            while(counter < 1.0):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                counter = counter + 0.005
                new_pose = {'translate_mobile_base': 0.005}
                self.move_to_pose(new_pose)


             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

        return TriggerResponse(
            success=True,
            message='Collected data'
        )

    # Move the stretch across the point chosen and save the changes in distance as the robot wrist moves
    def collect_data_task4(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []


            index = self.joint_states.name.index('joint_wrist_yaw') #put inside

            while(self.joint_states.position[index] < 1.5):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                current_pose = self.joint_states.position[index]
                new_pose = {'joint_wrist_yaw': current_pose + 0.02}
                self.move_to_pose(new_pose)


             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

        return TriggerResponse(
            success=True,
            message='Collected data'
        )
    
    # Move the stretch across the point chosen and save the changes in distance as the robot wrist moves
    def collect_data_task5(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []


            index = self.joint_states.name.index('joint_wrist_pitch') #put inside
            
            while(self.joint_states.position[index] > -0.65):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                current_pose = self.joint_states.position[index]
                new_pose = {'joint_wrist_pitch': current_pose - 0.01}
                self.move_to_pose(new_pose)


             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

        return TriggerResponse(
            success=True,
            message='Collected data'
        )
    
    def collect_data_task6(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []


            index = self.joint_states.name.index('joint_wrist_pitch') #put inside
            
            while(self.joint_states.position[index] > -0.65):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                current_pose = self.joint_states.position[index]
                new_pose = {'joint_wrist_pitch': current_pose - 0.01}
                self.move_to_pose(new_pose)
            
            counter = 0.0

            while(counter < 1.0):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                counter = counter + 0.005
                new_pose = {'translate_mobile_base': -0.005}
                self.move_to_pose(new_pose)


             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

        return TriggerResponse(
            success=True,
            message='Collected data'
        )

    def collect_data_task7(self,request):
        try:
            
            translations1 = []
            translations2 = []
            translations3 = []
            translations4 = []
            translations5 = []
            translations6 = []
            # rotations1 = []
            # rotations2 = []
            # rotations3 = []
            # rotations4 = []
            # rotations5 = []
            # rotations6 = []
            
            counter = 0.0
            count = 0

            while(counter < 1.0):
                # trans_iteration = []
                # rot_iteration = []
                try:
                    listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))

                    listener.waitForTransform('electrode1','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans1,rot1)=listener.lookupTransform('electrode1','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans_b,rot_b)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    ar1 = np.array(trans1)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    # # # translations1 = []
                    # # # rotations1 = []
                    translations1.append(subs)
                    # # # rotations1.append(rot1)
                

                    listener.waitForTransform('electrode2','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans2,rot2)=listener.lookupTransform('electrode2','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans2)
                    # rot_iteration.append(rot2)
                    ar1 = np.array(trans2)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations2.append(subs)
                    # rotations2.append(rot2)

                    listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans3,rot3)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans3)
                    # rot_iteration.append(rot3)
                    ar1 = np.array(trans3)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations3.append(subs)
                    # rotations3.append(rot3)

                    listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans4,rot4)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                    # listener.waitForTransform('sel_point','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    # (trans2,rot2)=listener.lookupTransform('sel_point','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans4)
                    # rot_iteration.append(rot4)
                    ar1 = np.array(trans4)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations4.append(subs)
                    # rotations4.append(rot4)

                    listener.waitForTransform('electrode5','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans5,rot5)=listener.lookupTransform('electrode5','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans5)
                    # rot_iteration.append(rot5)
                    ar1 = np.array(trans5)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations5.append(subs)
                    # rotations5.append(rot5)

                    listener.waitForTransform('electrode6','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                    (trans6,rot6)=listener.lookupTransform('electrode6','base_link',rospy.Time(0.0))
                    # trans_iteration.append(trans6)
                    # rot_iteration.append(rot6)
                    ar1 = np.array(trans6)
                    ar2 = np.array(trans_b)
                    subs = list(np.subtract(ar1, ar2))

                    translations6.append(subs)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("tf exception")
                
                index = self.joint_states.name.index('joint_lift') #put inside
                if(count % 2 == 0):
                    current_pose = self.joint_states.position[index]
                    new_pose = {'joint_lift': current_pose + 0.020}
                    self.move_to_pose(new_pose)
                else:
                    current_pose = self.joint_states.position[index]
                    new_pose = {'joint_lift': current_pose - 0.020}
                    self.move_to_pose(new_pose)

                counter = counter + 0.005
                count = count + 1
                new_pose = {'translate_mobile_base': -0.005}
                self.move_to_pose(new_pose)
             
            today = date.today()  

            dataset = 1

            if not os.path.exists("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)):
                os.makedirs("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset))
            
            file_trans1 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode1.txt", "w")
            for sublists in translations1:
                for item in sublists:
                    file_trans1.write(str(item) + " ")
                file_trans1.write("\n")
            
            file_trans2 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode2.txt", "w")
            for sublists in translations2:
                for item in sublists:
                    file_trans2.write(str(item) + " ")
                file_trans2.write("\n")

            file_trans3 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode3.txt", "w")
            for sublists in translations3:
                for item in sublists:
                    file_trans3.write(str(item) + " ")
                file_trans3.write("\n")

            file_trans4 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode4.txt", "w")
            for sublists in translations4:
                for item in sublists:
                    file_trans4.write(str(item) + " ")
                file_trans4.write("\n")

            file_trans5 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode5.txt", "w")
            for sublists in translations5:
                for item in sublists:
                    file_trans5.write(str(item) + " ")
                file_trans5.write("\n")

            file_trans6 = open("/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations"+str(today)+"/dataset"+str(dataset)+"/electrode6.txt", "w")
            for sublists in translations6:
                for item in sublists:
                    file_trans6.write(str(item) + " ")
                file_trans6.write("\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")

        return TriggerResponse(
            success=True,
            message='Collected data'
        )
    
    def main(self):
        hm.HelloNode.main(self, 'get_wrist_trans_to_point', 'get_wrist_trans_to_points', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        # register trigger services
        self.trigger_write_hello_service = rospy.Service('collect_data_task1',
                                                         Trigger,
                                                         self.collect_data_task1)

        self.trigger_write_hello_service = rospy.Service('collect_data_task2',
                                                         Trigger,
                                                         self.collect_data_task2)

        self.trigger_write_hello_service = rospy.Service('collect_data_task3',
                                                         Trigger,
                                                         self.collect_data_task3)

        self.trigger_write_hello_service = rospy.Service('collect_data_task4',
                                                         Trigger,
                                                         self.collect_data_task4) 

        self.trigger_write_hello_service = rospy.Service('collect_data_task5',
                                                         Trigger,
                                                         self.collect_data_task5) 

        self.trigger_write_hello_service = rospy.Service('collect_data_task6',
                                                         Trigger,
                                                         self.collect_data_task6)

        self.trigger_write_hello_service = rospy.Service('collect_data_task7',
                                                         Trigger,
                                                         self.collect_data_task7)                                                                                                                                                                                                 

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

    # Use this if you want to implement tf2 

    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)
    # while True:
    #     try:
    #         trans = tfBuffer.lookup_transform('root_tool', 'electrode6', rospy.Time())
    #         print(trans)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         continue

    listener=tf.TransformListener()
    # while True:
    #     try:
    #         (trans, rot) = listener.lookupTransform('root_tool', 'electrode6', rospy.Time(0))
    #         print(trans, rot)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    node = Get_distance()
    node.main()