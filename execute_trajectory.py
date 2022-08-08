#!/usr/bin/env python

# -------------------------------------------------------
# used for picking up tool as well as robot camera and arm
# setup
# -------------------------------------------------------


import queue
from this import s
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from select_points import PointAdd

class ExecuteTrajectoryNode(hm.HelloNode):
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

    def trajectory_callback(self, points_list):
        self.data = points_list.data
        self.points = np.reshape(self.data,(len(self.data)/3,3))

    # setup camera to position for capturing depth image of target
    def ready_camera(self,request):
        rospy.loginfo('setup_camera')
        pose={'joint_head_pan':-1.6,'joint_head_tilt':-.50}
        #, 'translate_mobile_base': 0s
        self.move_to_pose(pose)
        # pose={'translate_mobile_base': 0}
        # self.move_to_pose(pose)
        # rospy.sleep(1)
        
        return TriggerResponse(
            success=True,
            message='Completed Head Setup'
        )

    def trigger_trajec_execution(self, request):
        # self.points = np.flip((self.points[self.points[:,0].argsort()]),0)
        # rospy.loginfo("List of points: "+str(self.points))

        # prev=self.points[0]
        # moves=[]
        # for pt in self.points:
        #     [xdiff,ydiff,zdiff] = pt-prev
        #     moves.append([xdiff,ydiff,zdiff])
        #     prev=pt
        # rospy.loginfo(moves)
        # for move in moves:
        #     with self.joint_states_lock:
        #         i = self.joint_states.name.index('joint_lift')
        #         j = self.joint_states.name.index('wrist_extension')
        #         current_pos_lift = self.joint_states.position[i]
        #         current_pos_wrist = self.joint_states.position[j]
            # pose={'translate_mobile_base':-xdiff}
            # self.move_to_pose(pose)
            # rospy.sleep(.75)
            # pose={'wrist_extension':current_pos_wrist-ydiff}
        rospy.loginfo('lower_until_contact')
        pose={'joint_lift':0.50,'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        # rospy.loginfo('lower_until_contact')
        pose={'gripper_aperture':.075}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose={'gripper_aperture':-.1}
        self.move_to_pose(pose)

        
        rospy.sleep(.25)
        #move down until contact detected
        p = self.joint_states.name.index('joint_wrist_pitch')
        current_effort_pitch = self.joint_states.effort[p]
        if (current_effort_pitch<1.3):
            self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
        #rospy.loginfo(self.wrist_contact_detector.not_stopped())
        """ pose={'joint_lift':current_pos_lift+zdiff}
        self.move_to_pose(pose) """
        rospy.sleep(.25)
        return TriggerResponse(
            success=True,
            message='Executed Trajectory'
        )

    # align robot arm and wrist to grab tool
    def alignment(self, request):
        rospy.loginfo('grap_tool')
        # set lift, pitch, roll, and yaw to starting positions
        pose={'wrist_extension':0, 'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        rospy.sleep(1)

        # move the lift up to 74.5 cm
        pose={'joint_lift':0.745}
        self.move_to_pose(pose)
        rospy.sleep(4) 

        # extend end of arm to 20 cm
        pose={'wrist_extension':0.20}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # open gripper (put tool on gripper)
        pose={'gripper_aperture':0.050}
        self.move_to_pose(pose)
        rospy.sleep(15)

        # close gripper (secure tool on gripper)
        pose={'gripper_aperture':-0.1}
        self.move_to_pose(pose)
        rospy.sleep(15)

        # move the lift up to 83 cm (lift tool up)
        pose={'joint_lift':0.83, 'wrist_extension':0}
        self.move_to_pose(pose)
        rospy.sleep(5)

        pose={'joint_lift':0.60, 'joint_head_pan':-1.6,'joint_head_tilt':-.50}
        self.move_to_pose(pose)
        rospy.sleep(5)        

        return TriggerResponse(
            success=True,
            message='Completed Alignment'
        )

    # setup robot arm at 5 cm above the target for collecting data
    def ready_arm_task1(self, request):
        rospy.loginfo('lower_until_contact')

        pose={'joint_lift':0.85}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # extend end of arm out to 20 cm 
        pose={'wrist_extension':0.20}
        self.move_to_pose(pose)
        rospy.sleep(5)
          
        # move down until contact detected
        p = self.joint_states.name.index('joint_wrist_pitch')
        current_effort_pitch = self.joint_states.effort[p]
        if (current_effort_pitch<0.5):
            self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
            
        
        #rospy.loginfo(self.wrist_contact_detector.not_stopped())
        """ pose={'joint_lift':current_pos_lift+zdiff}
        self.move_to_pose(pose) """
        rospy.sleep(1)

        # retract end of arm to 0 cm and make sure pitch, roll and yaw are at 0 
        pose={'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0, 'wrist_extension':0}
        self.move_to_pose(pose)
        rospy.sleep(5) 
        
        return TriggerResponse(
            success=True,
            message='Completed Arm Setup'
        )
    
    def ready_arm_task2(self, request):
        rospy.loginfo('ready_arm_task2')

        pose={'joint_lift':1.09}
        self.move_to_pose(pose)
        rospy.sleep(5)


        pose={'wrist_extension':0.20, 'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        rospy.sleep(5)
        
        return TriggerResponse(
            success=True,
            message='Completed Arm Setup'
        )

    def ready_arm_task3(self, request):
        rospy.loginfo('ready_arm_task3')

        pose={'joint_lift':0.85}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # extend end of arm out to 20 cm 
        pose={'wrist_extension':0.20}
        self.move_to_pose(pose)
        rospy.sleep(5)
          
        # move down until contact detected
        p = self.joint_states.name.index('joint_wrist_pitch')
        current_effort_pitch = self.joint_states.effort[p]
        if (current_effort_pitch<0.5):
            self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
        
        rospy.sleep(5)
        # retract end of arm to 0 cm and make sure pitch, roll and yaw are at 0 
        pose={'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0, 'translate_mobile_base': -0.5}
        self.move_to_pose(pose)

    def ready_arm_task4(self, request):
        rospy.loginfo('ready_arm_task4')

        pose={'joint_lift':0.85}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # extend end of arm out to 20 cm 
        pose={'wrist_extension':0.20}
        self.move_to_pose(pose)
        rospy.sleep(5)
          
        # move down until contact detected
        p = self.joint_states.name.index('joint_wrist_pitch')
        current_effort_pitch = self.joint_states.effort[p]
        if (current_effort_pitch<0.5):
            self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
        
        rospy.sleep(5)
        # retract end of arm to 0 cm and make sure pitch, roll and yaw are at 0 
        pose={'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':-1.38}
        self.move_to_pose(pose)
         
        
        return TriggerResponse(
            success=True,
            message='Completed Arm Setup'
        )
    
    def ready_arm_task5(self, request):
        rospy.loginfo('ready_arm_task5')

        pose={'joint_lift':0.95}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # extend end of arm out to 30 cm 
        pose={'wrist_extension':0.35}
        self.move_to_pose(pose)
        rospy.sleep(5)
          
        # retract end of arm to 0 cm and make sure pitch, roll and yaw are at 0 
        pose={'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
         
        
        return TriggerResponse(
            success=True,
            message='Completed Arm Setup'
        )

    def ready_arm_task7(self, request):
        rospy.loginfo('lower_until_contact')

        pose={'joint_lift':0.85}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # extend end of arm out to 20 cm 
        pose={'wrist_extension':0.20}
        self.move_to_pose(pose)
        rospy.sleep(5)

        # retract end of arm to 0 cm and make sure pitch, roll and yaw are at 0 
        pose={'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        rospy.sleep(5) 
        
        return TriggerResponse(
            success=True,
            message='Completed Arm Setup'
        )

    # collect data by extending arm out pass points of interest
    def data_collection(self, request):

        self.alignment(request)
        # execfile("/home/jacob/catkin_ws/src/jacob_package/scripts/camera_pointcloud.py")
        exec(open("camera_pointcloud.py").read())
        # exec(open("select_points.py").read())


        # execfile("/home/jacob/catkin_ws/src/jacob_package/scripts/select_points.py")
        # self.ready_arm(request)
        # extend end of arm to 45 cm
        # pose={'wrist_extension':0.45}
        # self.move_to_pose(pose)
        # rospy.sleep(5)

        # index = self.joint_states.name.index('joint_gripper_finger_right')
        # print(self.joint_states.position[index])

        # listener.waitForTransform('joint_gripper_finger_right','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
        # (trans,rot)=listener.lookupTransform('joint_gripper_finger_right','link_aruco_top_wrist',rospy.Time(0.0))
        
        return TriggerResponse(
            success=True,
            message='Collected Data'
        )

    # used to reset position for next trial run
    def reset_trial(self, request):

        # extend end of arm out to 0 cm (used for task1)
        pose={'wrist_extension':0.0}
        self.move_to_pose(pose)
        rospy.sleep(3)

        # lift end of arm out to 1.09 (used for task2)
        # pose={'joint_lift':1.09}
        # self.move_to_pose(pose)
        # rospy.sleep(3)

        # return base to initial position (used for task3)
        # pose={'translate_mobile_base': 0.32}
        # self.move_to_pose(pose)
        # rospy.sleep(3)

        # return yaw to initial position (used for task4)
        # pose={'joint_wrist_yaw':1.5}
        # pose={'joint_wrist_yaw':-1.38}
        # self.move_to_pose(pose)
        # rospy.sleep(3)

        # return pitch to initial position (used for task5)
        # pose={'joint_wrist_pitch':0}
        # self.move_to_pose(pose)
        # rospy.sleep(3)

        # pose={'joint_lift':0.85}
        # self.move_to_pose(pose)
        # rospy.sleep(3)

        return TriggerResponse(
            success=True,
            message='Completed reset'
        )

    def data_test(self, request):
        rospy.loginfo('data_test')

        # extend end of arm out to 30 cm 
        pose={'wrist_extension':0.45}
        self.move_to_pose(pose)
        rospy.sleep(5)
        
        return TriggerResponse(
            success=True,
            message='Completed data_test'
        )


    def main(self):
        hm.HelloNode.main(self, 'exec_trajectory', 'exec_trajectory', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trajectory_subscriber = rospy.Subscriber('/trajectory_points',Float64MultiArray, self.trajectory_callback)

        # register trigger servicess
        self.trigger_write_hello_service = rospy.Service('execute_trajectory',
                                                         Trigger,
                                                         self.trigger_trajec_execution)

        self.trigger_write_hello_service = rospy.Service('ready_camera',
                                                         Trigger,
                                                         self.ready_camera)

        self.trigger_write_hello_service = rospy.Service('ready_arm_task1',
                                                         Trigger,
                                                         self.ready_arm_task1)

        self.trigger_write_hello_service = rospy.Service('ready_arm_task2',
                                                         Trigger,
                                                         self.ready_arm_task2)

        self.trigger_write_hello_service = rospy.Service('ready_arm_task3',
                                                         Trigger,
                                                         self.ready_arm_task3)

        self.trigger_write_hello_service = rospy.Service('ready_arm_task4',
                                                         Trigger,
                                                         self.ready_arm_task4)

        self.trigger_write_hello_service = rospy.Service('ready_arm_task5',
                                                         Trigger,
                                                         self.ready_arm_task5)                                                   

        self.trigger_write_hello_service = rospy.Service('ready_arm_task7',
                                                         Trigger,
                                                         self.ready_arm_task7) 

        self.trigger_write_hello_service = rospy.Service('align_tool',
                                                         Trigger,
                                                         self.alignment)

        self.trigger_write_hello_service = rospy.Service('data_collection',
                                                         Trigger,
                                                         self.data_collection)

        self.trigger_write_hello_service = rospy.Service('reset_trial',
                                                         Trigger,
                                                         self.reset_trial)                                                          

        self.trigger_write_hello_service = rospy.Service('data_test',
                                                         Trigger,
                                                         self.data_test) 
                                                         
        # self.trigger_write_hello_service = rospy.Service('collect_data',
        #                                                  Trigger,
        #                                                  self.collect_data)   

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
    rospy.init_node("exec_trajectory")
    
    node = ExecuteTrajectoryNode()
    node.main()