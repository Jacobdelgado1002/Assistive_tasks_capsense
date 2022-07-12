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
        rospy.sleep(1)
        
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

        # move the lift up to 75 cm
        pose={'joint_lift':0.75}
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
        pose={'joint_lift':0.83}
        self.move_to_pose(pose)
        rospy.sleep(5)

        return TriggerResponse(
            success=True,
            message='Completed Alignment'
        )

    # setup robot arm at 5 cm above the target for collecting data
    def ready_arm(self, request):
        rospy.loginfo('lower_until_contact')

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

    # collect data by extending arm out pass points of interest
    def collect_data(self, request):

        # extend end of arm to 45 cm
        pose={'wrist_extension':0.45}
        self.move_to_pose(pose)
        rospy.sleep(5)
        
        return TriggerResponse(
            success=True,
            message='Collected Data'
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

        self.trigger_write_hello_service = rospy.Service('ready_arm',
                                                         Trigger,
                                                         self.ready_arm)

        self.trigger_write_hello_service = rospy.Service('align_tool',
                                                         Trigger,
                                                         self.alignment)

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