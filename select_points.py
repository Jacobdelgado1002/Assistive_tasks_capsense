#!/usr/bin/env python

# -------------------------------------------------------
# used for selecting the point(s) of interest of the target(s)
# use shift+click to select point (might take several tries)
# -------------------------------------------------------



import rospy
import tf
import tf2_msgs.msg
import geometry_msgs.msg
import open3d as o3d
import networkx as nx
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

class PointAdd:

    def __init__(self,data):
        self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage, queue_size=1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id="camera_link"
            t.child_frame_id="sel_point"
            t.header.stamp=rospy.Time.now()
            t.transform.translation.x=data[0]
            t.transform.translation.y=data[1]
            t.transform.translation.z=data[2]
            
            t.transform.rotation.x=0
            t.transform.rotation.y=0
            t.transform.rotation.z=0
            t.transform.rotation.w=1

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('add_selected_point')
            listener=tf.TransformListener()
            
            pcd = o3d.io.read_point_cloud("/home/jacob/catkin_ws/src/jacob_package/scripts/Point_clouds/2022-07-09.ply") # read file.ply to select point(s)
            R = np.identity(3)  
            extent = np.ones(3)/.32 # trying to create a bounding box below 1 unit
            center = np.zeros(3) 
            obb = o3d.geometry.OrientedBoundingBox(center,R,extent)
            pcd = pcd.crop(obb)
            pcd = pcd.voxel_down_sample(voxel_size=0.018)
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(pcd)
            vis.run()  
            vis.destroy_window()
            pdata = vis.get_picked_points()

            (x,y,z)=pcd.points[pdata[0]]
            (x,y,z)=(-x,-y,-z)


            temp=PointAdd([x,y,z])
            rospy.spin()
            #rospy.sleep(2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf error")
    except KeyboardInterrupt:
        rospy.loginfo("closing")

