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
from datetime import date
import numpy as np

class PointAdd:

    def __init__(self,data):
        self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage, queue_size=1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id="camera_link"
            t.child_frame_id="sel_point"
            # t.header.frame_id="sel_point"
            # t.child_frame_id="base_link"
            t.header.stamp=rospy.Time.now()
            # t.transform.translation.x=data[2]
            # t.transform.translation.y=-data[1]
            # t.transform.translation.z=data[0]
            t.transform.translation.x=data[2]
            t.transform.translation.y=-data[0]
            t.transform.translation.z=-data[1]
            
            t.transform.rotation.x=0
            t.transform.rotation.y=0
            t.transform.rotation.z=0
            t.transform.rotation.w=1

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)
    
            # t.header.frame_id="camera_link"
            # t.child_frame_id="sel_point_{}",name

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('add_selected_point')
            listener=tf.TransformListener()
            
            today = date.today()
            # pcd = o3d.io.read_point_cloud("/home/jacob/catkin_ws/src/jacob_package/scripts/Point_clouds/2022-07-21.ply") # read file.ply to select point(s)
            # pcd = o3d.io.read_point_cloud("/home/jacob/catkin_ws/src/jacob_package/scripts/Point_clouds/"+str(today)+".ply") # read file.ply to select point(s)            

            xyz = np.load("./tmp_pc.npy")
            xyz = xyz.reshape(-1, 3)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            
            R = np.identity(3)  
            extent = np.ones(3)/.65 # trying to create a bounding box below 1 unit
            center = np.zeros(3) 
            obb = o3d.geometry.OrientedBoundingBox(center,R,extent)
            pcd = pcd.crop(obb)
            pcd = pcd.voxel_down_sample(voxel_size=0.008)
            
            
            
            try:

                listener.waitForTransform('base_link','electrode3',rospy.Time(0.0),rospy.Duration(1.0))
                (trans1,rot1)=listener.lookupTransform('base_link','electrode3',rospy.Time(0.0))
                p1 = trans1
                listener.waitForTransform('base_link','electrode4',rospy.Time(0.0),rospy.Duration(1.0))
                (trans2,rot2)=listener.lookupTransform('base_link','electrode4',rospy.Time(0.0))
                p2 = trans2

                # listener.waitForTransform('electrode3','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                # (trans1,rot1)=listener.lookupTransform('electrode3','base_link',rospy.Time(0.0))
                # p1 = trans1
                # listener.waitForTransform('electrode4','base_link',rospy.Time(0.0),rospy.Duration(1.0))
                # (trans2,rot2)=listener.lookupTransform('electrode4','base_link',rospy.Time(0.0))
                # p2 = trans2

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf exception")

            p1 = np.array(p1)
            p2 = np.array(p2)

            p1 = p1[[0, 1]]
            p2 = p2[[0, 1]]

            midpoint = (np.add(p1, p2) / 2)
            
            # listener.waitForTransform('electrode3','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans3,rot3)=listener.lookupTransform('electrode3','sel_point',rospy.Time(0.0))


            # listener.waitForTransform('electrode4','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans4,rot4)=listener.lookupTransform('electrode4','sel_point',rospy.Time(0.0))


            # listener.waitForTransform('electrode5','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans5,rot5)=listener.lookupTransform('electrode5','sel_point',rospy.Time(0.0))


            # listener.waitForTransform('electrode6','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            # (trans6,rot6)=listener.lookupTransform('electrode6','sel_point',rospy.Time(0.0))

            # filter pointcloud to ignore table and capture the arm. 
            # Might need to change tressholds for d and theta if setup is changed
            filtered_pointcloud = []

            # iterated over the position of each of the points in the pointcloud
            for point in pcd.points:
                axis_vector = np.subtract(p1, p2)
                traj_vector = point[0:2] - midpoint  # find the 2D vector btw the midpoint of the array and a given point on the point cloud

                # find deviation of a given point on the pointcloud and the axis direction
                #     you may not actually need to do this for your purposes
                d = np.linalg.norm(np.cross(axis_vector, traj_vector))/np.linalg.norm(axis_vector)

                axis_vector = axis_vector / np.linalg.norm(axis_vector)
                traj_vector = traj_vector/np.linalg.norm(traj_vector)  # make trajectory vector a unit vector
                
                # find the angle between the axis and trajectory vectors
                theta = np.arccos(np.clip(np.dot(axis_vector, traj_vector), -1.0, 1.0))

                # if d < 0.20 and theta < 2.50:
                # if d < 0.14 and theta < 3.00:
                if d < 0.22 and theta < 2.65:
                    filtered_pointcloud.append(point)

            # task flag
            # 1 = view filtered pointcloud over ninitial pointcloud (both in the same window)
            # 2 = choose point manually
            # default(else) = choose closest point on the pointcloud

            flag = 3

            if flag == 1:

                # view filtered pointcloud over ninitial pointcloud (both in the same window)

                pcd_sel = o3d.geometry.PointCloud()
                pcd_sel.points = o3d.utility.Vector3dVector(np.asarray(filtered_pointcloud))
                pcd_sel.paint_uniform_color([1, 0, 0])
                pcd.paint_uniform_color([0,1,0])
                o3d.visualization.draw_geometries([pcd, pcd_sel])
                exit()
            

            # down sample initial pointcloud (used as an alternative to pointcloud filtering above)

            # pcd = pcd.select_down_sample(np.where(pcd_array[:, 1] > -0.054)[0])
            # pcd_array = np.asarray(pcd.points)
            # pcd = pcd.select_down_sample(np.where(pcd_array[:, 2] <  -0.69)[0])
            # print(pcd)
            # exit()

            # create new point cloud (point cloud of interest) for testing

            # pcd_interest = o3d.geometry.PointCloud()
            # pcd_interest.points = o3d.utility.Vector3dVector(np.asarray(filtered_pointcloud))

            pcd.points = o3d.utility.Vector3dVector(np.asarray(filtered_pointcloud))
            pcd_array = np.asarray(pcd.points)

            # o3d.visualization.draw_geometries([pcd])
            # exit()

            if flag == 2:
            
                # used for selecting point manually

                vis = o3d.visualization.VisualizerWithEditing()
                vis.create_window()
                vis.add_geometry(pcd)
                vis.run()  
                vis.destroy_window()
                pdata = vis.get_picked_points()
                print("p_data is: ", pdata)
                (x,y,z)=pcd.points[pdata[0]]
            
            else:
                magnitudes = []
                for item in pcd_array:
                    magnitudes.append(np.linalg.norm(item))

                # print(magnitudes)
                # exit()

                closest = min(magnitudes)
                # print(closest)
                closest_index = magnitudes.index(closest)
                # print(magnitudes.index(closest))
                # exit()
                pcd_sel = o3d.geometry.PointCloud()

                pcd_sel.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[closest_index].reshape(1, 3))
                # print(np.asarray(pcd_sel.points))
                # exit()
                pcd_sel.paint_uniform_color([1, 0, 0])
                pcd.paint_uniform_color([0,1,0])
                o3d.visualization.draw_geometries([pcd, pcd_sel])
                # o3d.visualization.draw_geometries([pcd_sel])

                (x,y,z) = pcd.points[closest_index]

                
                # print(x)
                # print(y)
                # print(z)
                # exit()


            # (x,y,z)=(-x,-y,-z)

            # print(pdata)
            # # print(pcd.points[pdata[0]])
            # print(x)
            # print(y)
            # print(z)
            # exit()

            # min_z_idx = np.argmin(xyz[:, 2])
            # x, y, z = xyz[min_z_idx]
            ar = np.array([x, y , z])
            print("The distance is: ", np.linalg.norm(ar))
            temp=PointAdd([x,y,z])
            rospy.spin()
            #rospy.sleep(2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf error")
    except KeyboardInterrupt:
        rospy.loginfo("closing")

