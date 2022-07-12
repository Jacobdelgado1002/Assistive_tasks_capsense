# -------------------------------------------------------
# used for visualizing poinclouds (file.ply) obtained 
# with the depth camera
# -------------------------------------------------------

import open3d as o3d # installed by running: <pip install open3d-python>

# def img_to_pointcloud(img, depth, K, Rt):
#     rgb = o3d.geometry.Image(img)
#     depth = o3d.geometry.Image(depth)
#     rgbd = o3d.geometry.create_rgbd_image_from_color_and_depth(rgb, depth, depth_scale=1.0, depth_trunc=50.0)
#     fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
#     intrinsic = o3d.camera.PinholeCameraIntrinsic(int(cx*2), int(cy*2), fx, fy, cx, cy)
#     pc = o3d.create_point_cloud_from_rgbd_image(rgbd, intrinsic, Rt)
#     o3d.visualization.draw_geometries([pc])

pcd = o3d.io.read_point_cloud("Point_clouds/2022-07-09.ply")

o3d.visualization.draw_geometries([pcd])