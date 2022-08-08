# -------------------------------------------------------
# capture depth image of target(s)
# -------------------------------------------------------

# alternate method for capturing depth image utilzing .ply files

# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# import rospy
# import open3d
# from datetime import date

# rospy.sleep(5)
# #Camera Setup
# # Create a context object. This object owns the handles to all connected realsense devices
# pipe = rs.pipeline()
# # Configure streams
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color)
# # Start streaming
# pipe.start(config)
# #Image Capture
# for x in range(5):
#     pipe.wait_for_frames()
# frameset=pipe.wait_for_frames()
# color_frame = frameset.get_color_frame()
# depth_frame = frameset.get_depth_frame()

# print("Frames Captured")
# color = np.asanyarray(color_frame.get_data())
# colorizer = rs.colorizer()
# # Create alignment primitive with color as its target stream:
# align = rs.align(rs.stream.color)
# frameset = align.process(frameset)

# from datetime import date

# today = date.today()
# #Getting ply file
# ply = rs.save_to_ply("Point_clouds/"+str(today)+".ply")
# ply.set_option(rs.save_to_ply.option_ply_binary, False) #?
# ply.set_option(rs.save_to_ply.option_ply_normals, True) #?
# ply.process(colorizer.process(frameset))

# pipe.stop()

import pyrealsense2 as rs
import numpy as np
from datetime import date

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
# decimate = rs.decimation_filter()
# decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()

align_to = rs.stream.color
align = rs.align(align_to)

# Grab camera data
# Wait for a coherent pair of frames: depth and color
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()


depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Grab new intrinsics (may be changed by decimation)
depth_intrinsics = rs.video_stream_profile(
    depth_frame.profile).get_intrinsics()

w, h = depth_intrinsics.width, depth_intrinsics.height

depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
real_depth = depth_image * depth_scale
depth_to_show = np.dstack([real_depth]*3)
real_color = (color_image / 255)[:, :, ::-1]

depth_colormap = np.asanyarray(
    colorizer.colorize(depth_frame).get_data())

mapped_frame, color_source = depth_frame, depth_colormap

points = pc.calculate(depth_frame)
pc.map_to(mapped_frame)

# Pointcloud data to arrays
v, t = points.get_vertices(), points.get_texture_coordinates()
verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
pc_reshaped = np.asanyarray(v).view(np.float32).reshape((depth_image.shape[0], depth_image.shape[1], 3))
# pc_reshaped should be the point cloud data you need

print(type(pc_reshaped))
pc = pc_reshaped[real_depth > 0].reshape(-1, 3)
pc = pc[pc[:, 2] < 1]

# from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# ax = plt.axes(projection='3d')
# ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2])
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# plt.show()

# print(pc.shape)
np.save("./tmp_pc.npy", pc)

# today = date.today()
# #Getting ply file
# ply = rs.save_to_ply("Point_clouds/"+str(today)+".ply")
# ply.set_option(rs.save_to_ply.option_ply_binary, False) #?
# ply.set_option(rs.save_to_ply.option_ply_normals, True) #?
# ply.process(colorizer.process(aligned_frames))

pipeline.stop()