# -------------------------------------------------------
# capture depth image of target(s)
# -------------------------------------------------------

import pyrealsense2 as rs
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
from datetime import date

rospy.sleep(5)
#Camera Setup
# Create a context object. This object owns the handles to all connected realsense devices
pipe = rs.pipeline()
# Configure streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color)
# Start streaming
pipe.start(config)
#Image Capture
for x in range(5):
    pipe.wait_for_frames()
frameset=pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

print("Frames Captured")
color = np.asanyarray(color_frame.get_data())
colorizer = rs.colorizer()
# Create alignment primitive with color as its target stream:
align = rs.align(rs.stream.color)
frameset = align.process(frameset)

from datetime import date

today = date.today()
#Getting ply file
ply = rs.save_to_ply("Point_clouds/"+str(today)+".ply")
ply.set_option(rs.save_to_ply.option_ply_binary, False) #?
ply.set_option(rs.save_to_ply.option_ply_normals, True) #?
ply.process(colorizer.process(frameset))

pipe.stop()