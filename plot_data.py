from posixpath import split
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from sklearn import preprocessing 
from scipy.signal import lfilter 
from scipy.signal import savgol_filter 
from scipy.signal import firwin
from scipy.fftpack import fft, fftfreq

x = []
y1 = []
y2 = []
y3 = []
y4 = []
y5 = []
y6 = []




# task flag
# 1 = plot the datasets in 3-D
# 2 = plot the distance from the electrodes to the selected point
# 3 = plot frequency
# default(else) = plot capacitance data

flag = 0

if flag == 1:
    # plot the datasets in 3-D
    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode1.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y1.append(np.linalg.norm(dist))
        y1.append(dist)

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode2.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y2.append(np.linalg.norm(dist))
        y2.append(dist)

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode3.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y3.append(np.linalg.norm(dist))
        y3.append(dist)

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode4.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y4.append(np.linalg.norm(dist))
        y4.append(dist)

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode5.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y5.append(np.linalg.norm(dist))
        y5.append(dist)

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-08/dataset1/electrode6.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        # y6.append(np.linalg.norm(dist))
        y6.append(dist)

    ax = plt.axes(projection='3d')
    size=50
    ax.scatter([ele[0] for ele in y1], [ele[1] for ele in y1], [ele[2] for ele in y1], label='link_aruco_top_wrist', s=size)
    ax.scatter([ele[0] for ele in y2], [ele[1] for ele in y2], [ele[2] for ele in y2], label='link_aruco_right_base', s=size)
    ax.scatter([ele[0] for ele in y3], [ele[1] for ele in y3], [ele[2] for ele in y3], label='link_aruco_left_base', s=size)
    ax.scatter([ele[0] for ele in y4], [ele[1] for ele in y4], [ele[2] for ele in y4], label='link_aruco_shoulder', s=size)
    ax.scatter([ele[0] for ele in y5], [ele[1] for ele in y5], [ele[2] for ele in y5], label='electrode2', s=size)
    ax.scatter([ele[0] for ele in y6], [ele[1] for ele in y6], [ele[2] for ele in y6], label='selected_point', s=size)

    ax.legend()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

elif flag == 2:
    # plot the distance from the electrodes to the selected point

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode1.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y1.append(np.linalg.norm(dist))

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode2.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y2.append(np.linalg.norm(dist))

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode3.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y3.append(np.linalg.norm(dist))

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode4.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y4.append(np.linalg.norm(dist))

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode5.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y5.append(np.linalg.norm(dist))

    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-08-02/dataset1/electrode6.txt', 'r'):
        lines = [i for i in line.split()]
        dist = np.array([float(lines[0]), float(lines[1]), float(lines[2])])
        y6.append(np.linalg.norm(dist))

    fig, ax = plt.subplots()


    ax.set_title("Distance vs Time (s)")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance')

    x = range(len(y1))

    
    ax.plot(x, y1, label='electrode1')
    ax.plot(x, y2, label='electrode2')
    ax.plot(x, y3, label='electrode3')
    ax.plot(x, y4, label='electrode4')
    ax.plot(x, y5, label='electrode5')
    ax.plot(x, y6, label='electrode6')
    ax.legend()

elif flag == 4:
    # frequency graph
    sample_len1 = len(y1)
    fft_result = fft(y1 - np.mean(y1))
    yf = 2.0/sample_len1 * np.abs(fft_result[0:sample_len1//2])
    print(np.amax(yf))
    plt.plot(yf)

else:
    # plot capacitance data
    for line in open('/home/jacob/catkin_ws/src/jacob_package/scripts/data_collected/translations2022-07-22/straight_through_Arm/dataset1/capacitance_data1.txt', 'r'):
        lines = [i for i in line.split()]
        y1.append(int(lines[0]))
        y2.append(int(lines[1]))
        y3.append(int(lines[2]))
        y4.append(int(lines[3]))
        y5.append(int(lines[4]))
        y6.append(int(lines[5]))
        
    fig, ax = plt.subplots()

    

    ax.set_title("Capacitance vs Time (s)")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Capacitance')
    x = range(len(y1))

    yy1 = savgol_filter(y1, 101, 2)
    yy2 = savgol_filter(y2, 101, 2)
    yy3 = savgol_filter(y3, 101, 2)
    yy4 = savgol_filter(y4, 101, 2)
    yy5 = savgol_filter(y5, 101, 2)
    yy6 = savgol_filter(y6, 101, 2)


    y_filtered1 = preprocessing.minmax_scale(yy1, feature_range=(0, 1), axis=0, copy=True)
    y_filtered2 = preprocessing.minmax_scale(yy2, feature_range=(0, 1), axis=0, copy=True)
    y_filtered3 = preprocessing.minmax_scale(yy3, feature_range=(0, 1), axis=0, copy=True)
    y_filtered4 = preprocessing.minmax_scale(yy4, feature_range=(0, 1), axis=0, copy=True)
    y_filtered5 = preprocessing.minmax_scale(yy5, feature_range=(0, 1), axis=0, copy=True)
    y_filtered6 = preprocessing.minmax_scale(yy6, feature_range=(0, 1), axis=0, copy=True)


    ax.plot(x, y_filtered2, label='electrode1', c = 'r')
    ax.plot(x, y_filtered5, label='electrode2', c = 'c')
    ax.plot(x, y_filtered6, label='electrode3', c = 'm')
    ax.plot(x, y_filtered1, label='electrode4', c = 'b')
    ax.plot(x, y_filtered4, label='electrode5', c = 'y')
    ax.plot(x, y_filtered3, label='electrode6', c = 'g')

    ax.legend()
    # ax.plot(x, y_filtered1, label='electrode1', c = 'b')
    # ax.plot(x, y_filtered2, label='electrode2', c = 'r')
    # ax.plot(x, y_filtered3, label='electrode3', c = 'g')
    # ax.plot(x, y_filtered4, label='electrode4', c = 'y')
    # ax.plot(x, y_filtered5, label='electrode5', c = 'c')
    # ax.plot(x, y_filtered6, label='electrode6', c = 'm')




# # sample spacing

# T = 1.0 / 800.

# yf = fft(y1)

# xf = fftfreq(N, T)[:N//2]

# plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))

# n = 15  # the larger n is, the smoother curve will be
# b = [1.0 / n] * n
# a = 1
# yy = lfilter(b,a,y1)

# ax.plot(x, yy, linewidth=2, linestyle="-", c="b")

# ax.plot(x, Y_minmax, 'b')  # high frequency noise removed




plt.tight_layout()
plt.grid()
plt.show()