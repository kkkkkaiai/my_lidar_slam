#!/usr/bin/python

import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from collections import Counter

first = True

lowerBound, upperBound, beam = -45., 45., 64
factor = (beam - 1) / (upperBound - lowerBound)


def get_ring_for_angle(angle):
    return np.floor(((angle * 180 / np.pi) - lowerBound) * factor)


def os_callback(msg):
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    lidar_points = np.asarray(list(points))
    t_lidar_points = lidar_points.T
    x, y, z = t_lidar_points

    angles = np.round(np.arctan(np.divide(z, np.sqrt(x*x + y*y)+0.000001)), decimals=3)
    invalid_angles = np.isnan(angles)
    angles = np.delete(angles, np.where(invalid_angles))

    index = get_ring_for_angle(angles)
    angle_statistic = Counter(index)
    # sorted(angle_statistic.elements())
    # print(angle_statistic)
    global first
    # temp = sorted(angle_statistic)
    # print(temp)
    # if first:

    # print(angle_statistic.most_common(70))
    temp = np.array(sorted(angle_statistic.most_common(150)))
    print(temp[:, 0])
#     for index, data in enumerate(angle_statistic):
#         print(index, data, )
#         temp.append([data])
#     temp = sorted(temp)
#     temp = np.asarray(temp)
#     for i in range(len(temp)):
#         # print(temp[i])
#         pass
#     first = False

def main():
    rospy.init_node('stastic_ouster', anonymous=True)
    os_lidar_sub = rospy.Subscriber('/os_cloud_node/points', PointCloud2, os_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
