#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/01/31
# @author:aiden
# 判断位置是否发生位置变化(detect whether the position has changed)
import math
import numpy as np

def calculate_e_distance(point1, point2):
    # 计算两个点间的欧式距离(calculate the Euclidean distance between two points)
    e_distance = int(round(math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))))

    return e_distance


def position_change_or_not(last_point, current_points, distance):
    # 将上一次的某点与当前所有点对比，检测是否有距离符合设定的点，即当作位置没有改变的点(compare a certain point from the last time with all current points to detect if there are any points that meet the set distance criteria, treating them as points where the position has not changed)
    for p in current_points:
        if last_point[0][:-1] == p[0][:-1]:
            dis = calculate_e_distance(last_point[1], p[1])
            if dis < distance:
                current_points.remove(p)
                p[0] = last_point[0]
                return False, p, current_points

    return True, None, current_points


def position_reorder(current_points, last_points, distance=10):
    # distance 单位像素(distance unit pixel)
    # 对比上一次和当前点的位置，如果位置没有改变，则相应的颜色标签不变，否则从1开始重新安排标签序号(compare the positions of points from the previous and current iterations. If the position remains unchanged, the corresponding color label remains the same; otherwise, reassign label numbers starting from 1)
    new_points = []
    haved_change_points = []
    for p in last_points:  # 对上一次的所有点和当前点进行位置对比(compare the positions of all points from the previous iteration with the current points)
        res, not_change_point, haved_change_points = position_change_or_not(p, current_points, distance)
        if not res:  # 如果没有改变，就将此点记录下来作为重新排序的新点(if there is no change, record this point as a new point for reordering)
            new_points.extend([not_change_point])
    if haved_change_points != [] and new_points != []:
        names = np.array(new_points, dtype=object)[:, 0].tolist()
        for p in haved_change_points:
            index = 0
            while True:
                new_name = p[0][:-1]
                index += 1
                new_name += str(index)
                if new_name not in names:
                    p[0] = new_name
                    new_points.extend([p])
                    names.append(new_name)
                    break

    return new_points
