#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学库使用实例(Example of using the kinematics library for a robotic arm)
import arm_kinematics.transform as transform
from arm_kinematics.forward_kinematics import ForwardKinematics
from arm_kinematics.inverse_kinematics import get_ik, get_position_ik, set_link, get_link, set_joint_range, get_joint_range

##########forward_kinematics##################
fk = ForwardKinematics(debug=True)  # 实例化正运动学，开启打印(Instantiate forward kinematics and enable printing)

print('当前各连杆长度(m):', fk.get_link())  # 详细说明请参考transform里的注释(For specific instruction, please refer to the notes in transform)
print('当期各关节范围(deg):', fk.get_joint_range('deg'))  #以角度为单位返回(Returns the range of each joint in degrees)
pulse = transform.pulse2angle([500, 500, 500, 500, 500])  # 舵机脉宽值转为弧度(Convert servo pulse width values to radians)
print('input:', pulse)
res = fk.get_fk(pulse)  #获取运动学正解(Obtain forward kinematics solusion)
print('output:', res)
print('rpy:', transform.qua2rpy(res[1]))
# 设置连杆长度(m)base_link, link1, link2, link3, tool_link(Set the length of each link (m) base_link, link1, link2, link3, and tool_link)
fk.set_link(0.127, 0.08, 0.08, 0.052, 0.112)  

# 设置关节范围(deg)joint1, joint2, joint3, joint4, joint5(Set the range of each joint (deg) joint1, joint2, joint3, joint4, and joint5)
fk.set_joint_range([-90, 0], [-90, 0], [-90, 0], [-90, 0], [-90, 0], 'deg')  
print('当前各连杆长度(m):', fk.get_link())  # 详细说明请参考transform里的注释(For specific instruction, please refer to the notes in transform)
print('当期各关节范围(deg):', fk.get_joint_range('deg'))

print('---------------------------------------------------------------------')
##########inverse_kinematics##################
print('当前各连杆长度(m):', get_link())  # 详细说明请参考transform里的注释(For specific instruction, please refer to the notes in transform)
print('当期各关节范围(deg):', get_joint_range('deg'))  #以角度为单位返回(Returns the range of each joint in degrees)
# x, y, z(m), roll, pitch, yaw(deg)获取运动学逆解(Get inverse kinematics by specifying x, y, z (m), roll, pitch, and yaw (deg))
res = get_position_ik(0.2, 0, 0.2, 0, 0, 0)  
if res != []:
    pulse = transform.angle2pulse(res)  # 转为舵机脉宽值(Convert to servo pulse width values)
    for i in range(len(pulse)):
        print('output%s:'%(i + 1), pulse[i])
else:
    print('no solution')

# [x, y, z(m)], pitch, [pitch_min, pitch_max](deg)获取运动学逆解(Get inverse kinematics by specifying [x, y, z] (m), pitch, and [pitch_min, pitch_max] (deg))
res = get_ik([0.2, 0.0, 0.22], 0, [-180, 180])
if res != []:
    for i in range(len(res)):
        print('rpy%s:'%(i + 1), res[i][1])  # 解对应的rpy值(The corresponding rpy values for each solution)
        pulse = transform.angle2pulse(res[i][0])  # 转为舵机脉宽值(Convert to servo pulse width value)
        for j in range(len(pulse)):
            print('output%s:'%(j + 1), pulse[j])
else:
    print('no solution')
# 设置连杆长度(m)base_link, link1, link2, link3, tool_link(Set the length of each link (m) base_link, link1, link2, link3, and tool_link)
set_link(0.185, 0.08, 0.08, 0.052, 0.112)   

# 设置关节范围(deg)joint1, joint2, joint3, joint4, joint5(Set the range of each joint (deg) joint1, joint2, joint3, joint4, and joint5)
set_joint_range([-90, 0], [-90, 0], [-90, 0], [-90, 0], [-90, 0], 'deg')  
print('当前各连杆长度(m):', get_link())  # 详细说明请参考transform里的注释(For specific instruction, please refer to the notes in transform)
print('当期各关节范围(deg):', get_joint_range('deg'))

