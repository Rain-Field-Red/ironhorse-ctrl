#! /usr/bin/python2.7
#coding:utf-8

import numpy as np
import math
import sim as vrep
import time

#import rospy

import os
import ctypes
from pyquaternion import Quaternion as quat

get_last_vel = [0] * 3
jointName = ['Joint_HL2','Joint_HF2','Joint_Knee2',\
        'Joint_HL1','Joint_HF1','Joint_Knee1',\
            'Joint_HL4','Joint_HF4','Joint_Knee4',\
                'Joint_HL3','Joint_HF3','Joint_Knee3']
direction = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]

class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]


def convert_type(input):
    ctypes_map = {int: ctypes.c_int,
                  float: ctypes.c_double,
                  str: ctypes.c_char_p
                  }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length == 0:
            print("convert type failed...input is "+input)
            return 0
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(
                    input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
            return arr
    else:
        if input_type in ctypes_map:
            return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
        else:
            print("convert type failed...input is "+input)
            return 0


def get_data_from_sim():
    print("get data")

    global get_last_vel
    get_orientation = []
    get_matrix = []
    get_velocity = []
    get_invert = []
    imu_data = [0] * 10
    leg_data = [0] * 24

    _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle[0], -1, vrep.simx_opmode_oneshot)
    _, base_orn = vrep.simxGetObjectQuaternion(clientID, baseHandle[0], -1, vrep.simx_opmode_oneshot)
    pose_orn = base_pos + base_orn

    #x,y,z,w
    imu_data[3] = base_orn[0]
    imu_data[4] = base_orn[1]
    imu_data[5] = base_orn[2]
    imu_data[6] = base_orn[3]

    _, vel_lin, vel_ang = vrep.simxGetObjectVelocity(clientID, baseHandle[0], vrep.simx_opmode_oneshot)
    #print(vel_lin)
    #print(vel_ang)
    get_velocity = [vel_lin, vel_ang]
    #print(get_velocity)
    
    get_quaternion = quat(axis=[1,0,0], angle=0)
    get_quaternion[0] = base_orn[3]
    get_quaternion[1] = base_orn[0]
    get_quaternion[2] = base_orn[1]
    get_quaternion[3] = base_orn[2]
    R = get_quaternion.rotation_matrix
    #print(R)
    get_matrix = []
    for i in range(3):
        for j in range(3):
            get_matrix.append(R[i][j])
    #print(get_matrix)

    # imu_data[7] = get_matrix[0] * get_velocity[3] + get_matrix[1] * \
    #     get_velocity[4] + get_matrix[2] * get_velocity[5]
    # imu_data[8] = get_matrix[3] * get_velocity[3] + get_matrix[4] * \
    #     get_velocity[4] + get_matrix[5] * get_velocity[5]
    # imu_data[9] = get_matrix[6] * get_velocity[3] + get_matrix[7] * \
    #     get_velocity[4] + get_matrix[8] * get_velocity[5]

    imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * \
        get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
    imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * \
        get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
    imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * \
        get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

    #print(imu_data[7],imu_data[8],imu_data[9])

    # calculate the acceleration of the robot
    linear_X = (get_velocity[0][0] - get_last_vel[0]) * freq
    linear_Y = (get_velocity[0][1] - get_last_vel[1]) * freq
    linear_Z = 9.8 + (get_velocity[0][2] - get_last_vel[2]) * freq
    imu_data[0] = get_matrix[0] * linear_X + \
        get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
    imu_data[1] = get_matrix[3] * linear_X + \
        get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
    imu_data[2] = get_matrix[6] * linear_X + \
        get_matrix[7] * linear_Y + get_matrix[8] * linear_Z

    jointConfig = []
    jointVelConfig = []
    for h in jointHandle:
        _, jointAng = vrep.simxGetJointPosition(clientID, h, vrep.simx_opmode_oneshot)
        _, jointVel = vrep.simxGetObjectFloatParameter(clientID, h, 2012, vrep.simx_opmode_oneshot)
        jointConfig.append(jointAng)
        jointVelConfig.append(jointVel)
    #print(jointConfig)

    #由于模型中关节方向与mini_cheetah不同，为了使用mini_cheetah
    for i in range(12):
        leg_data[i] = direction[i]*jointConfig[i]
        leg_data[i+12] = direction[i]*jointVelConfig[i]
    #leg_data[0:12] = jointConfig
    #leg_data[12:24] = jointVelConfig

    #print(leg_data)

    get_last_vel = []
    get_last_vel = vel_lin

    #print(type(imu_data[0]))
    imu_data_t = list(map(float, imu_data))
    leg_data_t = list(map(float, leg_data))
    base_pos_t = list(map(float, base_pos))
    #print(type(imu_data_t[0]))
    
    return imu_data_t, leg_data_t, base_pos_t


def reset_robot():

    print("reset")
    
    #robot_z = 0.4

    #num, robotObjects = vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #for h in robotObjects:
    #    vrep.simxResetDynamicObject(h)
    #vrep.simxSetObjectPosition(clientID,baseHandle[0],-1,[0,0,robot_z],vrep.simx_opmode_blocking)

    _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle[0], -1, vrep.simx_opmode_oneshot)
    _, base_orn = vrep.simxGetObjectQuaternion(clientID, baseHandle[0], -1, vrep.simx_opmode_oneshot)
    jointConfig = []
    jointVelConfig = []
    for h in jointHandle:
        _, jointAng = vrep.simxGetJointPosition(clientID, h, vrep.simx_opmode_oneshot)
        _, jointVel = vrep.simxGetObjectFloatParameter(clientID, h, 2012, vrep.simx_opmode_oneshot)
        jointConfig.append(jointAng)
        jointVelConfig.append(jointVel)
    vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步
    time.sleep(1)
    print('trigger once')
    print(base_pos)
    print(jointConfig)
    
    #print(convert_type(freq))
    #print(convert_type([stand_kp, stand_kd, joint_kp, joint_kd]))

    # cpp_gait_ctrller.init_controller(convert_type(
    #    freq), convert_type([stand_kp, stand_kd, joint_kp, joint_kd]))


    for _ in range(10):
        vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步
        imu_data, leg_data, _ = get_data_from_sim()
        # cpp_gait_ctrller.pre_work(convert_type(
        #    imu_data), convert_type(leg_data))

        print(imu_data)
        print(leg_data)

    # cpp_gait_ctrller.set_robot_mode(convert_type(1)) # 
    # cpp_gait_ctrller.set_gait_type(convert_type(4))
    # for _ in range(200):
    #     run()
    #     vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步

    # cpp_gait_ctrller.set_robot_mode(convert_type(0))
    #cpp_gait_ctrller.set_robot_vel(convert_type([0.5, 0.0, 0.0]))


def init_vrep():
    
    print("init")

    global clientID,baseHandle,jointHandle
    robot_start_pos = [0, 0, 0.42]

    vrep.simxFinish(-1)
    # 每隔0.2s检测一次，直到连接上V-rep
    while True:
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if clientID > -1:
            break
        else:
            time.sleep(0.2)
        print("Failed connecting to remote API server!")
    print("Connection success!")

    tStep = 1.0/freq
    print(tStep)

    # 打开同步模式
    vrep.simxSynchronous(clientID, True) 
    # 设置仿真步长，为了保持API端与V-rep端相同步长
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tStep, vrep.simx_opmode_blocking)
    # 启动仿真
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    time.sleep(1)
    print('start simulation')

    baseName = ['Body']
    
    baseHandle = []
    jointHandle = []
    for i in baseName:
        _, handle = vrep.simxGetObjectHandle(clientID, i, vrep.simx_opmode_oneshot_wait)
        baseHandle.append(handle)
    for j in jointName:
        _, handle = vrep.simxGetObjectHandle(clientID, j, vrep.simx_opmode_blocking)
        jointHandle.append(handle)
    
    reset_robot()


def main():
    print('main')

    #cpp_gait_ctrller.set_gait_type(convert_type(0))

    cnt = 0

    try:
        while(True):
            #run()
            vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步

            cnt += 1
            if cnt > 99999999:
                cnt = 99999999

    except KeyboardInterrupt:
        print('KeyboardInterrupt')
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
        print('stop')



if __name__ == '__main__':
    
    #for simulation
    freq = 500.0
    stand_kp = 100.0
    stand_kd = 1.0
    joint_kp = 0.0
    joint_kd = 0.05

    path = os.getcwd()
    so_file = path + "/build/libironhorse_ctrl.so"
    print(os.getcwd())
    print(so_file)

    if(not os.path.exists(so_file)):
        print('cannot find cpp.so file')

    cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
    cpp_gait_ctrller.toque_calculator.restype = ctypes.POINTER(StructPointer)

    print('cpp.so file is loaded')


    init_vrep()

    print(jointHandle)

    main()