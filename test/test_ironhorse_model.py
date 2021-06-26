
import numpy
import time

import pybullet as p
import pybullet_data

import pyrbdl as rbdl
#import pydrake


freq = 500

get_last_vel = [0] * 3
motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
init_new_pos = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def get_data_from_sim():
    global get_last_vel
    get_orientation = []
    get_matrix = []
    get_velocity = []
    get_invert = []
    imu_data = [0] * 10
    leg_data = [0] * 24

    pose_orn = p.getBasePositionAndOrientation(boxId)

    for i in range(4):
        get_orientation.append(pose_orn[1][i])
    # get_euler = p.getEulerFromQuaternion(get_orientation)
    get_velocity = p.getBaseVelocity(boxId)
    get_invert = p.invertTransform(pose_orn[0], pose_orn[1])
    get_matrix = p.getMatrixFromQuaternion(get_invert[1])

    # IMU data
    imu_data[3] = pose_orn[1][0]
    imu_data[4] = pose_orn[1][1]
    imu_data[5] = pose_orn[1][2]
    imu_data[6] = pose_orn[1][3]

    imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * \
        get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
    imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * \
        get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
    imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * \
        get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

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

    # joint data
    joint_state = p.getJointStates(boxId, motor_id_list)
    leg_data[0:12] = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                      joint_state[3][0], joint_state[4][0], joint_state[5][0],
                      joint_state[6][0], joint_state[7][0], joint_state[8][0],
                      joint_state[9][0], joint_state[10][0], joint_state[11][0]]

    leg_data[12:24] = [joint_state[0][1], joint_state[1][1], joint_state[2][1],
                       joint_state[3][1], joint_state[4][1], joint_state[5][1],
                       joint_state[6][1], joint_state[7][1], joint_state[8][1],
                       joint_state[9][1], joint_state[10][1], joint_state[11][1]]
    com_velocity = [get_velocity[0][0],
                    get_velocity[0][1], get_velocity[0][2]]
    # get_last_vel.clear()
    get_last_vel = []
    get_last_vel = com_velocity

    return imu_data, leg_data, pose_orn[0]

p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.resetSimulation()
p.setTimeStep(1.0/freq)
p.setGravity(0, 0, -9.8)

robot_start_pos = [0, 0, 0.42]
boxId = p.loadURDF("../models/iron_horse/iron_horse_simple.urdf", robot_start_pos,
                       useFixedBase=False)

jointIds = []
for j in range(p.getNumJoints(boxId)):
    p.getJointInfo(boxId, j)
    jointIds.append(j)

p.resetBasePositionAndOrientation(
        boxId, [0, 0, 0.4], [0, 0, 0, 1])


p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0])
for j in range(12):
    p.resetJointState(boxId, motor_id_list[j], init_new_pos[j], init_new_pos[j+12])


for _ in range(10):
    p.stepSimulation()
    imu_data, leg_data, _ = get_data_from_sim()
    print(leg_data)
    time.sleep(1)




