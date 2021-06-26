
import numpy as np
import math
import sim as vrep
import time


class VrepProcess:
    def __init__(self):
        self.m_baseName = []
        self.m_jointName = []

        self.m_baseHandle = []
        self.m_jointHandle = []

        self.m_baseNum = 0
        self.m_jointNum = 0

        self.m_tStep = 0
        self.m_N = 0

        self.m_count = 0
        self.m_IsStart = False

        self.m_clientID = -1

    def simConnect(self):
        vrep.simxFinish(-1)
        # 每隔0.2s检测一次，直到连接上V-rep
        while True:
            self.m_clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
            if self.m_clientID > -1:
                break
            else:
                time.sleep(0.2)
                print("Failed connecting to remote API server!")
        print("Connection success!")

    def simSetup(self, tStep, N):
        self.m_tStep = tStep
        self.m_N = N

        # 然后打开同步模式
        vrep.simxSynchronous(self.m_clientID, True) 
        # 设置仿真步长，为了保持API端与V-rep端相同步长
        vrep.simxSetFloatingParameter(self.m_clientID, vrep.sim_floatparam_simulation_time_step, tStep, vrep.simx_opmode_blocking)

    def simGetHandle(self, baseName, jointName):
        """
        获取vrep中的对象句柄
        """
        self.m_baseName = baseName.copy()
        self.m_jointName = jointName.copy()

        self.m_baseNum = len(baseName)
        self.m_jointNum = len(jointName)

        for i in self.m_baseName:
        #for i in range(self.m_baseNum):
             _, baseHandle = vrep.simxGetObjectHandle(self.m_clientID, i, vrep.simx_opmode_oneshot_wait)
             self.m_baseHandle.append(baseHandle)

        for j in self.m_jointName:
        #for j in range(self.m_jointNum):
            _, jointHandle = vrep.simxGetObjectHandle(self.m_clientID, j, vrep.simx_opmode_blocking)
            print(jointHandle)
            self.m_jointHandle.append(jointHandle)



        print('Handles available!')
        print('baseHandles:', self.m_baseHandle)
        print('jointHandles:', self.m_jointHandle)
        #print(self.m_jointHandle)
        print(self.m_jointName)

    def simStart(self):
        
        self.m_IsStart = True

        # 启动仿真
        vrep.simxStartSimulation(self.m_clientID, vrep.simx_opmode_oneshot)

        time.sleep(1)
        print('start simulation')

        baseHandle = self.m_baseHandle[0]
        jointHandle = self.m_jointHandle[0]
        print(baseHandle)
        print(jointHandle)

        #lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
        _, base_pos = vrep.simxGetObjectPosition(self.m_clientID, baseHandle, -1, vrep.simx_opmode_oneshot)
        _, jointConfig = vrep.simxGetJointPosition(self.m_clientID, jointHandle, vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(self.m_clientID)  # 让仿真走一步

        time.sleep(1)
        print('trigger once')

    def simStop(self):
        vrep.simxStopSimulation(self.m_clientID, vrep.simx_opmode_blocking)

        print('stop')

    def simGetInfo(self):
        clientID = self.m_clientID
        baseHandle = self.m_baseHandle
        jointHandle = self.m_jointHandle

        currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间

        baseConfig = []
        jointConfig = []
        for i in baseHandle:
            _, base_pos = vrep.simxGetObjectPosition(clientID, i, -1, vrep.simx_opmode_oneshot)
            baseConfig.append(base_pos)

        for j in jointHandle:
            _, joint_pos = vrep.simxGetJointPosition(clientID, j, vrep.simx_opmode_oneshot)
            jointConfig.append(joint_pos)

        return currCmdTime, baseConfig, jointConfig


    def simSetCmd(self, control_force):
        clientID = self.m_clientID
        jointHandle = self.m_jointHandle

        # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
        vrep.simxPauseCommunication(clientID, True)

        for i in range(len(jointHandle)):
            if np.sign(control_force[i]) >= 0:
                set_force = control_force[i]
                set_velocity = 9999
            else:
                set_force = -control_force[i]
                set_velocity = -9999

            vrep.simxSetJointTargetVelocity(clientID, jointHandle[i], set_velocity, vrep.simx_opmode_oneshot)
            vrep.simxSetJointForce(clientID, jointHandle[i], set_force, vrep.simx_opmode_oneshot)

        vrep.simxPauseCommunication(clientID, False)

    def simNextStep(self):
        if self.m_count <= self.m_N:
            vrep.simxSynchronousTrigger(self.m_clientID)  # 让仿真走一步
            self.m_count += 1    
        else:
            self.m_IsStart = False



