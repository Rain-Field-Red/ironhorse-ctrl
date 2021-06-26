

from simulation_process import VrepProcess


def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.01, 100)

    baseName = ['Body']
    jointName = ['Joint_HL1','Joint_HF1','Joint_Knee1','Joint_HL2','Joint_HF2','Joint_Knee2','Joint_HL3','Joint_HF3','Joint_Knee3','Joint_HL4','Joint_HF4','Joint_Knee4']
    mainProcess.simGetHandle(baseName, jointName)

    mainProcess.simStart()

    try:
        while mainProcess.m_IsStart:
            currentTime, basePos, jointConfig = mainProcess.simGetInfo()

            print(currentTime)
            print(basePos)
            print(jointConfig)

            mainProcess.simNextStep()
            
        mainProcess.simStop()       #仿真完成后停止

    except KeyboardInterrupt:
        print('KeyboardInterrupt')

        mainProcess.simStop()       #ctrl+c后停止





if __name__ == '__main__':
    main()