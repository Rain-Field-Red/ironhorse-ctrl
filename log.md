# 项目过程



本文档记录项目建立的过程。



## step1 构建IronHorse的vrep模型

### 建立模型

IronHorse的vrep模型包括两个部分，一个是模型本身，另外就是仿真接口。

（1）IronHorse模型用简单形状构建，并使用stl文件的蒙皮。

（2）使用python编制了simulation_process类，简化主流程中的代码。

### 进行测试

编制sim_IronHorse.py文件，初步实现运行仿真和读取机器人状态的循环流程。



## step2 构建mini_cheetah的vrep模型

考虑到整个项目设计到仿真环境的替换、仿真对象的替换、编译过程的调整和算法模块的使用等多重因素，直接对IronHorse的vrep模型进行控制要调整的因素较多，因此考虑构建mini_cheetah的vrep模型。

这样，在对mini_cheetah的vrep模型进行控制仿真时，只需要调整编译过程。

### 建立模型

从chvmp/mini-cheetah-gazebo-urdf项目中得到mini_cheetah的urdf模型，并将其导入到vrep中。

### 精简模型

原模型中包括传感器、摄像机等附件。去掉，以便减少不必要的错误。



## step3 编译控制算法模块

对于cheetah-software包含的控制算法，能否模块化使用是本项目成功的核心。将大大降低控制开发的门槛。

### 准备算法程序

在初次确定能否编译的过程中，完全使用与参考项目一致的算法程序。

将其src子文件夹拷贝到项目文件夹中即可。

### 改写CMakeLists.txt文件

由于参考项目使用catkin_make对程序进行编译，与cmake存在很大的一致性，因此只需要在CMakeLists.txt文件中去掉ros相关的部分，并改写一些关键字即可。



## step4 基于vrep模型复现pubullet中的控制流程

在准备vrep中的mini_cheetah模型和控制算法模块动态连接诶库以后，用python语言编制控制流程，实现仿真环境配置、机器人信号获取、控制计算、控制信号输出的过程。

### 模型对照

在编制python程序的过程中，发现vrep中的mini_cheetah模型与控制算法中的模型在关节顺序和关节轴方向上有所区别。通过查看参考项目中的pybullet模型，确定了关节顺序和关节轴方向。并使用两个参数列表，实现了采集顺序和方向变换的过程。

```python
jointName = ['FR_hip_joint','FR_thigh_joint','FR_calf_joint',\
        'FL_hip_joint','FL_thigh_joint','FL_calf_joint',\
            'RR_hip_joint','RR_thigh_joint','RR_calf_joint',\
                'RL_hip_joint','RL_thigh_joint','RL_calf_joint']
direction = [1.0,-1.0,-1.0,1.0,-1.0,-1.0,1.0,-1.0,-1.0,1.0,-1.0,-1.0]


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
    
control_force = tau.contents.eff
for i in range(12):
    control_force[i] = direction[i]*control_force[i]
```

### 控制频率设置

cheetah-software和参考项目中，控制频率均为500Hz，而对应的MPC调用间隔是13。也就是说dtMPC为0.026s。尝试改变控制频率，发现不可行。

后续如果要改变，需要进行更深入的考察。

### 设置机器人初始位置

pybullet中可以对机器人初始位置进行设置。

在vrep尝试相同的操作时，发现提供的python接口中没有对应的内容（c语言接口是有的），因此改为直接在模型中操作，没有通过编程实现。

### python语言与c语言通信

参考项目中使用ctypes实现python语言与c语言的通信，为解决这一问题提供了很好的借鉴。

其中有一个需要注意的问题。就是不同计算机上python的默认数据类型似乎不一致，因此在使用其中convert_type()函数前，需要首先进行类型转换。因此我在仿真环境中读取到机器人状态数据后，都进行了强制类型转换。

```python
imu_data_t = list(map(float, imu_data))
leg_data_t = list(map(float, leg_data))
base_pos_t = list(map(float, base_pos))
```



## step5 基于mpc控制量构造位置规划

液压驱动的机器人难以实现关节力矩控制。

为了将mpc应用到IronHorse上，需要改造当前基于mpc的控制器。

第一步的设想是利用mpc给出的控制量来构造关节位置期望。

### 查看cheetah-software控制量生成时的调用路径

（1）基于mpc的规划生成足端期望位置和足端期望力，并指定肢体控制增益

（2）肢体控制在摆动期和支撑期分别计算关节力矩

### 修改肢体控制算法

通过考察，发现肢体控制器LegController中，控制量的构造是笛卡尔空间下的pd加上足端期望力
$$
u = K_p(p_{des}-p) + K_d(v_{des}-v) + force_{ff}
$$
这与基于位置控制构造阻抗的思路存在一致性，因为如果不考虑真实力，将期望力构造成位置期望的增量，然后再实现位置控制的话，有
$$
\delta u = k*force_{ff} \\
p_{com} = p_{des}+\delta u \\
$$
最终控制量为
$$
u = K_p(p_{com}-p) + K_d(v_{des}-v) \\
=K_p(p_{des}-p) + K_d(v_{des}-v) + K_p*k*force_{ff}
$$
因此，在LegController的cpp程序中，尝试如下改变：

（1）将支撑期增益改成与摆动期相同。

由于摆动期的目标是位置控制，这个改动可以看作是支撑期也进行位置控制。

（2）改变$K_p*k$的值。

当整体增益为$b$时，可知$k$的数值为$\frac{b}{K_p}$。



## step6 针对IronHorse编制ihGaitCtrller类

cheetah-software中使用共享内存实现各模块的信息传递。参考项目中保持了模块化，但并没有使用共享内存传递信息。而是构建了GaitCtrller类，在该类中实例化各种功能类，并以类成员变量的形式实现信息传递。

```c++
class GaitCtrller {
 public:
  GaitCtrller(double freq, double* PIDParam);
  ~GaitCtrller();
  void SetIMUData(double* imuData);
  void SetLegData(double* motorData);
  void PreWork(double* imuData, double* motorData);
  void SetGaitType(int gaitType);
  void SetRobotMode(int mode);
  void SetRobotVel(double* vel);
  void ToqueCalculator(double* imuData, double* motorData, double* effort);

 private:
  int _gaitType = 0;
  int _robotMode = 0;
  bool _safetyCheck = true;
  std::vector<double> _gamepadCommand;
  Vec4<float> ctrlParam;

  Quadruped<float> _quadruped;
  ConvexMPCLocomotion* convexMPC;
  LegController<float>* _legController;
  StateEstimatorContainer<float>* _stateEstimator;
  LegData _legdata;
  LegCommand legcommand;
  ControlFSMData<float> control_data;
  VectorNavData _vectorNavData;
  CheaterState<double>* cheaterState;
  StateEstimate<float> _stateEstimate;
  RobotControlParameters* controlParameters;
  DesiredStateCommand<float>* _desiredStateCommand;
  SafetyChecker<float>* safetyChecker;
};
```

这里参考上面的方法，实现一个ihGaitCtrller类。考虑到对于IronHorse进行控制时，算法中的模型也要进行调整，因此不仅是新建一个文集，而是要改变很多参数。所以在src文件夹下新建一个文件夹。拟将需要引用的文件都在其中保存，形成一个独立的副本。为防止重名，所有的文件名之前都加ih标识。



## step7 修改控制算法中的模型



原控制算法中的模型计算时完全定制化的。

为了提高运动学计算的通用性，拟通过机器人动力学库读取urdf模型，并根据机器人状态计算运动学的方式实现。



### iron horse的urdf模型



根据mini-cheetah的urdf模型修改得到，利用pybullet对urdf模型的读取和显示完成修改。



### 模型的运动学计算

模型参数的保存仍使用quadruped类。但正运动学使用rbdl类实现。

#### 模型参数

编制ironhorse类

#### 正运动学

drake本书不是独立的动力学库，有点笨重了。rbdl和pinocchio都可以。























