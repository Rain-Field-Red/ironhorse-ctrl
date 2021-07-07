# program for IronHorse



基于mini-cheetah的代码，改造出适合ironhorse的算法库。



## 动力学

mini-cheetah的动力学代码在Dynamics子文件夹中。

包括：

- ActuatorModel
- FloatingBaseModel
- Quadruped
- spatial
- SpatialInertia
- MiniCheetah

其中MiniCheetah程序提供了一个构造四足模型的函数，返回一个Quadruped对象。其他程序都可以复用，只需要重新为IronHorse编制一个构造模型的函数即可.

### 源码分析

查看源码时，发现动力学基本是为仿真服务的。在控制中，似乎直接给定模型参数即可。



### 编码流程

（1）增加一个机器人类型RobotType::IRON_HORSE

（2）根据Vrep模型改写机器人杆件参数，构造IronHorse.h文件

（3）程序中只用了运动学模型，关键在于获得肢体坐标系（髋部）的位置，以及运动学正解和雅可比矩阵的计算。

运动学模型计算在LegController中完成。然而，程序中构建的时针对mini-cheetah的计算过程，并不是通用的。即使这次针对ironhorse实现对应的计算过程，下一次需要控制另外的机器人时就需要在一次编码，因此考虑使用rbdl库，通过机器人的urdf模型来实现计算。

（4）建立ironhore的urdf模型









