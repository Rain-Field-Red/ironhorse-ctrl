# 控制模块的编译过程



要模块化使用cheetah-software中的控制算法程序，需要首先将其编译为一个动态链接库。



这里使用cmake实现编译过程。参考Derek-TH-Wang/quadruped_ctrl中的部分指令。



## 程序部分

由于quadruped_ctrl对于控制算法程序的使用也是编译为独立的动态链接库，因此src子文件夹中的内容完全一样。



## CMakeLists.txt文件



（1）指定cmake版本、工程名、编译类型、编译器版本等常规设置。

这里发现控制算法模块需要c++14才能完成编译，对应ros的版本是Melodic。也就是说，如果用ros的编译系统的话，ubuntu版本应为18以上。

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(ironhorse_ctrl)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
```



（2）指定额外的头文件目录。

这里完全参考quadruped_ctrl项目。

```cmake
include_directories(${INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
  src
)
```



（3）编译依赖库的部分

因为依赖的qpOASES、JCQP、和osqp都是通过cmake进行源码安装的，因此这里只需要增加子文件夹的指令即可。

```cmake
add_subdirectory(src/qpOASES)
add_subdirectory(src/JCQP)
add_subdirectory(src/osqp)
```



（4）定义需要编译的c++链接库

定义一个与工程名同名的动态链接库。给出库中用到的程序文件名。这里参考quadruped_ctrl项目，但是cmake编译时需要使用SHARED关键字，明确说明是动态链接库。否则会编译出静态库。

编译中需要指定依赖库。这里参考quadruped_ctrl项目的写法。

```cmake
## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/MPC_Ctrl/ConvexMPCLocomotion.cpp
  src/MPC_Ctrl/Gait.cpp
  src/MPC_Ctrl/SparseCMPC.cpp
  src/MPC_Ctrl/SparseCMPC_Math.cpp
  src/MPC_Ctrl/OsqpTriples.cpp
  src/MPC_Ctrl/convexMPC_interface.cpp
  src/MPC_Ctrl/SolverMPC.cpp
  src/MPC_Ctrl/RobotState.cpp
  src/Controllers/FootSwingTrajectory.cpp
  src/Controllers/LegController.cpp
  src/Controllers/DesiredStateCommand.cpp
  src/Controllers/OrientationEstimator.cpp
  src/Controllers/PositionVelocityEstimator.cpp
  src/Controllers/SafetyChecker.cpp
  src/Dynamics/Quadruped.cpp
  src/Dynamics/FloatingBaseModel.cpp
  src/GaitCtrller.cpp
)


#target_link_libraries(${PROJECT_NAME}
target_link_libraries(${PROJECT_NAME}
  JCQP
  qpOASES
  osqp
)
```



## 编译过程



对quadruped_ctrl项目的CMakeLists.txt进行改写，并明确了编译器版本和动态链接库关键字后，编译正常实现。

编译操作如下

```
mkdir build
cd build
cmake ..
make
```



编译完成后，build文件夹下生成了与工程同名的动态链接库文件（.so），同时生成了一个子文件夹src，其中有依赖库的动态链接库。







