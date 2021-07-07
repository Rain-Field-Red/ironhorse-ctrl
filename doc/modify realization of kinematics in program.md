# modify realization of kinematics in program



rbdl可以使用后，利用它读取urdf文件构造模型的能力，可以使程序更加通用。



通过查看源程序代码，发现运动学计算是由LegController类实现的，其他功能类需要运动学数据时都是读取该类的计算结果。因此，首先调整LegController类。

## LegController类的调整

LegController类在

- LegController.h
- LegController.cpp

中实现。

为了不影响原来的程序功能，对将原来的代码拷贝到ihGaitCtrller文件夹下的子文件夹ihControllers下，并改成

- ihLegController.h
- ihLegController.cpp

将LegController类改成ihLegController类。并在ihLegController类中增加rbdl的Model类指针变量，并在ihLegController类的构造函数中新开辟变量空间，在析构函数中清除变量空间。



## 相关程序文件的调整

原来使用LegController类的程序文件要改成使用ihLegController类。

同样，为了不影响原来的程序功能，对将原来的代码拷贝到ihGaitCtrller文件夹下，并改名

- ihControlFSMData.h
- ihSafetyChecker.h
- ihSafetyChecker.cpp
- ihConvexMPCLocomotion.h
- ihConvexMPCLocomotion.cpp

将其中使用LegController类的地方调整为使用ihLegController类，并修改对应的头文件。

需要注意的是，这些相关文件仅改了名字，其中的内容不变。













