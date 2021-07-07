# urdf model for iron horse



ironhorse的urdf模型直接在mini-cheetah的urdf模型上修改得到。

之所以这么作的原因，时因为ironhorse的urdf模型将用于运动学计算，其构型原本与mini-cheetah一样，只是杆件参数不同。另外，mini-cheetah已经验证可用，因此在其基础上修改参数可降低编码难度。



## 拷贝mini-cheetah的urdf模型中使用简单连杆的版本

重命名为ironhorse.urdf



## 模型参数修改

ironhorse和mini-cheetah的整体构型一致，因此只需要修改关节位置和杆件长度。

### 关节位置

mini-cheetah的髋部侧向关节在质心平面上，但ironhorse的髋部侧向关节在质心平面之下。

以前向左腿为例：

```
<axis xyz="1 0 0"/>
<origin rpy="0 0 0" xyz="0.315 0.18 -0.1"/>
```

mini-cheetah的髋部前向关节在侧向关节的侧方，但ironhorse的髋部前向关节在侧向关节的下方。

同样以前向左腿为例：

```
<axis xyz="0 -1 0"/>
<origin rpy="0.0 0.0 0.0" xyz="0.0 0.00 -0.06"/>
```

二者膝关节构型一致。

同样以前向左腿为例：

```
<axis xyz="0 -1 0"/>
<origin rpy="0.0 0 0.0" xyz="0.0 0.0 -0.4"/>
```

还有一个重要的关节，就是足端点与小腿的固连关节。这对于运动学计算非常关键。

将其放在小腿的末端

```
<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.372"/>
```



### 连杆长度

连杆长度在运动学计算时并不重要，仅作显示之用。

以前向左腿为例，大腿和小腿对应改为：

```
<visual>
    <geometry>
        <!--mesh filename="meshes/mini_upper_link.obj"/-->
        <cylinder length ="0.4" radius = "0.02"/>
    </geometry>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.2"/>
</visual>
```

和

```
<visual>
    <geometry>
        <!--mesh filename="meshes/mini_lower_link.obj"/-->
        <cylinder length ="0.372" radius = "0.02"/>
    </geometry>
    <origin rpy="0.0 3.141592 0.0" xyz="0.0 0.0 -0.18"/>
</visual>
```





























