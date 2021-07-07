# kinematics based on pinocchio



## 软件安装



首先安装用于程序验证的python包

```
pip install --user pinocchio
```



然后基于robotpkg安装pinocchio

（1）安装robotpkg

```
sudo apt install -qqy lsb-release gnupg2 curl

 echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
 
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
  
   sudo apt-get update
```

（2）安装pinocchio

```
sudo apt install -qqy robotpkg-py36-pinocchio
```

注意对应的python版本



## 基于pinocchio读取模型和运动学计算



### python代码



在模型测试程序中，加入下面的代码

















