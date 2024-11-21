# MUSE - MUltibody System dynamics Engine   
For Multibody Dynamics Simulation  
Copyright (c) 2023 Zhang He. All rights reserved.  

## MUSE 简介  
* MUSE是一个轻量化的多刚体动力学求解器。
* 采用纯C++语言编写，易于在Unix、Linux、Windows等各类操作系统上编译运行。  
* 采用了面向对象的编程方式，将刚体、各类约束、多刚体系统等物理概念以类的形式进行了封装。用户使用时不必了解多刚体动力学方程的具体表达式，而只需根据实际的机械系统创建相应刚体对象与约束对象，设置各对象初始状态，并指定对象间的连接关系，MUSE将自动生成多刚体动力学方程并完成方程的数值求解，给出指定时间区间内的多体系统的运动轨迹。  
* 提供了球铰、铰链、滑轨、滑轨铰、平面副、万向节、固支、大地固连等约束类，基本满足了目前各类机械约束形式的需要，用户也可根据实际需求添加自己的约束类。
* 提供了专门的脚本语言用于构建多刚体系统，脚本支持变量定义，逻辑判断，语句跳转等功能，用于构建复杂的仿真模型。此外用户也可通过直接修改main()函数构建多刚体系统与仿真模型。

## MUSE 用途  
* 机器人、机械臂、变体飞行器等多刚体机械系统模拟  
* 与CFD++等非定常CFD求解器耦合计算  
* 变体飞行器控制系统蒙特卡洛打靶仿真  

## 文件结构 
MUSE 包含以下文件与文件夹:

```
.
│ LICENSE           GNU General Public License (GPL)  
│ MUSE.sln          visual studio 工程文件  
│ README.md         本文件       
├─example           计算示例
│  ├─script
│  └─main    
└─src               源文件
    │ *.cpp
    │ *.h
    │ Makefile      makefile
    ├─Eigen         Eigen数学库   
    ├─MAKE          makefile组件
    │   Makefile.mpi
    │   Makefile.serial 
    └─STUBS         串行编译用dummy MPI库
        Makefile
        mpi.c
        mpi.h
```

## MUSE 编译方式
### WINDOWS   
* 使用Visual Studio打开MUSE.sln，执行编译   
* 使用Cygwin64：
```
cd src
make serial
```
### LINUX
* 串行版：
```
cd src
make serial
```
* 并行版(尚不完善)：
```
cd src
make mpi
```

## MUSE 运行方式
### 基于脚本运行   
* MUSE.exe < in.script   
* MUSE.exe -i in.script   
* 运行MUSE.exe，在cmd窗口中输入脚本指令运行
   
### 修改main函数运行   
* 替换src/文件夹中main.cpp，编译后运行

---
## 使用方法
使用MUSE开展多体动力学仿真主要分为四个主要步骤：
1. 分别创建多体系统内的所有刚体，指定每个刚体的质量、转动惯量、速度、角速度、质心坐标与姿态；
2. 创建连接刚体的约束，指定约束类型与连接位置；
3. 将创建好刚体与约束组成一个多体系统，MUSE会将它们联立成一个多体动力学微分方程；
4. 指定时间步长与求解步数，开展多体动力学求解。

### 创建刚体
创建一个刚体对象，并指定其质量、转动惯量、质心速度、角速度、质心坐标与姿态。其中刚体姿态用四元数的方式表示，程序中姿态四元数的第四个数为实数项。

质心坐标，质心速度是在系统惯性系下的坐标，角速度是在刚体随体坐标系下的坐标。

MUSE中刚体类在nody.h中定义，类的名称为Body，Body类中最重要的几个为
```C++
char *name;                        //刚体名
Eigen::Vector3d pos;               //惯性系下质心位置，默认值为0，0，0
Eigen::Vector3d vel;               //惯性系下质心速度，默认值为0，0，0
Eigen::Vector4d quat;              //质心姿态四元数，默认值为0，0，0，1
Eigen::Vector3d omega;             //体坐标系下角速度，默认值为0，0，0
double mass;                       //刚体质量，默认值为1。
Eigen::Matrix3d inertia;           //刚体转动惯量，默认值为单位矩阵。
```

* 通过直接修改main.cpp运行
```C++
//下面代码为创建一个刚体，命名为"body1"

//定义字刚体名符串
char name[] = "body1";
//创建名为"body1"的刚体对象
muse->add_Body(name1);
//创建名为"body1"的刚体对象，刚体新创建的刚体会储存在muse->body数组中，按创建顺序排列。
muse->add_Body(name1);
muse->body[0]->mass = 1;               //设置刚体质量
muse->body[0]->pos   << 0,0,0;         //设置刚体质心坐标（惯性系）
muse->body[0]->vel   << 0,0,0;         //设置刚体速度（惯性系）
muse->body[0]->quat  << 0, 0, 0, 1;     //设置刚体姿态   
muse->body[0]->set_Omega(0,0,1)         //设置刚体角速度（体坐标系）
muse->body[0]->set_Inertia(1,2,3,0,0,0) //设置刚体转动惯量Ixx，Iyy，Izz，Ixy，Ixz，Iyz（体坐标系）
```

* 通过脚本运行
创建刚体脚本命令主要形式为

<small>*create body 刚体名  属性1  属性值1  属性2  属性值2*</small>

示例
```
create body b1  pos 0 0 0 quat 0 0 0 1 #创建一个刚体，命名b1, 质心位置0 0 0，姿态四元数0 0 0 1
```
主要属性及初始值为

| 属性      | 初始属性值   | 备注 |
| -------- | ---------- | ---- |
| pos      | 0 0 0   |   惯性系    |
| quat   | 0 0 0 1  |最后一位数为四元数实部 |
| vel   | 0 0 0  |     惯性系  |
|omega | 0 0 0   | 体坐标系 |
| mass   | 1   |       |
| inertia     | 1 1 1 0 0 0   |分别为Ixx，Iyy，Izz，Ixy，Ixz，Iyz|

命令中不需要指定所有属性的值，未指定的属性将取默认值。

### 创建约束
文档待完善

#### 固连约束
文档待完善

### 构建多体系统
文档待完善

### 求解
文档待完善

## 技术服务 
若有疑问或建议请联系zhanghecalt@163.com。

## 备注 
软件尚未完成，目前为开发版本，后续会陆续完善功能，撰写说明文档，请持续关注。
