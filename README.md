# MUSE - MUltibody System dynamics Engine   
For Multibody Dynamics Simulation  
Copyright (c) 2023 Zhang He. All rights reserved.  

## MUSE 简介  
MUSE是一个轻量化的多刚体动力学求解器，采用纯C++语言编写，不依赖外部数学库、MPI等编译环境，易于在Unix、Linux、Windows等各类操作系统上编译运行。MUSE采用了面向对象的编程方式，将刚体、各类约束、多刚体系统等物理概念以类的形式进行了封装。用户使用时不必了解多刚体动力学方程的具体表达式，而只需根据实际的机械系统创建相应刚体对象与约束对象，设置各对象初始状态，并指定对象间的连接关系，MUSE将自动生成多刚体动力学方程并完成方程的数值求解，给出指定时间区间内的多体系统的运动轨迹。MUSE软件中提供了球铰、铰链、滑轨、滑轨铰、平面副、万向节、固支、大地固连等8种约束类，基本满足了目前各类机械约束形式的需要，用户也可根据实际需求添加自己的约束类。MUSE提供了专门的脚本语言用于构建多刚体系统，脚本支持变量定义，逻辑判断，语句跳转等功能，用于构建复杂的仿真模型。此外可通过直接通过修改main()程序代码构建多刚体系统与仿真模型。

## MUSE 用途  
* 多刚体系统模拟  
* 与CFD++等非定常CFD求解器耦合计算。 
* 读取飞行器气动数表，完成耦合控制系统的蒙特卡洛打靶仿真。

## 文件结构 
MUSE 包含以下文件与文件夹:

>README--------本文件  
>LICENSE-------GNU General Public License (GPL)  
>MUSE.sln------visual studio 工程  
>examples------计算示例  
>src-------------源代码及makefile  
>>Eigen------Eigen数学库  
>>MAKE-------makefile组件  
>>STUBS------串行用mpi替代库

## 关于作者 
MUSE由张贺开发，若有疑问请联系zhanghecalt@163.com。
