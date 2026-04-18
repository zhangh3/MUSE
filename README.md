# MUSE - MUltibody System dynamics Engine   
For Multibody Dynamics Simulation  
Copyright (c) 2023 Zhang He. All rights reserved.  

## MUSE 简介  
* MUSE是一个轻量化的多刚体动力学求解器。
* 采用纯C++语言编写，易于在Unix、Linux、Windows等各类操作系统上编译运行。  
* 采用了面向对象的编程方式，将刚体、各类约束、多刚体系统等物理概念以类的形式进行了封装。用户使用时不必了解多刚体动力学方程的具体表达式，而只需根据实际的机械系统创建相应刚体对象与约束对象，设置各对象初始状态，并指定对象间的连接关系，MUSE将自动生成多刚体动力学方程并完成方程的数值求解，给出指定时间区间内的多体系统的运动轨迹。  
* 提供了球铰、铰链、滑轨、固支、大地固连等约束类，基本满足了目前各类机械约束形式的需要，用户也可根据实际需求添加自己的约束类。
* 提供了专门的脚本语言用于构建多刚体系统，脚本支持变量定义，逻辑判断，语句跳转等功能，用于构建复杂的仿真模型。此外用户也可通过直接修改main()函数构建多刚体系统与仿真模型。

## MUSE 用途  
* 机器人、机械臂、变体飞行器等多刚体机械系统模拟  
* 与CFD++等非定常CFD求解器耦合计算  
* 变体飞行器控制系统蒙特卡洛打靶仿真  

---

## 文件结构 
MUSE 包含以下文件与文件夹:

```
.
│ LICENSE           GNU General Public License (GPL)  
│ MUSE.sln          Visual Studio 工程文件  
│ README.md         本文件       
├─example           计算示例
│  ├─script         脚本方式运行示例
│  │   in.script    示例脚本文件
│  └─main           修改main函数运行示例
│      main.cpp     示例main函数
└─src               源文件
    │ main.cpp      程序入口
    │ muse.h/cpp    主控类
    │ body.h/cpp    刚体类
    │ joint.h/cpp   约束基类
    │ joint_*.cpp   各类约束实现
    │ joint_enums.h 约束类型枚举
    │ MUSEsystem.h/cpp  多体系统与求解器核心
    │ create.h/cpp  创建命令
    │ change.h/cpp  修改命令
    │ run.h/cpp     运行命令
    │ input.h/cpp   脚本解析器
    │ output.h/cpp  输出管理
    │ stats.h/cpp   统计输出
    │ result.h/cpp  结果输出
    │ compute.h/cpp 计算量基类
    │ compute_body.h/cpp  刚体状态计算
    │ variable.h/cpp      变量系统
    │ modify.h/cpp  计算管理
    │ ensemble.h/cpp      MPI集成管理
    │ memory.h/cpp  内存管理
    │ error.h/cpp   错误处理
    │ timer.h/cpp   计时器
    │ math_extra.h  数学工具函数
    │ pointers.h    指针传递基类
    │ MUSEunistd.h  跨平台兼容
    │ version.h     版本号
    │ style_command.h   命令注册宏
    │ style_compute.h   计算注册宏
    │ style_result.h    结果注册宏
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

---

## MUSE 编译方式

### WINDOWS   
* 使用Visual Studio打开 `MUSE.sln`，执行编译   
* 使用Cygwin64：
```bash
cd src
make serial
```

### LINUX
* 串行版：
```bash
cd src
make serial
```
* 并行版（尚不完善）：
```bash
cd src
make mpi
```

---

## MUSE 运行方式

### 基于脚本运行   
```bash
MUSE.exe < in.script          # 重定向方式
MUSE.exe -i in.script         # 指定输入文件
MUSE.exe -in in.script        # 同上
```
也可运行 `MUSE.exe`，在命令行窗口中逐行输入脚本指令。
   
### 修改main函数运行   
* 替换 `src/main.cpp`，编译后运行。示例见 `example/main/main.cpp`。

---

## 使用方法

使用MUSE开展多体动力学仿真主要分为四个步骤：

1. **创建刚体** — 指定每个刚体的质量、转动惯量、速度、角速度、质心坐标与姿态；
2. **创建约束** — 指定约束类型、连接刚体与连接位置；
3. **构建多体系统** — 将刚体与约束组装为一个多体系统，MUSE自动生成多体动力学微分方程；
4. **求解** — 指定时间步长与求解步数，开展多体动力学求解。

---

### 1. 创建刚体

创建一个刚体对象，并指定其质量、转动惯量、质心速度、角速度、质心坐标与姿态。

**坐标约定：**
* 质心坐标（pos）、质心速度（vel）：在系统惯性系下的坐标
* 角速度（omega）：在刚体随体坐标系下的坐标
* 姿态四元数（quat）：`[qx, qy, qz, qw]` 格式，**第四个数为实数项（标量部分）**

**Body类主要属性：**

```C++
char *name;                        //刚体名
Eigen::Vector3d pos;               //惯性系下质心位置，默认值为(0,0,0)
Eigen::Vector3d vel;               //惯性系下质心速度，默认值为(0,0,0)
Eigen::Vector4d quat;              //质心姿态四元数[qx,qy,qz,qw]，默认值为(0,0,0,1)
Eigen::Vector3d omega;             //体坐标系下角速度，默认值为(0,0,0)
double mass;                       //刚体质量，默认值为1
Eigen::Matrix3d inertia;           //刚体转动惯量（体坐标系），默认值为单位矩阵
```

| 属性      | 默认值   | 脚本参数数量 | 说明 |
| -------- | ---------- | --- | ---- |
| `pos`      | 0 0 0   | 3 | 惯性系下质心位置 x y z |
| `vel`   | 0 0 0  | 3 | 惯性系下质心速度 vx vy vz |
| `quat`   | 0 0 0 1  | 4 | 姿态四元数 qx qy qz qw（最后一位为实部） |
| `omega` | 0 0 0   | 3 | 体坐标系下角速度 wx wy wz |
| `mass`   | 1   | 1 | 质量 |
| `inertia`     | 1 1 1 0 0 0  | 6 | Ixx Iyy Izz Ixy Ixz Iyz |

命令中不需要指定所有属性的值，未指定的属性将取默认值。

#### 脚本方式创建刚体

```
create body 刚体名  属性1  属性值1  属性2  属性值2 ...
```

示例：
```bash
# 创建刚体b1，质心位置(0,0,0)，姿态四元数(0,0,0,1)，质量2
create body b1  pos 0 0 0 quat 0 0 0 1 mass 2

# 创建刚体b2，指定位置、速度和转动惯量
create body b2  pos 1 0 0 vel 0 1 0 inertia 2 3 4 0 0 0

# 创建刚体b3，指定角速度
create body b3  pos 2 0 0 omega 0 0 1
```

#### 程序方式创建刚体
```C++
char name[] = "body1";
int id = muse->add_Body(name);                  // 创建刚体，返回索引
muse->body[id]->pos  << 0, 0, 0;                // 设置质心坐标（惯性系）
muse->body[id]->vel  << 0, 0, 0;                // 设置速度（惯性系）
muse->body[id]->quat << 0, 0, 0, 1;             // 设置姿态四元数
muse->body[id]->set_Omega(0, 0, 1);             // 设置角速度（体坐标系）
muse->body[id]->set_Mass(2.0);                  // 设置质量
muse->body[id]->set_Inertia(1, 2, 3, 0, 0, 0); // 设置转动惯量
```

---

### 2. 创建约束

约束用于限制两个刚体间的相对运动。MUSE目前支持以下约束类型：

| 类型 | 关键字 | 约束自由度 | 说明 |
|------|--------|-----------|------|
| 球铰 | `sphere` | 3 | 约束两刚体上连接点位置重合，允许任意旋转 |
| 铰链 | `hinge`  | 5 | 约束位置重合且只允许绕一个轴旋转 |
| 滑轨 | `slide`  | 5 | 约束旋转相同，只允许沿一个轴平动 |
| 固支 | `fix`    | 6 | 完全约束两刚体间的相对运动 |
| 大地固连 | `ground` | 7 | 将刚体完全固定在空间中（只需指定body1） |

#### 约束参数

| 参数 | 说明 |
|------|------|
| `body1` | 连接的第一个刚体名 |
| `body2` | 连接的第二个刚体名（ground类型不需要） |
| `point1` | 约束点在body1体坐标系中的位置（默认 0 0 0） |
| `point2` | 约束点在body2体坐标系中的位置（默认 0 0 0） |
| `axis1` | 约束轴在body1体坐标系中的方向（铰链/滑轨需要） |
| `axis2` | 约束轴在body2体坐标系中的方向 |

#### 脚本方式创建约束

```
create joint 约束名 约束类型 body1 刚体名1 body2 刚体名2 point1 x y z point2 x y z
```

示例：
```bash
# 球铰：b1和b2通过球铰连接
create joint j1 sphere body1 b1 body2 b2 point1 0.5 0 0 point2 -0.5 0 0

# 铰链：b2和b3通过铰链连接，铰链轴为body1的z轴方向
create joint j2 hinge body1 b2 body2 b3 point1 0.5 0 0 point2 -0.5 0 0 axis1 0 0 1

# 滑轨：允许沿body1的x轴方向滑动
create joint j3 slide body1 b1 body2 b2 point1 0 0 0 point2 0 0 0 axis1 1 0 0

# 固支：b1和b2完全固连
create joint j4 fix body1 b1 body2 b2 point1 0.5 0 0 point2 -0.5 0 0

# 大地固连：将b1固定在空间中
create joint grd ground body1 b1
```

#### 程序方式创建约束
```C++
#include "joint_enums.h"  // 需要包含约束类型枚举

char jname[] = "joint1";
int jid = muse->add_Joint(jname);
muse->joint[jid]->body[0] = muse->body[0];      // 设置body1
muse->joint[jid]->body[1] = muse->body[1];      // 设置body2
muse->joint[jid]->set_type(SPHERE);              // 设置类型
muse->joint[jid]->point1 << 0.5, 0, 0;          // body1体坐标系下的连接点
muse->joint[jid]->point2 << -0.5, 0, 0;         // body2体坐标系下的连接点
// 铰链/滑轨还需设置轴方向：
// muse->joint[jid]->set_axis(0, 0, 1, 1);      // axis1方向
```

---

### 3. 构建多体系统

将创建好的刚体与约束添加到多体系统中，设置时间步长和重力加速度。

#### 脚本方式

```bash
# 批量添加刚体和约束
system addbodys b1 b2 b3 /addbodys addjoints j1 j2 grd /addjoints

# 设置时间步长（秒）
system dt 1E-3

# 设置重力加速度（默认 0 -9.8 0）
system gravity 0 -9.8 0
```

单个添加/删除：
```bash
system addbody b1               # 添加单个刚体
system addjoint j1              # 添加单个约束
system removebody b1            # 删除单个刚体
system removejoint j1           # 删除单个约束
system removebodys b1 b2 /removebodys    # 批量删除刚体
system removejoints j1 j2 /removejoints  # 批量删除约束
```

#### 程序方式
```C++
muse->system->add_Body(muse->body[0]);
muse->system->add_Body(muse->body[1]);
muse->system->add_Joint(muse->joint[0]);
muse->system->add_Joint(muse->joint[1]);
muse->system->setup();          // 初始化系统矩阵（必须在添加完所有刚体约束后调用）
muse->system->dt = 1E-3;        // 设置时间步长
muse->system->ga << 0, -9.8, 0; // 设置重力加速度
```

---

### 4. 求解

#### 脚本方式

```bash
run 1000                 # 运行1000个时间步
run 5000 upto            # 运行到第5000步
run 100 start 100        # 从第100步开始运行100步
run 100 every 10 "print 'step $s'"  # 每10步执行一次命令
```

#### 程序方式
```C++
muse->system->solve(1000);  // 求解1000个时间步
```

---

### 5. 修改参数

在仿真过程中可以修改刚体和约束的参数：

```bash
# 修改刚体属性
change body b1 mass 2                    # 修改质量
change body b1 pos 1 0 0                 # 修改位置
change body b1 vel 0 1 0                 # 修改速度
change body b1 quat 0 0 0 1             # 修改姿态
change body b1 omega 0 0 1              # 修改角速度
change body b1 inertia 2 3 4 0 0 0      # 修改转动惯量

# 修改约束属性
change joint j1 body1 b1                # 修改连接刚体
change joint j1 point1 0.5 0 0          # 修改连接点
change joint j1 axis1 0 0 1             # 修改约束轴
```

---

### 6. 输出与统计

#### stats — 屏幕输出控制

```bash
stats 10                                 # 每10步输出一次
stats_style step cpu time c_comb1[*]     # 设置输出内容
```

`stats_style` 支持的关键字：

| 关键字 | 说明 |
|--------|------|
| `step` | 当前时间步数 |
| `cpu`  | CPU耗时（秒） |
| `dt`   | 时间步长 |
| `time` | 当前物理时间 |
| `c_XXX` | compute变量XXX的标量值 |
| `c_XXX[N]` | compute变量XXX的第N个分量 |
| `c_XXX[*]` | compute变量XXX的所有分量 |
| `v_XXX` | variable变量XXX的值 |

#### compute — 计算量定义

```bash
# 提取b1的位置和速度
compute comb1 body b1 pos vel

# 提取b2的角速度
compute comb2 body b2 omega

# 提取b3的姿态四元数
compute comb3 body b3 quat
```

`compute body` 支持的量：`pos`(3分量)、`vel`(3分量)、`quat`(4分量)、`omega`(3分量)

---

### 7. 脚本语言功能

MUSE脚本支持以下高级功能：

#### 变量定义
```bash
variable a equal 3.14               # 数值变量
variable b string hello              # 字符串变量
variable i loop 10                   # 循环变量 (1到10)
variable j index 1 2 3 5 8           # 索引变量
variable e getenv HOME               # 环境变量
```

#### 条件判断
```bash
if "$a > 2" then "print 'a is large'"
if "$a > 2" then "jump in.script label1" else "print 'a is small'"
```

#### 循环
```bash
variable i loop 5
label loop_start
  print "i = $i"
  run 100
  next i
jump in.script loop_start
label loop_end
```

#### 输出
```bash
print "Hello World"
print "Position = $x"           # 变量替换
echo screen                     # 回显脚本命令到屏幕
echo log                        # 回显到日志文件
echo both                       # 同时回显
```

#### 注释
```bash
# 这是注释
create body b1 pos 0 0 0  # 行尾注释也可以
```

---

## 完整脚本示例

```bash
# ============================================
# 三体球铰摆系统仿真
# ============================================

# --- 创建刚体 ---
create body b1  pos 0 0 0 quat 0 0 0 1 mass 1
create body b2  pos 1 0 0 quat 0 0 0 1 mass 1
create body b3  pos 2 0 0 quat 0 0 0 1 mass 1

# --- 创建约束 ---
# 球铰连接b1和b2
create joint j1 sphere body1 b1 body2 b2 point1 0.5 0 0 point2 -0.5 0 0
# 球铰连接b2和b3
create joint j2 sphere body1 b2 body2 b3 point1 0.5 0 0 point2 -0.5 0 0
# 将b1固连大地
create joint grd ground body1 b1

# --- 构建多体系统 ---
system addbodys b1 b2 b3 /addbodys addjoints j1 j2 grd /addjoints
system dt 1E-3
system gravity 0 -9.8 0

# --- 定义输出 ---
compute comb1 body b1 pos vel
compute comb2 body b2 pos omega
stats 10
stats_style step cpu time c_comb1[*] c_comb2[*]

# --- 求解 ---
run 1000

# --- 修改参数后继续求解 ---
change body b1 mass 2
run 500

print "Simulation complete"
```

---

## 技术服务 
若有疑问或建议请联系 zhanghecalt@163.com。

## 许可证
本项目采用 GNU General Public License (GPL) 开源许可证。详见 LICENSE 文件。
