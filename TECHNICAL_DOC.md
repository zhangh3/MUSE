# MUSE 技术文档

## 目录

1. [软件架构](#1-软件架构)
2. [核心数学基础](#2-核心数学基础)
3. [约束动力学求解](#3-约束动力学求解)
4. [约束类型详解](#4-约束类型详解)
5. [时间积分方法](#5-时间积分方法)
6. [脚本系统](#6-脚本系统)
7. [计算与输出系统](#7-计算与输出系统)
8. [内存管理](#8-内存管理)
9. [已知限制与注意事项](#9-已知限制与注意事项)
10. [Bug修复记录](#10-bug修复记录)

---

## 1. 软件架构

### 1.1 总体设计

MUSE采用类似LAMMPS的面向对象架构。核心设计模式为：

- **主控类 MUSE**：持有所有子系统指针，充当全局上下文
- **Pointers基类**：所有子系统类通过继承Pointers获取对其他子系统的引用
- **命令注册机制**：通过宏在编译时将命令类注册到解析器

### 1.2 类层次结构

```
MUSE (主控类, muse.h)
├── Memory        (memory.h)       内存管理：数组分配/释放工具
├── Error         (error.h)        错误处理：警告/致命错误/日志
├── Ensemble      (ensemble.h)     MPI管理 & 随机数生成器
│   ├── RanMars                    Marsaglia随机数
│   └── RanPark                    Park-Miller随机数
├── Input         (input.h)        脚本解析器：读取/解析/执行命令
│   └── Variable  (variable.h)     变量系统：equal/string/loop/index/getenv
├── Modify        (modify.h)       计算管理器
│   └── Compute[] (compute.h)      计算量数组
│       └── ComputeBody            刚体状态提取 (pos/vel/quat/omega)
├── Output        (output.h)       输出管理
│   ├── Stats     (stats.h)        屏幕统计输出
│   └── Result[]  (result.h)       文件结果输出
├── Timer         (timer.h)        计时器（wall/CPU time）
├── MUSEsystem    (MUSEsystem.h)   多体系统核心
│   ├── Body**                     系统内刚体指针数组
│   └── Joint**                    系统内约束指针数组
├── Body**        (body.h)         全局刚体对象数组
└── Joint**       (joint.h)        全局约束对象数组
```

### 1.3 Pointers基类

```c++
class Pointers {
protected:
    MUSE *muse;
    Memory *&memory;
    Error *&error;
    Ensemble *&ensemble;
    Input *&input;
    Modify *&modify;
    Output *&output;
    Timer *&timer;
    MUSEsystem *&system;
};
```

所有子系统类（Create, Change, Run, Input, MUSEsystem等）都继承此基类，从而可以通过引用访问任何其他子系统，避免全局变量。

### 1.4 命令注册机制

通过style头文件中的宏在编译时注册命令：

**style_command.h**
```c++
CommandStyle(create, Create)
CommandStyle(change, Change)
CommandStyle(run, Run)
CommandStyle(system, MUSEsystem)
// ...
```

Input解析器在处理命令时，通过字符串匹配查找对应的命令类并调用其`command()`方法。

### 1.5 主要执行流程

```
main() → MUSE构造函数 → Input::file() 读取脚本
                                ↓
                        逐行解析命令
                        ├── "create body ..." → Create::command()
                        ├── "create joint ..." → Create::command()
                        ├── "system ..." → MUSEsystem::command()
                        │       ├── addbody/addjoint → 组装系统
                        │       ├── setup() → 初始化矩阵
                        │       └── dt/gravity → 设置参数
                        ├── "run N" → Run::command()
                        │       └── MUSEsystem::solve(N)
                        │           └── RK4循环
                        │               ├── body2x() 刚体状态→状态向量
                        │               ├── makeBigM() 构造质量矩阵
                        │               ├── makeBigA() 构造约束矩阵
                        │               ├── makeBigF() 构造力向量
                        │               ├── SVD分解+求解
                        │               └── x2body() 状态向量→刚体状态
                        └── "print ..." → 输出
```

---

## 2. 核心数学基础

### 2.1 状态表示

每个刚体用7维广义坐标描述：

$$\mathbf{x}_i = \begin{bmatrix} \mathbf{r}_i \\ \mathbf{q}_i \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ q_x \\ q_y \\ q_z \\ q_w \end{bmatrix} \in \mathbb{R}^7$$

- $\mathbf{r}_i \in \mathbb{R}^3$：惯性系下质心位置
- $\mathbf{q}_i \in \mathbb{R}^4$：姿态四元数（**Hamilton约定，标量qw排在最后**）

系统总状态向量：
$$\mathbf{x} = [\mathbf{x}_1^T, \mathbf{x}_2^T, \ldots, \mathbf{x}_n^T]^T \in \mathbb{R}^{7n}$$

速度向量（也是7n维）：
$$\dot{\mathbf{x}} = [\dot{\mathbf{r}}_1^T, \dot{\mathbf{q}}_1^T, \ldots, \dot{\mathbf{r}}_n^T, \dot{\mathbf{q}}_n^T]^T$$

### 2.2 四元数运动学

**四元数格式：** $\mathbf{q} = [q_x, q_y, q_z, q_w]^T$，其中 $q_w$ 为标量部分。

**方向余弦矩阵（DCM）：** 从体坐标系到惯性系的旋转矩阵

$$DCM = (q_w^2 - \|\mathbf{q}_{vec}\|^2)I_{3\times3} + 2\mathbf{q}_{vec}\mathbf{q}_{vec}^T + 2q_w[\mathbf{q}_{vec}]_\times$$

其中 $\mathbf{q}_{vec} = [q_x, q_y, q_z]^T$。

**T矩阵**（3×4，体坐标系角速度到四元数导数的映射）：

$$\boldsymbol{\omega}_{body} = T \dot{\mathbf{q}}$$

$$T = 2\begin{bmatrix} q_w & -q_z & q_y & q_x \\ q_z & q_w & -q_x & q_y \\ -q_y & q_x & q_w & q_z \end{bmatrix}$$

（注意：这里使用了 $T = 2[q_w I - [\mathbf{q}_{vec}]_\times \;|\; -2\mathbf{q}_{vec}]$ 的紧凑表示，
实际实现见`body.cpp`中的`refresh()`函数。）

**逆映射：**
$$\dot{\mathbf{q}} = \frac{1}{4}T^T\boldsymbol{\omega}_{body}$$

**Td矩阵**（T的时间导数，用于计算约束右端项）：

$$\dot{T} = 2\begin{bmatrix} \dot{q}_w & -\dot{q}_z & \dot{q}_y & \dot{q}_x \\ \dot{q}_z & \dot{q}_w & -\dot{q}_x & \dot{q}_y \\ -\dot{q}_y & \dot{q}_x & \dot{q}_w & \dot{q}_z \end{bmatrix}$$

### 2.3 叉积矩阵

向量 $\mathbf{a} = [a_1, a_2, a_3]^T$ 的叉积矩阵（skew-symmetric matrix）：

$$[\mathbf{a}]_\times = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}$$

满足 $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a} \times \mathbf{b}$

实现位于 `math_extra.h` 的 `crs()` 函数。

### 2.4 惯性系中的旋转量

将体坐标系下的量转换到惯性系：

$$T_{I} = DCM \cdot T$$

$$\boldsymbol{\omega}_{I} = DCM \cdot \boldsymbol{\omega}_{body}$$

$$J_{I} = DCM \cdot J_{body} \cdot DCM^T$$

---

## 3. 约束动力学求解

### 3.1 运动方程

无约束的运动方程：
$$M\ddot{\mathbf{x}} = \mathbf{F}$$

约束方程（加速度级别）：
$$A\ddot{\mathbf{x}} = \mathbf{b}$$

### 3.2 质量矩阵

系统的广义质量矩阵 $M$（7n×7n 分块对角矩阵）：

$$M = \text{diag}(M_1, M_2, \ldots, M_n)$$

每个刚体的子矩阵（7×7）：

$$M_i = \begin{bmatrix} m_i I_{3\times3} & 0_{3\times4} \\ 0_{4\times3} & T_i^T J_i T_i \end{bmatrix}$$

- $m_i$：刚体质量
- $J_i$：体坐标系下惯性张量
- $T_i$：四元数运动学矩阵

实现见 `MUSEsystem.cpp` 的 `makeBigM()` 函数。

### 3.3 广义力向量

$$\mathbf{F}_i = \begin{bmatrix} m_i \mathbf{g} \\ -T_i^T(\boldsymbol{\omega}_i \times J_i \boldsymbol{\omega}_i) \end{bmatrix} \in \mathbb{R}^7$$

- $m_i \mathbf{g}$：重力（惯性系）
- $\boldsymbol{\omega}_i \times J_i \boldsymbol{\omega}_i$：陀螺力矩（Coriolis/gyroscopic torque），在快速旋转体中至关重要

实现见 `MUSEsystem.cpp` 的 `makeBigF()` 函数。

### 3.4 约束矩阵组装

总约束矩阵由各个约束和四元数归一化约束组装而成：

$$A = \begin{bmatrix} A_{joint,1} \\ A_{joint,2} \\ \vdots \\ A_{quat,1} \\ A_{quat,2} \\ \vdots \end{bmatrix}, \quad \mathbf{b} = \begin{bmatrix} \mathbf{b}_{joint,1} \\ \mathbf{b}_{joint,2} \\ \vdots \\ b_{quat,1} \\ b_{quat,2} \\ \vdots \end{bmatrix}$$

**四元数归一化约束：**
$$\mathbf{q}^T\mathbf{q} = 1 \quad \Rightarrow \quad 2\mathbf{q}^T\ddot{\mathbf{q}} = -2\dot{\mathbf{q}}^T\dot{\mathbf{q}}$$

每个刚体贡献1行约束：
$$A_{quat,i} = [0_{1\times3} \;|\; 2\mathbf{q}_i^T], \quad b_{quat,i} = -2\dot{\mathbf{q}}_i^T\dot{\mathbf{q}}_i$$

实现见 `MUSEsystem.cpp` 的 `makeBigA()` 函数。

### 3.5 求解方法

采用基于SVD伪逆的约束投影方法（Udwadia-Kalaba）：

**步骤1：** SVD分解约束矩阵
$$A = U\Sigma V^T$$

**步骤2：** 计算伪逆
$$A^+ = V\Sigma^+U^T$$

其中 $\Sigma^+$ 通过将奇异值取倒数（阈值截断小奇异值）得到。

**步骤3：** 构造零空间投影矩阵
$$P = I - A^+A$$

**步骤4：** 组装扩展系统

$$\bar{M} = \begin{bmatrix} P \cdot M \\ A \end{bmatrix} \in \mathbb{R}^{(7n+m)\times 7n}$$

$$\bar{\mathbf{F}} = \begin{bmatrix} \mathbf{F} \\ \mathbf{b} \end{bmatrix} \in \mathbb{R}^{7n+m}$$

**步骤5：** 最小二乘求解
$$\ddot{\mathbf{x}} = \bar{M}^{-1}\bar{\mathbf{F}}$$

使用Eigen的SVD求解器：`Mbar.bdcSvd(ComputeThinU | ComputeThinV).solve(Fall)`

实现见 `MUSEsystem.cpp` 的 `solve()` 函数。

---

## 4. 约束类型详解

### 4.1 约束基类 (Joint)

所有约束类继承自Joint基类，需实现以下虚函数：

```c++
virtual void makeA(int flag);   // 构造约束矩阵 A1, A2, b
```

Joint基类主要成员：
```c++
int nbody;          // 连接刚体数量 (ground=1, 其余=2)
int nconstraint;    // 约束方程数
Body *body[2];      // 连接的两个刚体
Vector3d point1;    // body1体坐标系下的连接点
Vector3d point2;    // body2体坐标系下的连接点
Vector3d axis1;     // body1体坐标系下的约束轴
Vector3d axis2;     // body2体坐标系下的约束轴
MatrixXd A1, A2;    // 约束矩阵子块
VectorXd b;         // 约束方程右端项
```

### 4.2 球铰约束 (Sphere Joint)

**物理含义：** 两刚体上的连接点在惯性系中保持位置重合，允许任意相对旋转。

**自由度：** 3个平动约束，保留3个旋转自由度。

**位置级约束：**
$$\mathbf{r}_1 + DCM_1 \cdot \mathbf{s}_1 = \mathbf{r}_2 + DCM_2 \cdot \mathbf{s}_2$$

**加速度级约束方程推导：**

对位置约束二阶求导：
$$\ddot{\mathbf{r}}_1 + \frac{d^2}{dt^2}(DCM_1 \cdot \mathbf{s}_1) = \ddot{\mathbf{r}}_2 + \frac{d^2}{dt^2}(DCM_2 \cdot \mathbf{s}_2)$$

利用 $\frac{d^2}{dt^2}(DCM_i \cdot \mathbf{s}_i) = [\boldsymbol{\alpha}_{I,i}]_\times \cdot \mathbf{p}_{I,i} + [\boldsymbol{\omega}_{I,i}]_\times^2 \cdot \mathbf{p}_{I,i}$

其中 $\mathbf{p}_{I,i} = DCM_i \cdot \mathbf{s}_i$，$\boldsymbol{\alpha}_{I,i} = DCM_i \cdot T_i \cdot \ddot{\mathbf{q}}_i + DCM_i \cdot \dot{T}_i \cdot \dot{\mathbf{q}}_i$

整理得：

$$A_1 = [I_{3\times3} \;|\; -[\mathbf{p}_{1,I}]_\times \cdot T_{1,I}] \in \mathbb{R}^{3\times7}$$

$$A_2 = [-I_{3\times3} \;|\; [\mathbf{p}_{2,I}]_\times \cdot T_{2,I}] \in \mathbb{R}^{3\times7}$$

$$\mathbf{b} = -[\boldsymbol{\omega}_{1,I}]_\times^2 \cdot \mathbf{p}_{1,I} + [\boldsymbol{\omega}_{2,I}]_\times^2 \cdot \mathbf{p}_{2,I} + DCM_2 \cdot \dot{T}_2 \cdot \dot{\mathbf{q}}_2 \times \mathbf{p}_{2,I} - DCM_1 \cdot \dot{T}_1 \cdot \dot{\mathbf{q}}_1 \times \mathbf{p}_{1,I}$$

实现见 `joint_sphere.cpp`。

### 4.3 铰链约束 (Hinge Joint)

**物理含义：** 两刚体在连接点位置重合，且只能绕铰链轴相对旋转。

**自由度：** 5个约束（3平动+2旋转），保留绕铰链轴的1个旋转自由度。

**实现方法：** "两点法"——在铰链轴上取两个对称点，分别约束两点位置重合：

点1：$P_1 + axis_1$ 和 $P_2 + axis_{2,local}$
点2：$P_1 - axis_1$ 和 $P_2 - axis_{2,local}$

每对点贡献3个位置约束方程，共6个方程。由于铰链轴方向的约束是冗余的（两对点在轴方向的约束等价），实际独立约束为5个。SVD求解器自动处理这种冗余。

$$A_1 = \begin{bmatrix} I_{3\times3} & -[\mathbf{ap}_{11,I}]_\times \cdot T_{1,I} \\ I_{3\times3} & -[\mathbf{ap}_{12,I}]_\times \cdot T_{1,I} \end{bmatrix} \in \mathbb{R}^{6\times7}$$

其中 $\mathbf{ap}_{11} = DCM_1 \cdot (\mathbf{s}_1 + \mathbf{a}_1)$，$\mathbf{ap}_{12} = DCM_1 \cdot (\mathbf{s}_1 - \mathbf{a}_1)$。

实现见 `joint_hinge.cpp`。

### 4.4 滑轨约束 (Slide Joint)

**物理含义：** 两刚体旋转完全锁定，只允许沿指定轴方向相对平动。

**自由度：** 5个约束（2平动+3旋转），保留沿轴方向的1个平动自由度。

**约束方程（6个方程，5个独立）：**

位置约束（3个方程，2个独立）：通过叉积矩阵 $[\mathbf{a}_I]_\times$ 投影，消除轴方向分量

$$[\mathbf{a}_I]_\times \cdot (\ddot{\mathbf{r}}_1 - \ddot{\mathbf{r}}_2 + \ldots) = \mathbf{b}_{pos}$$

旋转约束（3个方程，与Fix相同）：

$$T_{1,I}\ddot{\mathbf{q}}_1 - T_{2,I}\ddot{\mathbf{q}}_2 = \mathbf{b}_{rot}$$

其中 $[\mathbf{a}_I]_\times$ 是3×3反对称矩阵，秩为2，因此3个位置方程中只有2个独立。

实现见 `joint_slide.cpp`。

### 4.5 固支约束 (Fix Joint)

**物理含义：** 两刚体完全固连，无任何相对运动。

**自由度：** 6个约束（3平动+3旋转），无剩余自由度。

**位置约束（3个，与球铰相同）：**
$$\mathbf{r}_1 + DCM_1 \cdot \mathbf{s}_1 = \mathbf{r}_2 + DCM_2 \cdot \mathbf{s}_2$$

$$A_1^{pos} = [I_{3\times3} \;|\; -[\mathbf{p}_{1,I}]_\times \cdot T_{1,I}]$$

**旋转约束（3个）：**
$$\boldsymbol{\omega}_{1,I} = \boldsymbol{\omega}_{2,I}$$

$$A_1^{rot} = [0_{3\times3} \;|\; T_{1,I}], \quad A_2^{rot} = [0_{3\times3} \;|\; -T_{2,I}]$$

$$\mathbf{b}_{rot} = DCM_2 \cdot \dot{T}_2 \cdot \dot{\mathbf{q}}_2 - DCM_1 \cdot \dot{T}_1 \cdot \dot{\mathbf{q}}_1$$

实现见 `joint_fix.cpp`。

### 4.6 大地固连约束 (Ground Joint)

**物理含义：** 将刚体完全固定在空间中（位置和姿态均不变）。

**自由度：** 7个约束（3平动+4四元数），无任何自由度。

**约束方程：**
$$\ddot{\mathbf{x}} = \mathbf{0}$$

$$A_1 = I_{7\times7}, \quad \mathbf{b} = \mathbf{0}$$

实现见 `joint_ground.cpp`。

---

## 5. 时间积分方法

### 5.1 RK4（4阶Runge-Kutta，默认方法）

系统ODE：$\dot{\mathbf{y}} = f(t, \mathbf{y})$，其中 $\mathbf{y} = [\mathbf{x}, \dot{\mathbf{x}}]$

$$\mathbf{k}_1 = f(t_n, \mathbf{y}_n)$$
$$\mathbf{k}_2 = f(t_n + h/2, \mathbf{y}_n + h\mathbf{k}_1/2)$$
$$\mathbf{k}_3 = f(t_n + h/2, \mathbf{y}_n + h\mathbf{k}_2/2)$$
$$\mathbf{k}_4 = f(t_n + h, \mathbf{y}_n + h\mathbf{k}_3)$$
$$\mathbf{y}_{n+1} = \mathbf{y}_n + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$$

每步需要4次约束矩阵组装和SVD求解。

实现见 `MUSEsystem.cpp` 的 `rk4()` 函数。

### 5.2 前向欧拉法

$$\mathbf{y}_{n+1} = \mathbf{y}_n + h \cdot f(t_n, \mathbf{y}_n)$$

仅用于调试，精度较低。

### 5.3 四元数归一化

每一步积分后，对四元数进行归一化处理：
$$\mathbf{q} \leftarrow \frac{\mathbf{q}}{\|\mathbf{q}\|}$$

实现位于 `body.cpp` 的 `refresh()` 函数。

---

## 6. 脚本系统

### 6.1 Input解析器

`Input`类负责读取脚本文件，逐行解析命令。解析流程：

1. 读取一行文本
2. 去除注释（`#`后内容）
3. 变量替换（`$var`或`${var}`）
4. 分词（按空白分割）
5. 匹配命令并执行

### 6.2 变量类型

| 类型 | 脚本定义 | 说明 |
|------|---------|------|
| `equal` | `variable a equal 3.14` | 数值变量，支持数学表达式 |
| `string` | `variable s string hello` | 字符串变量 |
| `loop` | `variable i loop 10` | 循环变量，配合next/jump使用 |
| `index` | `variable j index 1 2 3 5` | 索引变量，依次取列表中的值 |
| `getenv` | `variable e getenv PATH` | 读取系统环境变量 |

### 6.3 控制流

**条件判断：**
```
if "条件表达式" then "命令1"
if "条件表达式" then "命令1" else "命令2"
```

支持的比较运算符：`==` `!=` `<` `<=` `>` `>=`
支持的逻辑运算符：`&&` `||`

**标签与跳转：**
```
label 标签名
jump 文件名 标签名
```

**循环：**
```
variable i loop N
label start
  # 循环体
  next i
jump 文件名 start
```

`next`命令递增循环/索引变量，若未到达上限则跳转到文件开头继续执行。

### 6.4 命令速查表

| 命令 | 语法 | 说明 |
|------|------|------|
| `create body` | `create body 名称 属性...` | 创建刚体 |
| `create joint` | `create joint 名称 类型 body1 b1 body2 b2 ...` | 创建约束 |
| `change body` | `change body 名称 属性 值...` | 修改刚体属性 |
| `change joint` | `change joint 名称 属性 值...` | 修改约束属性 |
| `system` | `system 子命令 参数...` | 系统管理 |
| `run` | `run N [upto] [start S] [every E "cmd"]` | 运行求解 |
| `compute` | `compute 名称 body 刚体名 量...` | 定义计算量 |
| `stats` | `stats N` | 设置输出频率 |
| `stats_style` | `stats_style 关键字...` | 设置输出内容 |
| `variable` | `variable 名称 类型 值...` | 定义变量 |
| `print` | `print "文本"` | 输出文本 |
| `echo` | `echo screen/log/both` | 脚本回显 |
| `label` | `label 名称` | 定义标签 |
| `jump` | `jump 文件名 [标签名]` | 跳转 |
| `next` | `next 变量名` | 递增循环变量 |
| `if` | `if "条件" then "命令" [else "命令"]` | 条件执行 |
| `log` | `log 文件名` | 打开日志文件 |

---

## 7. 计算与输出系统

### 7.1 Compute机制

`Compute`类用于从仿真中提取数据。当前支持：

**ComputeBody** — 提取刚体状态量

```bash
compute 计算名 body 刚体名 量1 量2 ...
```

| 量 | 分量数 | 说明 |
|----|-------|------|
| `pos` | 3 | 惯性系下质心位置 |
| `vel` | 3 | 惯性系下质心速度 |
| `quat` | 4 | 姿态四元数 |
| `omega` | 3 | 体坐标系下角速度 |

数据存储在 `Compute::vector` 数组中，可被stats和result引用。

### 7.2 Stats输出

Stats负责在仿真运行过程中定期向屏幕输出信息。

```bash
stats N                    # 每N步输出一次
stats_style 关键字列表     # 设置输出格式
```

内建关键字：`step`（步数）、`cpu`（CPU时间）、`dt`（时间步长）、`time`（物理时间）

引用计算量：`c_名称`（标量）、`c_名称[N]`（第N分量）、`c_名称[*]`（所有分量）

引用变量：`v_名称`

### 7.3 Result输出

Result负责将计算量输出到文件。

```bash
result 结果名 every N 文件名 关键字列表
```

---

## 8. 内存管理

### 8.1 Memory工具类

`Memory`类提供模板化的数组分配与释放功能：

```c++
template <typename TYPE>
TYPE *create(TYPE *&array, int n);          // 1D数组
template <typename TYPE>
TYPE **create(TYPE **&array, int n1, int n2); // 2D数组
template <typename TYPE>
void destroy(TYPE *&array);                 // 释放1D
template <typename TYPE>
void destroy(TYPE **&array);                // 释放2D
```

### 8.2 对象管理

- Body和Joint对象通过`muse->add_Body()`和`muse->add_Joint()`创建
- 创建时使用`memory->srealloc()`动态扩展数组
- 析构时在`MUSE::~MUSE()`中释放所有对象

---

## 9. 已知限制与注意事项

### 9.1 约束漂移

MUSE仅在加速度级别满足约束方程。由于数值积分误差的累积，位置级和速度级的约束将逐渐产生漂移。

**影响：** 长时间仿真中，球铰连接点可能逐渐分离，固支约束的刚体间可能出现微小位移。

**缓解措施：**
- 使用较小的时间步长
- 定期监控约束误差

**建议改进：** 引入Baumgarte约束稳定化方法：
$$A\ddot{\mathbf{x}} = \mathbf{b} - 2\alpha\dot{\Phi} - \beta^2\Phi$$

### 9.2 计算效率

每个时间步需要进行SVD分解，计算复杂度 $O(n^3)$（n为状态向量维度）。RK4每步需要4次SVD。

**适用规模：** 建议刚体数量不超过数十个。

### 9.3 外力接口

当前仅支持重力和陀螺力矩。添加外力/力矩需修改 `MUSEsystem.cpp` 的 `makeBigF()` 函数。

### 9.4 未实现约束类型

以下约束类型在枚举中定义但尚未实现：
- `CARDAN` — 万向节
- `PLANE` — 平面副
- `FREE` — 自由铰

### 9.5 MPI并行

并行版尚不完善，建议使用串行版编译运行。

---

## 10. Bug修复记录

以下为代码审查中发现并修复的bug列表：

### 10.1 严重Bug（影响计算正确性）

| # | 文件 | 描述 | 修复方式 |
|---|------|------|---------|
| 1 | `create.cpp` | `create body`命令设置速度时写入了`pos`而非`vel` | `muse->body[id]->pos << vx,vy,vz` → `muse->body[id]->vel << vx,vy,vz` |
| 2 | `change.cpp` | `change body`命令设置速度时同样写入了`pos` | 同上 |
| 3 | `muse.cpp` | MPI通信器`world`在使用前未初始化 | 在`MPI_Comm_rank`调用前添加`world = communicator` |
| 4 | `MUSEsystem.cpp` | `makeBigF()`遗漏了陀螺力矩项$\omega\times(J\omega)$，导致旋转体动力学完全错误 | 添加陀螺力矩计算 |
| 5 | `joint_fix.cpp` | A1矩阵位置-旋转耦合项符号错误：应为$-[\mathbf{p}]_\times T_I$但写成了$+[\mathbf{p}]_\times T_I$ | `crsp1_I * T1_I` → `-crsp1_I * T1_I` |
| 6 | `MUSEsystem.cpp` | `solve()`和`x2body()`中使用`muse->body[ibody]`而非系统内部的`body[ibody]`，导致系统刚体与全局刚体不一致时出错 | 改为使用`body[ibody]` |

### 10.2 中等Bug（可能导致崩溃或数据错误）

| # | 文件 | 描述 | 修复方式 |
|---|------|------|---------|
| 7 | `math_extra.h` | `scale4()`函数访问`v[4]`（数组越界，应为`v[3]`） | `v[4]` → `v[3]` |
| 8 | `math_extra.h` | `copy_mat3()`自赋值：`mat2 = mat2`应为`mat2 = mat1` | `mat2 = mat2` → `mat2 = mat1` |
| 9 | `MUSEsystem.cpp` | `removebodys`命令终止符错误地设为`"/addbodys"` | → `"/removebodys"` |
| 10 | `MUSEsystem.cpp` | `removejoints`命令终止符错误地设为`"/addjoints"` | → `"/removejoints"` |
| 11 | `MUSEsystem.cpp` | `remove_Body()`在移除后访问已越界的索引 | 移除前保存指针 |
| 12 | `MUSEsystem.cpp` | `remove_Joint()`同样的越界问题 | 同上 |
| 13 | `variable.cpp` | `GETENV`类型变量分配`num[nvar]=1`但写入`data[nvar][1]`（越界） | `num[nvar]` 改为 2 |
| 14 | `error.cpp` | `warning()`和`message()`应写入`logfile`但实际写入了`screen` | `fprintf(screen,...)` → `fprintf(logfile,...)` |

### 10.3 低等Bug（内存泄漏/功能缺失）

| # | 文件 | 描述 | 修复方式 |
|---|------|------|---------|
| 15 | `body.cpp` | `set_Name()`未释放旧`name`内存（内存泄漏） | 添加`delete[] name` |
| 16 | `joint.cpp` | `set_Name()`同样的内存泄漏 | 同上 |
| 17 | `joint.cpp` | `set_type_by_name()`缺少"hinge"和"slide"类型 | 添加对应分支 |
| 18 | `joint_hinge.cpp` | `axis2`成员变量每步被覆盖（应为局部变量） | 改用局部变量 |
| 19 | `compute_body.cpp` | 错误消息引用`arg[4]`但应为`arg[2]` | 修正索引 |
| 20 | `output.cpp` | 析构函数中结果清理代码被注释掉（内存泄漏） | 取消注释 |
| 21 | `create.cpp` | `create_joint`中冗余的`set_type`调用 | 删除冗余调用 |

---

## 附录：数学函数参考 (math_extra.h)

| 函数 | 签名 | 说明 |
|------|------|------|
| `crs` | `Matrix3d crs(Vector3d v)` | 返回向量v的叉积矩阵（反对称矩阵） |
| `quat_to_mat` | `void quat_to_mat(double *q, double mat[3][3])` | 四元数转旋转矩阵 |
| `quat_to_mat_trans` | `void quat_to_mat_trans(double *q, double mat[3][3])` | 四元数转旋转矩阵的转置 |
| `normalize4` | `void normalize4(double *v)` | 四元数归一化 |
| `scale4` | `void scale4(double s, double *v)` | 四维向量缩放 |
| `copy_mat3` | `void copy_mat3(Matrix3d &mat1, Matrix3d &mat2)` | 复制3×3矩阵 |
