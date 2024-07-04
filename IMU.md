# 本文参考资料

[个人blog ](https://gutsgwh1997.github.io/2020/05/01/IMU%E9%A2%84%E7%A7%AF%E5%88%86-%E4%B8%80/)

[csdn 关于VIO中IMU预积分的讲解](https://blog.csdn.net/guoyunlei/article/details/90642932)

[ORB_SLAM3中IMU预积分过程原理分析](https://blog.csdn.net/weixin_37835423/article/details/122370251)

[VIO初始化相关Paper简单梳理 | 一索哥传奇 (zhehangt.github.io)](https://zhehangt.github.io/2019/03/23/SLAM/Basic/VIOInit/)

[On-Manifold Preintegration for Real-Time Visual-Inertial Odometry 论文笔记 | 一索哥传奇 (zhehangt.github.io) 个人blog  结合因子图一起讲了](https://zhehangt.github.io/2017/11/07/SLAM/Basic/PreintegratedIMU/)

[IMU预积分的理解和推导  个人觉得很详细 看这一篇就行](https://zhuanlan.zhihu.com/p/473227932)

[imu预积分原理的个人理解 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/438525032?utm_id=0)

[如何理解IMU以及预积分](https://zhuanlan.zhihu.com/p/38009126)

[古月居 IMU预积分](https://www.guyuehome.com/17443)

[博客园 IMU预积分](https://www.cnblogs.com/weihao-ysgs/p/IMU-Pre-Integration.html#242-%E5%9F%BA%E4%BA%8E%E4%B8%80%E9%98%B6%E6%B3%B0%E5%8B%92%E5%B1%95%E5%BC%80%E7%9A%84%E8%AF%AF%E5%B7%AE%E9%80%92%E6%8E%A8%E6%96%B9%E7%A8%8B)

[github IMU预积分邱的文档](https://github.com/PetWorm/IMU-Preintegration-Propogation-Doc?tab=readme-ov-file)

# 1. 预积分引出

## 状态估计

> Forster C, Carlone L, Dellaert F, Scaramuzza D. On-manifold preintegration theory for fast and accurate visual-inertial navigation. arxiv 2015. arxiv preprint arxiv:1512.02363.

所有的SLAM问题都可以建模成状态估计问题，通过相机或者激光的约束方程，来对需要估计的状态进行约束，从而得到状态估计的最优估计。
$$
x_i \doteq [R_i, p_i, v_i, b_i] \quad
$$
其中下标$i$表示某个时刻，$(R_i, p_i)$表示机器人的位姿，即旋转矩阵和平移向量。$v_i \in \mathbb{R^3} $表示速度。$b_i=[b_i^g, b_i^a] \in \mathbb{R^6}$表示IMU中加速度计和陀螺仪的偏差。**$X_k \doteq \{x_i\}_{i\in K_k}$都是在系统运行过程中未知的，需要我们通过观测数据来进行估计的。**

OK，那我们有哪些**观测数据**呢？当然就是相机数据和IMU数据了。用如下变量表示：
$$
Z_k \doteq \{C_i, I_{ij}\}_{(i,j) \in K_k} 
$$
其中$C_i$表示图像关键帧，$I_{ij}$表示两个连续关键帧之间的IMU数据。

因此整个状态估计问题就可以建模成基于观测数据$Z_k$，求$X_k$的最大后验
$$
p(\mathcal{X}_{k}|\mathcal{Z}_{k})\propto p(\mathcal{X}_{0})p(\mathcal{Z}_{k}|\mathcal{X}_{k})\stackrel{(a)}{=}p(\mathcal{X}_{0})\prod_{(i,j)\in\mathcal{K}_{k}}p(\mathcal{C}_{i},\mathcal{I}_{ij}|\mathcal{X}_{k})\\\stackrel{(b)}{=}p(\mathcal{X}_{0})\prod_{(i,j)\in\mathcal{K}_{k}}p(\mathcal{I}_{ij}|\mathbf{x}_{i},\mathbf{x}_{j})\prod_{i\in\mathcal{K}_{k}}\prod_{l\in\mathcal{C}_{i}}p(\mathbf{z}_{il}|\mathbf{x}_{i}).\quad(25)
$$

$$
\mathcal{X}_{k}^{\star}\doteq\arg\min_{\mathcal{X}_{k}} -\log_{e} p(\mathcal{X}_{k}|\mathcal{Z}_{k})\text{(26)}\\=\arg\min_{\mathcal{X}_{k}} \|\mathbf{r}_{0}\|_{\boldsymbol{\Sigma}_{0}}^{2}+\sum_{(i,j)\in\mathcal{K}_{k}}\|\mathbf{r}_{\mathcal{I}_{ij}}\|_{\boldsymbol{\Sigma}_{ij}}^{2}+\sum_{i\in\mathcal{K}_{k}}\sum_{l\in\mathcal{C}_{i}}\|\mathbf{r}_{\mathcal{C}_{il}}\|_{\boldsymbol{\Sigma}_{\mathcal{C}}}^{2}
$$

其中$r_{I_{i,j}}$是基于IMU数据的残差值，$r_{C_{i,l}}$是基于图像数据的残差值。因此，要求解这个状态估计问题，理清楚$r_{I_{i,j}}$和$r_{C_{i,l}}$的表达方式非常重要。

# 2. IMU预积分的思路

[IMU预积分思维导图](https://download.csdn.net/download/qq_53131867/89515016?spm=1001.2014.3001.5503)

# 3. IMU预积分推导过程

## 3.1 推导前预备知识

[IMU预积分思维导图](https://download.csdn.net/download/qq_53131867/89515016?spm=1001.2014.3001.5503)

李群李代数

## 3.2 IMU器件测量模型和运动学模型

### 3.2.1 测量模型

> **陀螺仪和加速度计的量测：是相对于惯性空间的测量值在载体坐标系下的投影。**在Globally referenced的frame中积分计算导航解时，IMU测量值首先需要被转换到Globally regerenced的参考系下。为了完成这个转换，我们需要获取载体在Globally frame中的初始位置和姿态，以及加速度二次积分过程中需要的初始速度（这也被称为初始条件要求）。

一般的低成本imu的读数都是基于IMU坐标系。

根据imu的误差模型我们可以假设：**角速度读数**会在真实值的基础上受到bias和白噪声的影响，**加速度**除了受到bias和噪声的影响，还会受到重力的影响。

#### 陀螺测量模型：

$$
\mathbf{\tilde{\omega}}_{wb}^b\left(t\right)=\mathbf{\omega}_{wb}^b\left(t\right)+\mathbf{b}_g\left(t\right)+\mathbf{\eta}_g\left(t\right)
$$

其中$\mathbf{b}_\mathrm{g}$是随时间缓慢变化的 bias, $\mathbf{\eta}_\mathrm{g}$ 是白噪声。该模型利用了 Static World Assumption,（忽略地球自转）此测量模型不考虑地球自转世界坐标系w假设为一个惯性系 。

> 下标“g”代表陀螺仪（gyroscope）。在这个上下文中，𝑏~g~(𝑡) 是指陀螺仪的偏差，它是随时间缓慢变化的。偏差是系统误差的一部分，可以是由于传感器的制造缺陷、环境因素或其它非理想特性引起的。

- $\mathbf{\omega}$ 表示角速度矢量。
- 下标 $wb$ 表示从 \(w\) 参考系到 \(b\) 参考系的角速度。
- 上标 $b$ 表示角速度矢量是在 \(b\) 参考系下表示的。

> 各种坐标系
>
> 惯性坐标系 i系  地心地固坐标系 e系  载体坐标系 b系
>
>
> 导航坐标系（东北天） n系       相机坐标系 c系

#### 加计测量模型：

$$
\mathbf{f}^b\left(t\right)=\mathbf{R}_b^{wT}\left(\mathbf{a}^w-\mathbf{g}^w\right)+\mathbf{b}_a\left(t\right)+\mathbf{\eta}_a\left(t\right)
$$

其中$\mathbf{b}_a$是随时间缓慢变化的 bias, $\mathbf{\eta}_a$ 是白噪声。

### 3.2.2 运动模型

角速度积分得到姿态，加速度积分得到速度，速度积分得到位移。

#### 运动模型连续形式

$$
\begin{aligned}&\mathbf{R}_{b\left(t+\Delta t\right)}^w=\mathbf{R}_{b\left(t\right)}^w\mathrm{Exp}\left(\int_{t}^{t+\Delta t}\mathbf{\omega}_{wb}^b(\tau)d\tau\right)

\\&\mathbf{v}^w(t+\Delta t)=\mathbf{v}^w\left(t\right)+\int_{t}^{t+\Delta t}\mathbf{a}^w(\tau)d\tau

\\&\mathbf{p}^w(t+\Delta t)=\mathbf{p}^w\left(t\right)+\int_{t}^{t+\Delta t}\mathbf{v}^w(\tau)d\tau+\int\int_{t}^{t+\Delta t}\mathbf{a}^w(\tau)d\tau^{2}.\end{aligned}
$$

#### 运动模型离散形式→差分形式：


$$
\begin{aligned}
&\mathbf{R}^w_{b}\left(t+\Delta t\right)=\mathbf{R}_{b\left(t\right)}^w\mathrm{Exp}\left(\mathbf{\omega}_{wb}^b\left(t\right)\cdotp\Delta t\right) \\

&\mathbf{v}^w\left(t+\Delta t\right)=\mathbf{v}^w\left(t\right)+\mathbf{a}^w\left(t\right)\cdot\Delta t \\

&\text{} \mathbf{p}^w\left(t+\Delta t\right)=\mathbf{p}^w\left(t\right)+\mathbf{v}^w\left(t\right)\cdot\Delta t+\frac12\mathbf{a}^w\left(t\right)\cdot\Delta t^2 \\

\end{aligned}
$$

简写：
$$
\mathbf{R}\left(t\right)\doteq\mathbf{R}_{b(t)}^w;\quad\mathbf{w}\left(t\right)\doteq\mathbf{w}_{wb}^b\left(t\right);\quad
\mathbf{f}\left(t\right)=\mathbf{f}^b\left(t\right);\quad\\
\mathbf{v}\left(t\right)\doteq\mathbf{v}^w\left(t\right);\quad\mathbf{p}\left(t\right)\doteq\mathbf{p}^w\left(t\right);\quad\mathbf{g}\doteq\mathbf{g}^w
$$

### 3.2.3 测量模型代入运动模型

$$
\begin{aligned}
\mathbf{R}\left(t+\Delta t\right)& =\mathbf{R}\big(t\big)\cdotp\mathrm{Exp}\big(\mathbf{\omega}\big(t\big)\cdotp\Delta t\big) \\
&=\mathbf{R}\left(t\right)\cdotp\mathrm{Exp}\left(\left(\tilde{\mathbf{\omega}}\left(t\right)-\mathbf{b}_g\left(t\right)-\mathbf{\eta}_{gd}\left(t\right)\right)\cdotp\Delta t\right)
\end{aligned}

\\
\begin{aligned}\mathbf{v}\left(t+\Delta t\right)&=\mathbf{v}\left(t\right)+\mathbf{a}^w\left(t\right)\cdot\Delta t\\&=\mathbf{v}\left(t\right)+\mathbf{R}\left(t\right)\cdot\left(\tilde{\mathbf{f}}\left(t\right)-\mathbf{b}_a\left(t\right)-\mathbf{\eta}_{ad}\left(t\right)\right)\cdot\Delta t+\mathbf{g}\cdot\Delta t\end{aligned}

\\
\begin{aligned}
\mathbf{p}\big(t+\Delta t\big)& =\mathbf{p}\left(t\right)+\mathbf{v}\left(t\right)\cdot\Delta t+\frac{1}{2}\mathbf{a}^{w}\left(t\right)\cdot\Delta t^{2} \\
&=\mathbf{p}\left(t\right)+\mathbf{v}\left(t\right)\cdot\Delta t+\frac12{\left[\mathbf{R}\left(t\right)\cdot\left(\mathbf{\tilde{f}}\left(t\right)-\mathbf{b}_a\left(t\right)-\mathbf{\eta}_{ad}\left(t\right)\right)+\mathbf{g}\right]}\cdot\Delta t^2 \\
&=\mathbf{p}\left(t\right)+\mathbf{v}\left(t\right)\cdot\Delta t+\frac12\mathbf{g}\cdot\Delta t^2+\frac12\mathbf{R}\left(t\right)\cdot\left(\tilde{\mathbf{f}}\left(t\right)-\mathbf{b}_a\left(t\right)-\mathbf{\eta}_{ad}\left(t\right)\right)\cdot\Delta t^2
\end{aligned}
$$

:herb:进一步：假设Δt恒定（即采样频率不变），每个离散时刻由0,1,2,...k 表示，前述三个离散运动方程可进一步简化（符号简化）为

（也可以叫做IMU积分 它们都是由k时刻对应的值加上从k到k+1的变化量求得的，而这个变化量是由IMU积分获得的）
$$
\begin{aligned}
&\mathbf{R}_{k+1}=\mathbf{R}_{k}\cdot\mathrm{Exp}\Big(\Big(\tilde{\mathbf{\omega}}_{k}-\mathbf{b}_{k}^{g}-\mathbf{\eta}_{k}^{gd}\Big)\cdotp\Delta t\Big) \\
&\mathbf{v}_{k+1}=\mathbf{v}_{k}+\mathbf{R}_{k}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{ad}\right)\cdot\Delta t+\mathbf{g}\cdot\Delta t \\
&\mathbf{p}_{k+1}=\mathbf{p}_{k}+\mathbf{v}_{k}\cdot\Delta t+\frac{1}{2}\mathbf{g}\cdot\Delta t^{2}+\frac{1}{2}\mathbf{R}_{k}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{ad}\right)\cdot\Delta t^{2}
\end{aligned}
$$
**上述方程描述了IMU测量数据和系统状态（姿态、速度和位置）之间的关系。**  也就是说，**可以根据IMU数据得到相应的状态量**。



:thinking:这个式子存在一些问题：

:one:从理论上是可以作为$r_{I_{i,j}}$的约束条件的，但是由于IMU获取观测数据的频率非常高，这会导致约束条件过多，从计算速度上来说是不可行的。和其他传感器相比，即使是低成本的IMU，采样**频率也很高**。这样在融合初始化时，需要对边缘化滤波器(marginalizing filter)(如EKF FGO)进行高速率的更新，或者延迟状态方法需要大量的位姿状态。

:two:另一个问题，特别是对于批处理初始化的方法(batch initialized inertial methods)，**它需要存储大量的惯性观测**，一旦初始条件可被观测，就需要在批滤波器中处理这些惯性观测。

如果在初始条件未知的情况下，这些观测值可以被整合在一起，那么大量的惯性观测值就可以被看作是滤波器中的一个单独的观测值，从而避免前面列出的问题。**基本思路就是将两个关键帧（i和j）之间的所有IMU数据进行一个计算，从而构成关键帧i和关键帧j的一个相对运动约束**。

<img src="D:\typora\pic\image-20240701132730007.png" alt="image-20240701132730007" style="zoom:67%;" />

### 3.2.4 关键帧约束 IMU积分

将两关键帧之间的所有imu测量值转换成一个测量值，这样就可以只加入一个观测约束，减少了约束数量，可有效的保证vio过程的实时进行。假设两相邻关键帧获取时刻采集的imu帧索引为i和j。则j时刻的状态信息可由i时刻状态信息通过如下积分获得:
$$
\begin{aligned}
& \mathbf{R}_{j}=\mathbf{R}_{i}\cdot\prod_{k=i}^{j-1}\mathrm{Exp}\Big(\Big(\tilde{\mathbf{\omega}}_{k}-\mathbf{b}_{k}^{g}-\mathbf{\eta}_{k}^{gd}\Big)\cdot\Delta t\Big) \\
&\mathbf{v}_j=\mathbf{v}_i+\mathbf{g}\cdot\Delta t_{ij}+\sum_{k=i}^{j-1}\mathbf{R}_k\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_k^a-\mathbf{\eta}_k^{ad}\right)\cdot\Delta t \\
&\mathbf{p}_{j}=\mathbf{p}_{i}+\sum_{k=i}^{j-1}\mathbf{v}_{k}\cdot\Delta t+\frac{j-i}{2}\mathbf{g}\cdot\Delta t^{2}+\frac{1}{2}\sum_{k=i}^{j-1}\mathbf{R}_{k}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{ad}\right)\cdot\Delta t^{2} \\
&=\mathbf{p}_i+\sum_{k=i}^{j-1}\left[\mathbf{v}_k\cdot\Delta t+\frac12\mathbf{g}\cdot\Delta t^2+\frac12\mathbf{R}_k\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_k^a-\mathbf{\eta}_k^{ad}\right)\cdot\Delta t^2\right] \\
&\text{其中}\Delta t_{ij}=\sum_{k=i}^{j-1}\Delta t=\bigl( j-i\bigr)\Delta t
\end{aligned}\quad(1)
$$


:thinking:但是上式（1）存在一个问题，两个关键帧解算之间的IMU积分需要给定第一个解算的状态估计量作为积分初始条件。而每次优化迭代，这些状态估计量都会更新，这就需要不断重复地进行关键帧之间的IMU 积分。

具体说：

当$t_i$时刻的状态$[\mathbf{p_i,v_i,R_i,b_i^a,b_i^g}]$（如$R_{i}$）**发生变化时，所有的运动约束关系都需要重新计算。**即每个时刻的位置（$\mathbf{p}_j$）、速度（$\mathbf{v}_j$）和姿态（$\mathrm{R}_j$）的更新依赖于前一时刻的状态（$\mathbf{p}_i, \mathbf{v}_i, \mathrm{R}_i$）。**这种依赖性意味着，每当初始时刻$i$的状态发生变化时，所有从$i$到$j$的传播都需要被重新计算。**这在实际应用中尤其在SLAM（同时定位与地图构建）或其他导航应用中会导致计算效率低下，特别是在优化或状态估计更新时。

> 每次拿到IMU数据，**更新速度**的时候，会受到姿态的影响（因为朝向不同，速度方向就不一样），因此更新速度前需要知道该时刻的姿态，在**更新平移**时，需要每个IMU时刻速度和旋转。
>
> 在构建优化问题时，会将相机或激光关键帧的pvq（位置、速度、旋转）以及imu bias作为状态量进行优化。**在求解优化问题时，会不断迭代更新这些状态量。所以，我们在求解优化问题的过程中每迭代一次，更新了一下关键帧的位姿、速度和IMU bias，就需要重复一次积分操作**，要知道我们在优化的时候不止迭代一次的，这样就会花费大量的时间重新积分，显然是不太合适的。



## 3.3 IMU预积分

> IMU预积分的初衷，是将帧与帧之间的IMU相对测量信息转换为约束载体姿态的边加入到优化框架中来。

:sun_with_face:  为了解决这个问题，引入了一个增量的概念，即“预积分”这一概念，**使得$i$时刻和$j$时刻之间的约束关系只与IMU的观测数据有关，而与当前的状态无关。**

预积分的核心思想是将IMU的观测数据（加速度计和陀螺仪的读数）在两个时间点之间的影响积累起来，形成独立于具体状态（如$\mathbf{p}_i, \mathbf{v}_i, \mathrm{R}_i$）的增量（$\Delta\mathrm{R}_{ij}, \Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$）。这样，这些**增量只与IMU的观测数据相关，与初始状态无关，可以提前计算并重复使用。**

<img src="D:\typora\pic\image-20240704170351643.png" alt="image-20240704170351643" style="zoom:67%;" />

从公式(2)可以看出右边积分部分只与$[\mathbf{b}_{\mathrm{i}}^{\mathrm{a}},\mathbf{b}_{\mathrm{i}}^{\mathrm{g}}]$ 状态有关，按当前的公式来说，偏置发生改变，右侧的积分过程需要重新进行，但由于偏置变化较小，可以用近似的方法来计算，后面3.3节会专门来说这个问题。

式（2）是预积分中的关键，它可以直接利用IMU的观测数据计算得到，作为$r_{I_{i,j}}$的约束条件。后续所有的公式都是围绕怎么计算这个公式，以及如何在非线性优化中计算状态估计的更新量。

> :whale2:上面公式中的$\Delta\mathbf{v}_{ij}$、$\Delta\mathbf{p}_{ij}$并不是通常意义上的速度和位置变化量，而是根据IMU加速度计的测量值计算出来的所谓的位移和速度增量，由于IMU加速度测量值耦合了重力加速度，因此对应的IMU预积分真值也必须含有一个重力加速度的分量，否则无法解释速度的变化量为什么还要减去 g$\cdot\Delta t_ij$ .



## 3.4 预积分测量值和测量噪声

:cactus:  式(2)是预积分的理想形式，无法根据测量值直接进行计算【其中在**连乘与连加中包含零偏估计值**，**还有未知的噪声项**】，因此需要对这些量进行分离：预积分测量值（含 IMU 测量值及 bias 估计值）与理想值之间的关系，即形如**“测量值=理想值’+’噪声”**的形式（旋转量为乘性）

**【假设零偏已知，然后将噪声项分离出来】**这里做一个假设，认为预积分计算区间内(和视觉融合时，通常是两帧间)的 bias 相等，即$\mathbf{b}_i^g=\mathbf{b}_{i+1}^g=\cdots=\mathbf{b}_j^g$ 以及$\mathbf{b}_i^a=\mathbf{b}_{i+1}^a=\cdots=\mathbf{b}_j^a$。

> 具体一步步推导 [IMU预积分的理解和推导 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/473227932)

### 3.4.1 预积分测量值

#### 3.4.1.1 $\Delta\mathbf{R}_{ij}$

$$
\begin{aligned}
\Delta\mathbf{R}_{ij}& =\prod_{k=i}^{j-1}\operatorname{Exp}\Big(\Big(\tilde{\boldsymbol{\omega}}_k-\mathbf{b}_i^g\Big)\Delta t-\mathbf{\eta}_k^{gd}\Delta t\Big) \\
&\overset{1}{\operatorname*{\approx}}\prod_{k=i}^{j-1}\left\{\operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_k-\mathbf{b}_i^g\right)\Delta t\right)\cdotp\operatorname{Exp}\left(-\mathbf{J}_r\left(\left(\tilde{\mathbf{\omega}}_k-\mathbf{b}_i^g\right)\Delta t\right)\cdotp\mathbf{\eta}_k^{gd}\Delta t\right)\right\} \\
&\overset{2}{\operatorname*{=}}\Delta\tilde{\mathbf{R}}_{ij}\cdot\prod_{k=i}^{j-1}\operatorname{Exp}\left(-\Delta\tilde{\mathbf{R}}_{k+1 j}^T\cdot\mathbf{J}_r^k\cdot\mathbf{\eta}_k^{gd}\Delta t\right)
\end{aligned}
$$

于是
$$
\Delta\mathbf{R}_{ij}\triangleq\Delta\tilde{\mathbf{R}}_{ij}\cdot\mathrm{Exp}{\left(-\delta\vec{\phi}_{ij}\right)}
$$
$\begin{aligned}&\Delta\tilde{\mathbf{R}}_{ij}\text{ 即PVQ增量测量值,它由陀螺仪测量值和对陀螺仪偏差的估计得到,而 }\delta\vec{\phi}_{ij}\text{ 或}\\&\mathrm{Exp}\left(\delta\vec{\phi}_{ij}\right)\text{即测量噪声。}\end{aligned}$

#### 3.4.1.2 $\Delta\mathbf{v}_{ij}$

将$\Delta\mathbf{R}_{ij}\triangleq\Delta\mathbf{\tilde{R}}_{ij}\cdot\mathbf{Exp}\Big(-\delta\vec{\phi}_{ij}\Big)$代入 $\Delta\mathbf{v}_{ij}$得：
$$
\begin{aligned}
\Delta\mathbf{V}_{ij}& =\sum_{k=i}^{j-1}\Delta\mathbf{R}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a-\mathbf{\eta}_k^{ad}\right)\cdot\Delta t \\
&\boldsymbol{\approx}\sum_{k=i}^{j-1}\Delta\tilde{\mathbf{R}}_{ik}\cdot\mathrm{Exp}\Big(-\delta\vec{\phi}_{ik}\Big)\cdot\Big(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a-\mathbf{\eta}_k^{ad}\Big)\cdot\Delta t \\
&\overset{1}{\operatorname*{\approx}}\sum_{k=i}^{j-1}\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\mathbf{I}-\delta\vec{\phi}_{ik}^{\wedge}\right)\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\mathbf{\eta}_{k}^{ad}\right)\cdot\Delta t \\
&\overset{2}{\operatorname*{\approx}}\sum_{k=i}^{j-1}\biggl[\Delta\tilde{\mathbf{R}}_{ik}\cdot\biggl(\mathbf{I}-\delta\vec{\phi}_{ik}^{\wedge}\biggr)\cdot\biggl(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\biggr)\cdot\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_{k}^{ad}\Delta t\biggr] \\
&=\sum_{k=i}^{3}\left[\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)\cdot\Delta t+\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge}\cdot\delta\tilde{\phi}_{ik}\cdot\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_{k}^{ad}\Delta t\right] \\
&=\sum_{k=i}^{j-1}\biggl[\Delta\tilde{\mathbf{R}}_{ik}\cdot\biggl(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\biggr)\cdot\Delta t\biggr]+\sum_{k=i}^{j-1}\biggl[\Delta\tilde{\mathbf{R}}_{ik}\cdot\biggl(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\biggr)^\wedge\cdot\delta\vec{\phi}_{ik}\cdot\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_k^{ad}\Delta t\biggr]
\end{aligned}
$$
再令
$$
\begin{aligned}\Delta\tilde{\mathbf{v}}_{ij}&\triangleq\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)\cdot\Delta t\right]\\\delta\mathbf{v}_{ij}&\triangleq\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{R}}_{ik}\eta_{k}^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge}\cdot\delta\vec{\phi}_{ik}\cdot\Delta t\right]\end{aligned}
$$
即
$$
\Delta\mathbf{v}_{ij}\triangleq\Delta\tilde{\mathbf{v}}_{ij}-\delta\mathbf{v}_{ij}
$$
$\tilde{\mathbf{v}}_{ij}\text{ 即速度增量测量值,它由IMU测量值和对偏差的估计或猜测计算得到。 }\delta\mathbf{v}_{ij}\text{即其测量噪声。}$

#### 3.4.1.3 $\Delta\mathbf{p}_{ij}$

$$
\begin{aligned}
\Delta\mathbf{p}_{ij}& =\sum_{k=i}^{j-1}\left[\Delta\mathbf{v}_{ik}\cdot\Delta t+\frac12\Delta\mathbf{R}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a-\boldsymbol{\eta}_k^{ad}\right)\cdot\Delta t^2\right] \\
&\approx\sum_{k=i}^{j-1}\left[(\Delta\tilde{\mathbf{v}}_{ik}-\delta\mathbf{v}_{ik})\cdot\Delta t+\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\mathrm{Exp}{\left(-\delta\vec{\phi}_{ik}\right)}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a-\boldsymbol{\eta}_k^{ad}\right)\cdot\Delta t^2\right. \\
&\overset{(1)}{\operatorname*{\approx}}\sum_{k=i}^{j-1}\left[(\Delta\tilde{\mathbf{v}}_{ik}-\delta\mathbf{v}_{ik})\cdot\Delta t+\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\mathbf{I}-\delta\vec{\phi}_{ik}^{\wedge}\right)\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a-\boldsymbol{\eta}_k^{ad}\right)\cdot\Delta t^2\right] \\
&\overset{(2)}{\operatorname*{\approx}}\sum_{k=i}^{j-1}\left[(\Delta\tilde{\mathbf{v}}_{ik}-\delta\mathbf{v}_{ik})\cdot\Delta t\right. \\
&\left.+\frac12\Delta\tilde{\mathbf{R}}_{ik}\right.\cdot\left(\mathbf{I}-\delta\vec{\phi}_{ik}^{\wedge}\right)\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)\cdot\Delta t^2-\frac12\Delta\tilde{\mathbf{R}}_{ik}\boldsymbol{\eta}_{k}^{ad}\Delta t^2 \\
&\overset{(3)}{\operatorname*{=}}\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{v}}_{ik}\Delta t+\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)\Delta t^2\right. \\
&+\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^{\wedge}\delta\vec{\phi}_{ik}\Delta t^2-\frac12\Delta\tilde{\mathbf{R}}_{ik}\boldsymbol{\eta}_k^{ad}\Delta t^2-\delta\mathbf{v}_{ik}\Delta t
\end{aligned}
$$

于是：
$$
\begin{aligned}
&\Delta\tilde{\mathbf{p}}_{ij}\triangleq\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{v}}_{ik}\Delta t+\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)\Delta t^2\right] \\
&\delta\mathbf{p}_{ij}\triangleq\sum_{k=i}^{j-1}\left[\delta\mathbf{v}_{ik}\Delta t-\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^{\wedge}\delta\vec{\phi}_{ik}\Delta t^2+\frac12\Delta\tilde{\mathbf{R}}_{ik}\boldsymbol{\eta}_k^{ad}\Delta t^2\right]
\end{aligned}
$$
则
$$
\Delta\mathbf{p}_{ij}\triangleq\Delta\tilde{\mathbf{p}}_{ij}-\delta\mathbf{p}_{ij}
$$
$\Delta\tilde{\mathbf{p}}_{ij}\text{ 即位置增量测量值,它由IMU测量值和对偏差的估计得到。 }\delta\mathbf{p}_{ij}\text{即其测量噪声}$

#### 3.4.1.5 总结

上面得到PVQ增量真值和测量值的关系如下：
$$
\begin{aligned}
&\Delta\mathbf{R}_{ij}\triangleq\Delta\tilde{\mathbf{R}}_{ij}\cdot\mathrm{Exp}\Big(-\delta\vec{\phi}_{ij}\Big) \\
&\Delta\mathbf{v}_{ij}\triangleq\Delta\tilde{\mathbf{v}}_{ij}-\delta\mathbf{v}_{ij} \\
&\Delta\mathbf{p}_{ij}\triangleq\Delta\tilde{\mathbf{p}}_{ij}-\delta\mathbf{p}_{ij}
\end{aligned}
$$
上述表达式即为PVQ增量测量值（含IMU测量值及偏差估计值）与真值之间的关系，即形如“测量值=真值+噪声”的形式。
$$
\begin{aligned}
&\Delta\tilde{\mathbf{R}}_{ij}\approx\Delta\mathbf{R}_{ij}\operatorname{Exp}{\left(\delta\vec{\phi}_{ij}\right)}=\mathbf{R}_i^T\mathbf{R}_j\operatorname{Exp}{\left(\delta\vec{\phi}_{ij}\right)} \\
&\Delta\tilde{\mathbf{v}}_{ij}\approx\Delta\mathbf{v}_{ij}+\delta\mathbf{v}_{ij}=\mathbf{R}_i^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)+\delta\mathbf{v}_i \\
&\Delta\tilde{\mathbf{p}}_{ij}\approx\Delta\mathbf{p}_{ij}+\delta\mathbf{p}_{ij}=\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)+\delta\mathbf{p}_{ij}
\end{aligned}
$$

### 3.4.2 测量噪声

对预积分量测噪声进行分析，目的是给出其协方差表达式，得到不确定性。
$$
\mathbf{\eta}_{ij}^\Delta\triangleq\begin{bmatrix}\delta\vec{\phi}_{ij}^T&\delta\mathbf{v}_{ij}^T&\delta\mathbf{p}_{ij}^T\end{bmatrix}^T
$$
我们希望其满足高斯分布，即$\mathbf{\eta}_{ij}^\Delta\thicksim N\left(\mathbf{0}_{9\times1},\mathbf{\Sigma}_{ij}\right)$。由于$\mathbf{\eta}_{ij}^\Delta$是$\delta\vec{\phi}_{ij}$、$\delta\mathbf{v}_{ij}$和$\delta\mathbf{p}_{ij}$的线性组合，下面分别分析这三个噪声项的分布形式。

#### 3.4.2.1 $\delta\vec{\phi}_{ij}$

> 具体还是看知乎 或者pdf 的推导  这边只有结论性的公式

对取对数
$$
\delta\vec{\phi}_{ij}=-\log\left(\prod_{k=i}^{j-1}\mathrm{Exp}{\left(-\Delta\tilde{\mathbf{R}}_{k+1j}^T\cdot\mathbf{J}_r^k\cdot\eta_k^{gd}\Delta t\right)}\right)
$$
令
$$
\boldsymbol{\xi}_{k}=\Delta\tilde{\mathbf{R}}_{k+1j}^{T}\cdot\mathbf{J}_{r}^{k}\cdot\eta_{k}^{gd}\Delta t
$$
于是：
$$
\begin{aligned}
\delta\vec{\phi}_{ij}& =-\log\left(\prod_{k=i}^{j-1}\mathrm{Exp}(-\xi_k)\right) \\
&=-\log\left(\mathrm{Exp}(-\xi_i)\prod_{k=i+1}^{j-1}\mathrm{Exp}(-\xi_k)\right) \\
&\approx-\left(-\xi_i\right.+\mathbf{I}\cdot\log\left(\prod_{k=i+1}^{j-1}\left.\mathrm{Exp}(-\xi_k)\right)\right)=\xi_i-\log\left(\prod_{k=i+1}^{j-1}\mathrm{Exp}(-\xi_k)\right) \\
&=\xi_i-\log\left(\operatorname{Exp}(-\xi_{i+1})\prod_{k=i+2}^{j-1}\operatorname{Exp}(-\xi_k)\right) \\
&\approx\xi_i+\xi_{i+1}-\log\left(\prod_{k=i+2}^{j-1}\mathrm{Exp}(-\xi_k)\right) \\
&\approx\ldots \\
&\approx\sum_{k=i}^{j-1}\xi_k
\end{aligned}
$$
注意 log(ab)=log a+log b

也就是：
$$
\delta\vec{\phi}_{ij}\approx\sum_{k=i}^{j-1}\Delta\tilde{\mathbf{R}}_{k+1 j}^T\mathbf{J}_r^k\mathbf{\eta}_k^{gd}\Delta t
$$
$\begin{aligned}&\text{由于 }\Delta\tilde{\mathbf{R}}_{k+1j}^T\mathrm{~、}\mathbf{J}_r^k\text{ 和 }\Delta t\text{ 都是已知量,而 }\mathbf{\eta}_k^{gd}\text{ 是零均值高斯噪声,因此 }\delta\vec{\phi}_{ij}\text{(的一阶近似)也}\\&\text{为零均值高斯噪声。}\end{aligned}$



#### 3.4.2.2  $\delta\mathbf{v}_{ij}$

由于$\delta\vec{\phi}_{ij}$近似拥有了高斯噪声的形式，且 $\mathbf{n}_k^{ad}$ 也是零均值高斯噪声，根据 $\delta\mathbf{v}_{ij}$ 的表达式：
$$
\delta\mathbf{v}_{ij}=\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_k^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\phi}_{ik}\cdot\Delta t\right]
$$
可知其也将拥有高斯分布的形式。

#### 3.4.2.3 $\delta\mathbf{p}_{ij}$

$$
\delta\mathbf{p}_{ij}=\sum_{k=i}^{j-1}\left[\delta\mathbf{v}_{ik}\Delta t-\frac12\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^\wedge\delta\vec{\phi}_{ik}\Delta t^2+\frac12\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_k^{ad}\Delta t^2\right]
$$

可知 $\delta\mathbf{p}_{ij}$ 也具有高斯分布的形式





### 3.4.3 噪声传递模型

下面推导预积分测量噪声的递推形式，即 $\mathbf{\eta}_{ij-1}^{\Delta}\to\mathbf{\eta}_{ij}^{\Delta}$,及其协方差$\boldsymbol\Sigma_ij$ 的递推形式，即
$\boldsymbol{\Sigma}_{ij-1}\to\boldsymbol{\Sigma}_{ij}$。先推导$\delta \vec{\phi } _{ij- 1}\to \delta \vec{\phi } _{ij}$, $\delta \mathbf{v} _{ij- 1}\to \delta \mathbf{v} _{ij}$和$\delta\mathbf{p}_ij-1\to\delta\mathbf{p}_{ij}$ 。

#### 3.4.3.1  $\delta\vec{\phi}_{ij-1}\to\delta\vec{\phi}_{ij}$


$$
\begin{aligned}
\delta\phi_{ij}& \simeq\sum_{k=i}^{j-1}\Delta\tilde{\mathrm{R}}_{k+1j}^{\top}\mathrm{J}_{r}^{k}\eta_{k}^{gd}\Delta t \\
&=\sum_{k=i}^{j-2}\Delta\tilde{\mathrm{R}}_{k+1j}^{\mathrm{T}}\mathrm{J}_{r}^{k}\boldsymbol{\eta}_{k}^{gd}\Delta t+\overbrace{\Delta\tilde{\mathrm{R}}_{jj}^{\mathrm{T}}}^{=\mathbf{I}_{3\times3}} \mathrm{J}_{r}^{j-1}\boldsymbol{\eta}_{j-1}^{gd}\Delta t \\
&=\sum_{k=i}^{j-2}(\overbrace{\Delta\tilde{\mathrm{R}}_{k+1j-1}\Delta\tilde{\mathrm{R}}_{j-1j}}^{=\Delta\tilde{\mathrm{R}}_{k+1j}})^{\mathsf{T}}\mathrm{J}_{r}^{k}\eta_{k}^{gd}\Delta t+\mathrm{J}_{r}^{j-1}\eta_{j-1}^{gd}\Delta t \\
&=\Delta\tilde{\mathrm{R}}_{j-1j}^{\mathsf{T}}\sum_{k=i}^{j-2}\Delta\tilde{\mathrm{R}}_{k+1j-1}^{\mathsf{T}}\mathrm{J}_{r}^{k}\eta_{k}^{gd}\Delta t+\mathrm{J}_{r}^{j-1}\eta_{j-1}^{gd}\Delta t \\
&=\Delta\tilde{\mathrm{R}}_{j-1j}^{\mathsf{T}}\delta\phi_{ij-1}+\mathrm{J}_{r}^{j-1}\boldsymbol{\eta}_{j-1}^{gd}\Delta t.
\end{aligned}
$$


#### 3.4.3.2 $\delta\mathbf{v}_{ij-1}\to\delta\mathbf{v}_{ij}$


$$
\begin{aligned}
\delta\mathbf{v}_{ij}& =\sum_{k=i}^{j-1}\left[\Delta\tilde{\mathbf{R}}_{ik}\boldsymbol{\eta}_k^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\phi}_{ik}\cdot\Delta t\right] \\
&=\sum_{k=i}^{j-2}\biggl[\Delta\tilde{\mathbf{R}}_{ik} \mathbf{\eta}_k^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\phi}_{ik}\cdot\Delta t\biggr]... \\
&+\Delta\tilde{\mathbf{R}}_{ij-1}\boldsymbol{\eta}_{j-1}^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ij-1}\cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\boldsymbol{\phi}}_{ij-1}\cdot\Delta t \\
&=\delta\mathbf{v}_{ij-1}+\Delta\tilde{\mathbf{R}}_{ij-1}\boldsymbol{\eta}_{j-1}^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ij-1}\cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\phi}_{ij-1}\cdot\Delta t
\end{aligned}
$$




#### 3.4.3.3  $\delta\mathbf{p}_{ij-1}\to\delta\mathbf{p}_{ij}$

> 注意下面的下标 和 a要改一下

$$
\begin{aligned}
&\delta\mathbf{p}_{ij}=\sum_{k=i}^{j-1}\biggl[\delta\mathbf{v}_{ik}\Delta t-\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik} (\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a})^{\wedge}\delta\phi_{ik}\Delta t^{2}+\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\eta_{k}^{ad}\Delta t^{2}\biggr] \\
&=\sum_{k=i}^{j-2}\left[\delta\mathbf{v}_{ik}\Delta t-\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge}\delta\phi_{ik}\Delta t^{2}+\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\boldsymbol{\eta}_{k}^{ad}\Delta t^{2}\right] \\
&+\delta\mathbf{v}_{ij-1}\Delta t-\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ij-1}(\tilde{\mathbf{a}}_{j-1}-\mathbf{b}_{i}^{a})^{\wedge}\delta\phi_{ij-1}\Delta t^{2}+\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ij-1}\boldsymbol{\eta}_{j-1}^{ad}\Delta t^{2} \\
&=\delta\mathbf{p}_{ij-1}+\delta\mathbf{v}_{ij-1}\Delta t-\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ij-1}\left(\tilde{\mathbf{a}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge}\delta\phi_{ij-1}\Delta t^{2} \\
&+\frac{1}{2}\Delta\tilde{\mathsf{R}}_{ij-1}\boldsymbol{\eta}_{j-1}^{ad}\Delta t^{2}
\end{aligned}
$$



#### 3.4.3.4 总结

对预积分量测噪声进行分析，目的是给出其协方差表达式，得到不确定性。
$$
\mathbf{\eta}_{ij}^\Delta\triangleq\begin{bmatrix}\delta\vec{\phi}_{ij}^T&\delta\mathbf{v}_{ij}^T&\delta\mathbf{p}_{ij}^T\end{bmatrix}^T
$$
我们希望其满足高斯分布，即$\mathbf{\eta}_{ij}^\Delta\thicksim N\left(\mathbf{0}_{9\times1},\mathbf{\Sigma}_{ij}\right)$。由于$\mathbf{\eta}_{ij}^\Delta$是$\delta\vec{\phi}_{ij}$、$\delta\mathbf{v}_{ij}$和$\delta\mathbf{p}_{ij}$的线性组合
$$
\begin{aligned}
\delta\phi_{ij}& =\sum_{k\operatorname{=}1}^{j-1}\Delta\tilde{R}_{k\operatorname{+}1}^TJ_r^k\eta_k^{gd}\Delta t \\
&=\Delta\widetilde{R}_{j,j-1}\delta\phi_{i,j-1}+J_r^{j-1}\eta_{j-1}^{gd}\Delta t
\end{aligned}
\\
\begin{aligned}\delta v_{ij}&=\sum_{k=i}^{j-1}\biggl[\Delta\tilde{\mathbf{R}}_{ik} \mathbf{\eta}_{k}^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ik}\cdot\biggl(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\biggr)^{\wedge}\cdot\delta\vec{\phi}_{ik}\cdot\Delta t\biggr]\\&=\delta\mathbf{v}_{ij-1}+\Delta\tilde{\mathbf{R}}_{ij-1}\mathbf{\eta}_{j-1}^{ad}\Delta t-\Delta\tilde{\mathbf{R}}_{ij-1}\cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\cdot\delta\vec{\phi}_{ij-1}\cdot\Delta t\end{aligned}

\\
\begin{aligned}
\delta p_{ij}& =\sum_{k=i}^{j-1}\biggl[\boldsymbol{\delta}\mathbf{v}_{ik}\Delta t-\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge}\delta\vec{\phi}_{ik}\Delta t^{2}+\frac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\mathbf{\eta}_{k}^{ad}\Delta t^{2}\biggr] \\
&=\delta\mathbf{p}_{ij-1}+\delta\mathbf{v}_{ij-1}\Delta t-\frac12\Delta\tilde{\mathbf{R}}_{ij-1}\cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\delta\vec{\phi}_{ij-1}\Delta t^2+\frac12\Delta\tilde{\mathbf{R}}_{ij-1}\mathbf{\eta}_{j-1}^{ad}\Delta t^2
\end{aligned}
$$


$\text{综上可得}\mathbf{\eta}_{ij}^\Delta\text{的递推形式如下(令}\mathbf{\eta}_k^d=\left[\left(\mathbf{\eta}_k^{gd}\right)^T\quad\left(\mathbf{\eta}_k^{ad}\right)^T\right]^T).$


$$
\begin{aligned}\mathbf{\eta}_{ij}^{\Delta}=&\begin{bmatrix}\Delta\mathbf{\tilde{R}}_{j j-1}&\mathbf{0}&\mathbf{0}\\-\Delta\mathbf{\tilde{R}}_{ij-1}\cdot\left(\mathbf{\hat{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\Delta t&\mathbf{I}&\mathbf{0}\\\\-\frac12\Delta\mathbf{\tilde{R}}_{ij-1}\cdot\left(\mathbf{\tilde{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\Delta t^2&\Delta t\mathbf{I}&\mathbf{I}\end{bmatrix}\mathbf{\eta}_{ij-1}^\Delta\ldots\\&+\begin{bmatrix}\mathbf{J}_r^{j-1}\Delta t&\mathbf{0}\\\mathbf{0}&\Delta\tilde{\mathbf{R}}_{ij-1}\Delta t\\\\\mathbf{0}&\frac12\Delta\tilde{\mathbf{R}}_{ij-1}\Delta t^2\end{bmatrix}\mathbf{\eta}_{j-1}^d\end{aligned}
$$
令
$$
\begin{gathered}\mathbf{A}_{j-1}=\begin{bmatrix}\Delta\mathbf{\tilde{R}}_{j j-1}&\mathbf{0}&\mathbf{0}\\-\Delta\mathbf{\tilde{R}}_{ij-1}\cdot\left(\mathbf{\tilde{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\Delta t&\mathbf{I}&\mathbf{0}\\-\frac12\Delta\mathbf{\tilde{R}}_{ij-1}\cdot\left(\mathbf{\tilde{f}}_{j-1}-\mathbf{b}_i^a\right)^\wedge\Delta t^2&\Delta t\mathbf{I}&\mathbf{I}\end{bmatrix}\\\mathbf{B}_{j-1}=\begin{bmatrix}\mathbf{J}_r^{j-1}\Delta t&\mathbf{0}\\\mathbf{0}&\Delta\mathbf{\tilde{R}}_{ij-1}\Delta t\\\mathbf{0}&\frac12\Delta\mathbf{\tilde{R}}_{ij-1}\Delta t^2\end{bmatrix}\end{gathered}
$$
则
$$
\eta_{ij}^\Delta=\mathbf{A}_{j-1}\mathbf{\eta}_{ij-1}^\Delta+\mathbf{B}_{j-1}\mathbf{\eta}_{j-1}^d
$$
从上述推导可以看出，预积分噪声项的传递是线性的，因此可以直接通过如下线性传递的方式进行协方差的更新
$$
\boldsymbol{\Sigma}_{ij}=\mathbf{A}_{j-1}\boldsymbol{\Sigma}_{ij-1}\mathbf{A}_{j-1}^T+\mathbf{B}_{j-1}\boldsymbol{\Sigma}_{\mathbf{\eta}}\mathbf{B}_{j-1}^T
$$
从形式上看，IMU预积分协方差的递推形式类似于卡尔曼滤波中的状态变量协方差的预测方程，其中的$\mathbf{Q}$就相当于$\Sigma_\eta$ ,在每个递推周期都固定的加上这样一个常量噪声，表示从当前状态转移到下一个状态的过程中，存在各种噪声，总是会引入新的误差：

<img src="D:\typora\pic\image-20240704170444897.png" alt="image-20240704170444897" style="zoom:50%;" />

**IMU预积分测量噪声的协方差矩阵（即噪声分布）将用来计算信息矩阵，在优化框架中起到平衡权重的作用**。在实际应用中首先要求协方差矩阵的逆矩阵，相当于取了协方差的倒数，方差越大权重越小，反之权重越大，然后再将逆矩阵转成信息矩阵，与残差相乘，起到调节残差比例的作用。

关于噪声的内容到此为止，接下来讨论偏差的问题。



## 3.5 偏差更新时的预积分测量值更新

前面的预积分计算，都是在**假设积分区间内陀螺和加计的偏差恒定的基础上推导的**。当 bias 发生变化时，若仍按照前述公式，预积分测量值需要整个重新计算一遍，这将非常的耗费算力。为了解决这个问题，提出了利用线性化来进行偏差变化时预积分项的一阶近似更新方法。

:sailboat:符号说明

令$\overline{\mathbf{b}}_i^g$ 和$\overline{\mathbf{b}}_i^a$为旧的偏差bias,

新的 bias($\hat{\mathbf{b}}_i^g$ 和$\hat{\mathbf{b}}_i^a$) 由旧 bias （ $\overline{\mathbf{b}}_i^g$ 和$\overline{\mathbf{b}}_i^a$）与 更新量( $\delta\mathbf{b}_i^g$ 和 $\delta\mathbf{b}_i^a$) 相加得到，即$\hat{\mathbf{b}}_i^g\leftarrow\overline{\mathbf{b}}_i^g+\delta\mathbf{b}_i^g$、$\hat{\mathbf{b}}_i^a\leftarrow\overline{\mathbf{b}}_i^a+\delta\mathbf{b}_i^a$。

于是有预积分关于 bias 估计值变化的一阶近似更新公式如下：

<img src="D:\typora\pic\image-20240704170539225.png" alt="image-20240704170539225" style="zoom:57%;" />

符号简化：

<img src="D:\typora\pic\image-20240704170648791.png" alt="image-20240704170648791" style="zoom:57%;" />

得到简化后的公式如下：

<img src="D:\typora\pic\image-20240704170655022.png" alt="image-20240704170655022" style="zoom:57%;" />

**上式说明了IMU预积分是如何计算出测量值的修正值的，为什么雅可比能够起到修正值的作用？**

1. 其中的$\Delta\overline{\mathbf{R}}_{ij},\Delta\overline{\mathbf{v}}_{ij},\Delta\overline{\mathbf{p}}_{ij}$表示旧的测量值，其中包含了旧的偏差$\overline{\mathbf{b}}_i^g$和$\overline{\mathbf{b}}_i^a$ 。

2. 其中的$\Delta\hat{\mathbf{R}}_{ij},\Delta\hat{\mathbf{v}}_{ij},\Delta\hat{\mathbf{p}}_{ij}$表示新的测量值，其中包含了 新的偏差$\hat{\mathbf{b}}_i^g$和$\hat{\mathbf{b}}_i^a$ 。
3. 新偏差=旧偏差+更新量$\delta\mathbf{b}_i^g$ ,那么，**如果把测量值当做偏差的函数，只需要在旧的测量值上添加一个近似的修正量就可以获得近似的新测量值，而不需要重新积分。**
4. 而这个修正量 (增量)就是用偏差的更新量$\delta\mathbf{b}_i^g$和$\delta\mathbf{b}_i^a$乘以函数的导数(即斜率)获得。

这样一来，对于i、j两帧之间的IMU积分我们只需要做一次就可以了(即式中的$\Delta\overline{\mathbf{R}}_{ij},\Delta\overline{\mathbf{v}}_{ij},\Delta\overline{\mathbf{p}}_{ij}$ ),**通过测量值函数对偏差的偏导数(即雅可比)和偏差更新量$\delta\mathbf{b}_i^g,\delta\mathbf{b}_i^a$就可以近似的计算出修正量，获得新测量值的近似值，而不需要重新积分**。

如果优化过程中起始位姿发生了变化，则雅可比也相应更新。而偏差更新量$\delta\mathbf{b}_i^g,\delta\mathbf{b}_i^a$本身就是待优化的变量之一，自然也是相应更新。从而测量值的修正量实现了自动更新。

以上就是IMU预积分避免重新积分，降低运算量的关键。

### 3.5.1 各式中的偏导项

> 具体推导过程见pdf 

$$
\begin{aligned}
&\frac{\partial\Delta\overline{\mathbf{R}}_{ij}}{\partial\overline{\mathbf{b}}^g} =\sum_{k=i}^{j-1}\left(-\Delta\overline{\mathbf{R}}_{k+1j}^T\mathbf{J}_r^k\Delta t\right) \\
&\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^g} =-\sum_{k=i}^{j-1}\left(\Delta\overline{{\mathbf{R}}}_{ik}\cdot\left(\tilde{\mathbf{f}}_{k}-\overline{{\mathbf{b}}}_{i}^{a}\right)^{\wedge}\frac{\partial\Delta\overline{{\mathbf{R}}}_{ik}}{\partial\overline{{\mathbf{b}}}^{g}}\Delta t\right) \\
&\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^a} =-\sum_{k=i}^{j-1}\left(\Delta\overline{\mathbf{R}}_{ik}\Delta t\right) \\
&\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^g} =\sum_{k=i}^{j-1}\left[\frac{\partial\Delta\overline{\mathbf{v}}_{ik}}{\partial\overline{\mathbf{b}}^g}\Delta t-\frac12\Delta\overline{\mathbf{R}}_{ik}\cdot\left(\tilde{\mathbf{f}}_k-\overline{\mathbf{b}}_i^a\right)^{\wedge}\frac{\partial\Delta\overline{\mathbf{R}}_{ik}}{\partial\overline{\mathbf{b}}^g}\Delta t^2\right] \\
&\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^a} =\sum_{k=i}^{j-1}\left[\frac{\partial\Delta\overline{\mathbf{v}}_{ik}}{\partial\overline{\mathbf{b}}^a}\Delta t-\frac12\Delta\overline{\mathbf{R}}_{ik}\Delta t^2\right] 
\end{aligned}\\
其中\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\begin{pmatrix}\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right)\Delta t\end{pmatrix}。
$$

## 3.6 残差

> **残差**是当前时刻的**实际测量值**与**当前时刻的预测值**之间的差异，它表示当前时刻的观测值与=预测值之间的误差。 
>
> **新息**则是当前时刻的**实际测量值**与**该时刻的最优估计值**之间的差异，它表示当前时刻的观测值与最优估计值之间的误差

在实际应用中，通常以$\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j$等为导航求解的目标，同时由于IMU的偏差也是不可忽视的，因此，全部的导航状态是：
$$
\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j,\delta\mathbf{b}_i^g,\delta\mathbf{b}_i^a
$$
在进行 optimization 时，将对所有待优化的导航状态进行 lifting（可以理解为“更新”，后面将介绍这个概念，从而影响**预积分计算值**和**预积分测量值**，进而改变残差值，**optimization的最终目的是要使残差（的加权范数）最小化。**

定义增量如下：
$$
\begin{aligned}&R_i\longleftarrow R_iExp\left(\delta\phi_i\right)\text{,}\quad R_j\longleftarrow R_jExp\left(\delta\phi_j\right)\\&p_i\longleftarrow p_i+R_i\delta p_i\text{,}\quad p_j\longleftarrow p_j+R_j\delta p_j\\&v_i\longleftarrow v_i+\delta v_i\text{,}\quad v_j\longleftarrow v_j+\delta v_j\\&\delta b_i^g\longleftarrow\delta b_i^g+\delta\widetilde{b}_i^g\text{,}\quad \delta b_i^a\longleftarrow\delta b_i^a+\delta\widetilde{b}_i^a\end{aligned}
$$
这里的状态增量分别是$\delta\phi_i$,$\delta\phi_j$,$\delta p_i,\delta p_j,\delta v_i,\delta v_j,\delta\tilde{b}_i^g$,分别对应着：第$i$ 和 $j$时刻的姿态增量，第$i$ 和 $j$时刻的位置增量，第$i$ 和$j$时刻的速度增量，第$i$ 和 $j$时刻的零偏增量。

根据各预积分项的定义，可得$\Delta\mathbf{R}_{ij}$、$\Delta\mathbf{v}_{ij}$和$\Delta\mathbf{p}_{ij}$的理想值表达式如下：
$$
\begin{aligned}&\Delta\mathbf{R}_{ij}=\mathbf{R}_i^T\mathbf{R}_j\\&\Delta\mathbf{v}_{ij}=\mathbf{R}_i^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)\\&\Delta\mathbf{p}_{ij}=\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)\end{aligned}
$$

### 3.6.1 具体表达式

残差定义如下，其中第一部分是PVQ增量的估计值，需要通过非IMU的方式获得，例如点云到Map的匹配，第二部分是PVQ增量的测量值，即通过推导的“**偏差更新时的预积分测量值更新**”方法获得的修正后的测量值，这种近似的修正方式免去了积分的重新计算，是预积分降低计算量的关键。

当预积分参与到 BA 优化中时，最终是希望使得这些残差项的加权平方和最小。**之所以要加权是因为各状态量的单位不统一，需要对其进行归一化**。:herb:由于残差项 即 **预积分计算值（由状态估计计算得到）与预积分测量值之“差”**（广义的差，注意到对于ΔR来说是逆），而我们期望的估计结果是通过对状态进行无偏估计，从而使预积分计算值逼近理想值，因此权重就是前面给出的预积分噪声协方差的逆。



:heavy_exclamation_mark: 需要再次强调和格外注意的一点是，残差中待优化的状态为$\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j,\delta\mathbf{b}_i^g,\delta\mathbf{b}_i^a$ (由残差项中预积分测量值的修正方式决定),

在迭代中求解增量方程得到的增量是
$$
\delta\vec{\phi}_i,\delta\mathbf{p}_i,\delta\mathbf{v}_i,\delta\vec{\phi}_j,\delta\mathbf{p}_j,\delta\mathbf{v}_j,\widetilde{\delta\mathbf{b}}_i^\mathrm{g},\widetilde{\delta\mathbf{b}}_i^a
$$
其中$\delta\vec{\phi}_i$和$\delta\vec{\phi}_j$是姿态在切空间上的增量，而$\widetilde{\delta\mathbf{b}}_i^\mathrm{g}$ 和$\widetilde{\delta\mathbf{b}_i^a}$ 是 bias 的增量的增量。这部分概念需要读者好好思考理解，不要被 bias 弄晕了。

$\text{设第}i\text{ 时刻到 }j\text{时刻的旋转残差为}r_{\Delta R_{ij}}\text{,速度残差为}r_{\Delta v_{ij}}\text{,位置残差}r_{\Delta p_{ij}}$


$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{R}_{ij}}& \triangleq\operatorname{Log}\left\{\left[\Delta\tilde{\mathbf{R}}_{ij}\left(\overline{\mathbf{b}}_i^g\right)\cdotp\operatorname{Exp}\left(\frac{\partial\Delta\overline{\mathbf{R}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g\right)\right]^T\cdot\mathbf{R}_i^T\mathbf{R}_j\right\} \\
&\triangleq\operatorname{Log}\left[\left(\Delta\mathbf{\hat{R}}_{ij}\right)^T\Delta\mathbf{R}_{ij}\right] \\
\end{aligned}\\

\begin{aligned}\mathbf{r}_{\Delta\mathbf{v}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\left[\Delta\tilde{\mathbf{v}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{v}_{ij}-\Delta\mathbf{\hat{v}}_{ij}\end{aligned}
\\
\begin{aligned}\mathbf{r}_{\Delta\mathbf{p}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\left[\Delta\tilde{\mathbf{p}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{p}_{ij}-\Delta\mathbf{\hat{p}}_{ij}\end{aligned}
$$

前面推导的预积分测量值关于 bias 变化的修正在残差中进行了应用，这种近似的修正方式免去了积分的重新运算，是预积分技术降低计算量的关键。

在估计中，通常以$\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j$等为导航求解的目标，同时由于 bias 的作用在以mems 器件为基础的应用中不可忽视，因此 bias 也常常被当做状态量进行估计。但由残差的表达式可以看到，关于 bias 采取的是估计 bias 偏差的方式(由预积分测量值的修正方式决定),即估计$\partial\mathbf{b}_i^g$和$\delta\mathbf{b}_i^a$。所以在 IMU 预积分中，全部的导航状态是
$$
\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j,\delta\mathbf{b}_i^g,\delta\mathbf{b}_i^a
$$



在利用各类方法进行非线性最小二乘计算时，需要**提供残差关于这些状态的 Jacobian**。对于姿态来说，一般更习惯采用扰动模型（详见《视觉 SLAM 十四讲》P75，这种模型比直接对李代数求导能获得更好的 Jacobian 形式），因此为了统一状态的表述形式，**我们一般采用对扰动/摄动/增量进行求导来获取 Jacobian 矩阵**。注意到，对于除了姿态之外的其他状态来说，对于状态本身求导得到的 Jacobian 和对状态增量求导得到的 Jacobian，是完全一样的（增量在 0 处进行展开）。

关于“$\mathbf{p}_i$ 和$\mathbf{p}_j$为什么不采用类似$\mathbf{v}_i\leftarrow\mathbf{v}_i+\delta\mathbf{v}_i$ 的直接加增量 lifiting 的形式”的分析。

$i$时刻的位姿可以由下述矩阵表示：
$$
\mathbf{T}_i=\begin{bmatrix}\mathbf{R}_i&\mathbf{p}_i\\0&1\end{bmatrix}
$$
<img src="D:\typora\pic\image-20240702215941274.png" alt="image-20240702215941274" style="zoom:67%;" />

给该位姿一个右乘摄动如下：
$$
\delta\mathbf{T}_i=\begin{bmatrix}\delta\mathbf{R}_i&\delta\mathbf{p}_i\\0&1\end{bmatrix}
$$
$\text{对}\mathbf{T}_i\text{ 右乘}\delta\mathbf{T}_i\text{有}:$
$$
\begin{aligned}
\mathbf{T}_i^{\prime}& =\mathbf{T}_i\cdot\delta\mathbf{T}_i=\begin{bmatrix}\mathbf{R}_i&\mathbf{p}_i\\0&1\end{bmatrix}\begin{bmatrix}\delta\mathbf{R}_i&\delta\mathbf{p}_i\\0&1\end{bmatrix} \\
&=\begin{bmatrix}\mathbf{R}_i\cdot\delta\mathbf{R}_i&\mathbf{p}_i+\mathbf{R}_i\cdot\delta\mathbf{p}_i\\0&1\end{bmatrix}=\begin{bmatrix}\mathbf{R}_i^{\prime}&\mathbf{p}_i^{\prime}\\0&1\end{bmatrix}
\end{aligned}
$$

可以看到，当采用这种右乘方式对位姿进行摄动时，得到的旋转和平移部分就是前面给出的“lifting”的形式。因此这种右乘摄动的方式就是$\mathbf{p}_i$ 和$\mathbf{p}_j$ 的 lifting 采取特定形式的原因。

此外，熟悉非线性最小二乘方法的读者应该知道，在利用高斯牛顿或者 LM 方法进行优化求解时，每次迭代需要通过求解一个增量方程来求取状态的增量**，这个增量方程的系数矩阵由残差关于待优化状态（增量）的 Jacobian 和权重来共同计算。因此我们还需要将 Jacobian的表达式求出。**

## 3.7 雅可比

:seedling: 这些 Jacobian 分成三类： 

第一类是“0 类”（残差中不包括某些状态时，对应的 Jacobian 自然为0）； 

第二类是“线性类”（残差关于某些状态是线性的，因此对应的 Jacobian 可直接由线性系数得到）； 

第三类为“复杂类”（这种情况下的状态在残差表达式中的耦合关系比较复杂，需要对残差使用 lifting，并进行相应变形来求取 Jacobian）。

### 3.7.1 $\mathbf{r}_{\Delta\mathbf{R}_{ij}}$ 的雅可比相关

#### 3.7.1.1 0类

$\mathbf{r}_{\Delta\mathbf{R}_{ij}}$ 中不含 $\mathbf{p}_i\text{、}\mathbf{p}_j\text{、}\mathbf{v}_i\text{、}\mathbf{v}_j$ 以及 $\delta\mathbf{b}_i^a$因此$\mathbf{r}_{\Delta\mathbf{R}_{ij}}$关于这些状态增量的 Jacobian 都为0零矩阵。
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{p}_{i}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\mathbf{p}_{i}}=\mathbf{0} , \frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{p}_{j}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\mathbf{p}_{j}}=\mathbf{0} , \\\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{v}_{i}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\mathbf{v}_{i}}=\mathbf{0} , \frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{v}_{j}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\mathbf{v}_{j}}=\mathbf{0} ,\\\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\widetilde{\partial\mathbf{b}_{i}^{a}}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{b}_{i}^{a}}=\mathbf{0}
$$

#### 3.7.1.2  线性类

无

#### 3.7.1.3 复杂类

下面求关于$\delta\vec{\phi}_i(\mathbf{R}_i)$ 对应的李代数扰动)$  \delta\vec{\phi}_j(\mathbf{R}_j$ 对应的李代数扰动)和 $\widetilde{\delta\mathbf{b}_i^g}$ 的 Jacobian.

一些公式
$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{R}_{ij}}& \triangleq\operatorname{Log}\left\{\left[\Delta\tilde{\mathbf{R}}_{ij}\left(\overline{\mathbf{b}}_i^g\right)\cdotp\operatorname{Exp}\left(\frac{\partial\Delta\overline{\mathbf{R}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g\right)\right]^T\cdot\mathbf{R}_i^T\mathbf{R}_j\right\} \\
&\triangleq\operatorname{Log}\left[\left(\Delta\mathbf{\hat{R}}_{ij}\right)^T\Delta\mathbf{R}_{ij}\right] \\
\end{aligned}\\
$$

##### :one:$\delta\vec{\phi}_i$的雅可比

回顾一些公式
$$
\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_{j}\right)=\mathrm{Log}\left(\left(\Delta\mathbf{\hat{R}}_{ij}\right)^T\mathbf{R}_i^T\mathbf{R}_j\right)

\\
\Delta\mathbf{R}_{ij}\triangleq\Delta\tilde{\mathbf{R}}_{ij}\cdot\mathbf{Exp}\Big(-\delta\vec{\phi}_{ij}\Big)

\\
\Delta\mathbf{R}_{ij}\triangleq\mathbf{R}_i^T\mathbf{R}_j
\\
\begin{aligned}
&\Delta\tilde{\mathbf{R}}_{ij}\left(\hat{\mathbf{b}}_i^g\right)\approx\Delta\tilde{\mathbf{R}}_{ij}\left(\overline{\mathbf{b}}_i^g\right)\cdot\mathrm{Exp}\left(\frac{\partial\Delta\overline{\mathbf{R}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g\right)\end{aligned}
\\
\begin{aligned}\Delta\hat{\mathbf{R}}_{ij}\doteq\Delta\tilde{\mathbf{R}}_{ij}\left(\hat{\mathbf{b}}_{i}^{g}\right),&&\Delta\overline{{\mathbf{R}}}_{ij}\doteq\Delta\tilde{\mathbf{R}}_{ij}\left(\overline{{\mathbf{b}}}_{i}^{g}\right)\end{aligned}\\
$$
则
$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_i\mathrm{Exp}\Big(\delta\vec{\phi}_i\Big)\right)& =\mathrm{Log}\left[\left(\Delta\mathbf{\hat{R}}_{ij}\right)^T\left(\mathbf{R}_i\mathrm{Exp}\left(\delta\vec{\phi}_i\right)\right)^T\mathbf{R}_j\right] \\
&\overset{1}{\operatorname*{=}}\mathrm{Log}\left[\left(\Delta\hat{\mathbf{R}}_{ij}\right)^T\mathrm{Exp}\left(-\delta\vec{\phi}_i\right)\mathbf{R}_i^T\mathbf{R}_j\right] \\
&\overset{2}{\operatorname*{=}}\mathrm{Log}\left[\left(\Delta\mathbf{\hat{R}}_{ij}\right)^T\mathbf{R}_i^T\mathbf{R}_j\mathrm{Exp}\left(-\mathbf{R}_j^T\mathbf{R}_i\delta\vec{\phi}_i\right)\right] \\
&=\mathrm{Log}\left\{\mathrm{Exp}\bigg[\mathrm{Log}\bigg(\bigg(\Delta\mathbf{\hat{R}}_{ij}\bigg)^T\mathbf{R}_i^T\mathbf{R}_j\bigg)\bigg]\cdotp\mathrm{Exp}\bigg(-\mathbf{R}_j^T\mathbf{R}_i\delta\vec{\phi}_i\bigg)\right\} \\
&=\mathrm{Log}\bigg[\mathrm{Exp}\bigg(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_i\right)\bigg)\cdotp\mathrm{Exp}\bigg(-\mathbf{R}_j^T\mathbf{R}_i\delta\vec{\phi}_i\bigg)\bigg] \\
&\overset{3}{\operatorname*{\approx}}\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_i\right)-\mathbf{J}_r^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_i\right)\right)\mathbf{R}_j^T\mathbf{R}_i\delta\vec{\phi}_i \\
&\overset{4}{=}\mathbf{r}_{\Delta\mathbf{R}_{ij}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\mathbf{R}_{j}^{T}\mathbf{R}_{i}\delta\vec{\phi}_{i}
\end{aligned}
$$
于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\vec{\phi}_i}=-\mathbf{J}_r^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\mathbf{R}_j^T\mathbf{R}_i
$$

##### :two:  $\delta\vec{\phi}_j$的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_{j}\mathrm{Exp}\Big(\delta\vec{\phi}_{j}\Big)\right)& =\mathrm{Log}\bigg[\bigg(\Delta\mathbf{\hat{R}}_{ij}\bigg)^T\mathbf{R}_i^T\mathbf{R}_j\mathrm{Exp}\bigg(\delta\vec{\phi}_j\bigg)\bigg] \\
&=\mathrm{Log}\left\{\mathrm{Exp}\bigg[\mathrm{Log}\bigg(\bigg(\Delta\hat{\mathbf{R}}_{ij}\bigg)^T\mathbf{R}_i^T\mathbf{R}_j\bigg)\bigg]\cdotp\mathrm{Exp}\bigg(\delta\vec{\phi}_j\bigg)\right\} \\
&=\mathrm{Log}\left\{\mathrm{Exp}\Big(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_j\right)\Big)\cdotp\mathrm{Exp}\Big(\delta\vec{\phi}_j\Big)\right\} \\
&\overset{1}{\operatorname*{\approx}}\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_j\right)+\mathbf{J}_r^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\mathbf{R}_j\right)\right)\delta\vec{\phi}_j \\
&\overset{2}{\operatorname*{=}}\mathbf{r}_{\Delta\mathbf{R}_{ij}}+\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\delta\vec{\phi}_{j}
\end{aligned}
$$

##### :three:$\widetilde{\delta\mathbf{b}_i^\mathrm{g}}$的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{R}_{ij}}\left(\delta\mathbf{b}_{i}^{g}+\widetilde{\delta\mathbf{b}_{i}^{g}}\right)& =\mathrm{Log}\left\{\left[\Delta\tilde{\mathbf{R}}_{ij}\left(\overline{\mathbf{b}}_{i}^{g}\right)\mathrm{Exp}\left(\frac{\partial\Delta\overline{\mathbf{R}}_{ij}}{\partial\overline{\mathbf{b}}^{g}}\left(\delta\mathbf{b}_{i}^{g}+\widetilde{\delta\mathbf{b}_{i}^{g}}\right)\right)\right]^{T}\mathbf{R}_{i}^{T}\mathbf{R}_{j}\right\} 
\\

&\overset{6}{\operatorname*{=}}\mathbf{r}_{\Delta\mathbf{R}_{ij}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\cdotp\mathrm{Exp}\left(-\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\cdotp\mathbf{J}_{r}\left(\frac{\partial\Delta\mathbf{\bar{R}}_{ij}}{\partial\mathbf{\bar{b}}^{g}}\delta\mathbf{b}_{i}^{g}\right)\cdotp\frac{\partial\Delta\mathbf{\bar{R}}_{ij}}{\partial\mathbf{\bar{b}}^{g}}\cdotp\widetilde{\delta\mathbf{b}_{i}^{g}}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\widetilde{\delta\mathbf{b}_{i}^{\mathrm{g}}}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{R}_{ij}}}{\partial\delta\mathbf{b}_{i}^{\mathrm{g}}}=-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\cdot\mathrm{Exp}\left(-\mathbf{r}_{\Delta\mathbf{R}_{ij}}\right)\cdot\mathbf{J}_{r}\left(\frac{\partial\Delta\mathbf{\overline{R}}_{ij}}{\partial\mathbf{\overline{b}}^{g}}\partial\mathbf{b}_{i}^{g}\right)\cdot\frac{\partial\Delta\mathbf{\overline{R}}_{ij}}{\partial\mathbf{\overline{b}}^{g}}
$$

### 3.7.2 $\mathbf{r}_{\Delta\mathbf{v}_{ij}}$ 的雅可比相关

$$
\begin{aligned}\mathbf{r}_{\Delta\mathbf{v}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\left[\Delta\tilde{\mathbf{v}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{v}_{ij}-\Delta\mathbf{\hat{v}}_{ij}\end{aligned}
$$



#### 3.7.2.1 0类

$\mathbf{r}_{\Delta\mathbf{v}_{ij}}$ 中不含 $\mathbf{p}_i\text{、}\mathbf{p}_j\text{、}\mathbf{v}_i\text{、}\mathbf{v}_j$ 因此$\mathbf{r}_{\Delta\mathbf{v}_{ij}}$关于这些状态增量的 Jacobian 都为0零矩阵。
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\vec{\phi}_j}=\mathbf{0} , \frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{p}_i}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\mathbf{p}_i}=\mathbf{0} , \frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{p}_j}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\mathbf{p}_j}=\mathbf{0}
$$

#### 3.7.2.2 线性类

$\mathbf{r}_{\Delta\mathbf{v}_{ij}}$ 关于 $\delta\mathbf{b}_i^g$ 和$\delta\mathbf{b}_i^a$ 是线性的,因此 $\mathbf{r}_{\Delta\mathbf{v}_{ij}}$ 关于 $\widetilde{\delta\mathbf{b}_i^g}\text{ 和 }\widetilde{\delta\mathbf{b}_i^a}$的 Jacobian 可直接由线性系数求得:
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\widetilde{\delta\mathbf{b}_i^g}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{b}_i^g}=-\frac{\partial\Delta\mathbf{\overline{v}}_{ij}}{\partial\mathbf{b}^g},\quad\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\widetilde{\delta\mathbf{b}_i^a}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{b}_i^a}=-\frac{\partial\Delta\mathbf{\overline{v}}_{ij}}{\partial\mathbf{b}^a}
$$


#### 3.7.2.3 复杂类

$$
\begin{aligned}\mathbf{r}_{\Delta\mathbf{v}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\left[\Delta\tilde{\mathbf{v}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{v}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{v}_{ij}-\Delta\mathbf{\hat{v}}_{ij}\end{aligned}
$$

##### :one:$\delta\mathbf{v}_i$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{v}_{ij}}\left(\mathbf{v}_{i}+\delta\mathbf{v}_{i}\right)& =\mathbf{R}_i^T\cdot\left(\mathbf{v}_j-\mathbf{v}_i-\delta\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\mathbf{\hat{v}}_{ij} \\
&=\mathbf{R}_i^T\cdot\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\mathbf{\hat{v}}_{ij}-\mathbf{R}_i^T\delta\mathbf{v}_i \\
&=\mathbf{r}_{\Delta\mathbf{v}_{ij}}\left(\mathbf{v}_{i}\right)-\mathbf{R}_{i}^{T}\delta\mathbf{v}_{i} \\
&=\mathbf{r}_{\Delta\mathbf{v}_{ij}}-\mathbf{R}_i^T\delta\mathbf{v}_i
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{v}_i}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\mathbf{v}_i}=-\mathbf{R}_i^T
$$


##### :two:$\delta\mathbf{v}_j$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{v}_{ij}}\left(\mathbf{v}_{j}+\delta\mathbf{v}_{j}\right)& =\mathbf{R}_i^T\cdot\left(\mathbf{v}_j+\delta\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\mathbf{\hat{v}}_{ij} \\
&=\mathbf{R}_i^T\cdot\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\mathbf{\hat{v}}_{ij}+\mathbf{R}_i^T\delta\mathbf{v}_j \\
&=\mathbf{r}_{\Delta\mathbf{v}_{ij}}\left(\mathbf{v}_j\right)+\mathbf{R}_i^T\delta\mathbf{v}_j \\
&\overset{1}{\operatorname*{=}}\mathbf{r}_{\Delta\mathbf{v}_{ij}}+\mathbf{R}_i^T\delta\mathbf{v}_j \\
\text{1}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\mathbf{v}_j}=\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\mathbf{v}_j}=\mathbf{R}_i^T
$$

##### :three:$\delta\mathbf{\phi}_i$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{v}_{ij}}\left(\mathbf{R}_{i}\mathrm{Exp}\Big(\delta\vec{\phi}_{i}\Big)\right)& =\left(\mathbf{R}_i\mathrm{Exp}\Big(\delta\vec{\phi}_i\Big)\right)^T\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\hat{\mathbf{v}}_{ij} \\
&\overset{1}{\operatorname*{=}}\mathrm{Exp}\left(-\delta\vec{\phi}_i\right)\cdotp\mathbf{R}_i^T\cdotp\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)-\Delta\hat{\mathbf{v}}_{ij} \\

&\overset{4}{\operatorname*{=}}\mathbf{r}_{\Delta\mathbf{v}_{ij}}+\left[\mathbf{R}_i^T\cdot\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)\right]^\wedge\cdot\delta\vec{\phi}_i
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{v}_{ij}}}{\partial\delta\vec{\phi}_i}=\left[\mathbf{R}_i^T\cdot\left(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\cdot\Delta t_{ij}\right)\right]^\wedge 
$$


### 3.7.3  $\mathbf{r}_{\Delta\mathbf{p}_{ij}}$ 的雅可比相关

$$
\begin{aligned}\mathbf{r}_{\Delta\mathbf{p}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\left[\Delta\tilde{\mathbf{p}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{p}_{ij}-\Delta\mathbf{\hat{p}}_{ij}\end{aligned}
$$

#### 3.7.3.1 0类

$\mathbf{r}_{\Delta\mathbf{p}_{ij}}$ 中不含 $\mathbf{R}_j\text{、}\mathbf{v}_j$ 因此$\mathbf{r}_{\Delta\mathbf{p}_{ij}}$关于这些状态增量(对于姿态来说是关于它的李代数扰动)的 Jacobian 都为0零矩阵。
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\vec{\phi}_j}=\mathbf{0} , \frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{v}_j}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\mathbf{v}_j}=\mathbf{0}
$$

#### 3.7.3.2 线性类

$\mathbf{r}_{\Delta\mathbf{p}_{ij}}$ 关于 $\delta\mathbf{b}_i^g$ 和$\delta\mathbf{b}_i^a$ 是线性的,因此 $\mathbf{r}_{\Delta\mathbf{p}_{ij}}$ 关于 $\widetilde{\delta\mathbf{b}_i^g}\text{ 和 }\widetilde{\delta\mathbf{b}_i^a}$的 Jacobian 可直接由线性系数求得:
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\widetilde{\delta\mathbf{b}_{i}^{g}}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{b}_{i}^{g}}=-\frac{\partial\Delta\mathbf{\overline{p}}_{ij}}{\partial\mathbf{b}^{g}} ,\quad\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\widetilde{\delta\mathbf{b}_{i}^{a}}}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{b}_{i}^{a}}=-\frac{\partial\Delta\mathbf{\overline{p}}_{ij}}{\partial\mathbf{b}^{a}}
$$


#### 3.7.3.3 复杂类

$$
\begin{aligned}\mathbf{r}_{\Delta\mathbf{p}_{ij}}&\triangleq\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\left[\Delta\tilde{\mathbf{p}}_{ij}\left(\overline{\mathbf{b}}_i^g,\overline{\mathbf{b}}_i^a\right)+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^g}\delta\mathbf{b}_i^g+\frac{\partial\Delta\overline{\mathbf{p}}_{ij}}{\partial\overline{\mathbf{b}}^a}\delta\mathbf{b}_i^a\right]\\&\triangleq\Delta\mathbf{p}_{ij}-\Delta\mathbf{\hat{p}}_{ij}\end{aligned}
$$



##### :one:$\delta\mathbf{p}_i$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{p}_{i}+\mathbf{R}_{i}\cdot\delta\mathbf{p}_{i}\right)& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{R}_{i}\cdot\delta\mathbf{p}_{i}-\mathbf{v}_{i}\cdot\Delta t_{ij}-\frac{1}{2}\mathbf{g}\cdot\Delta t_{ij}^{2}\right)-\Delta\hat{\mathbf{p}}_{ij} \\
&=\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\Delta\mathbf{\hat{p}}_{ij}-\mathbf{I}\cdot\delta\mathbf{p}_i \\
&=\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{p}_i\right)-\mathbf{I}\cdot\delta\mathbf{p}_i \\

&=\mathbf{r}_{\Delta\mathbf{p}_{ij}}-\mathbf{I}\cdot\delta\mathbf{p}_{i}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{p}_i}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\mathbf{p}_i}=-\mathbf{I}
$$

##### :two:$\delta\mathbf{p}_j$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{p}_{j}+\mathbf{R}_{j}\cdot\delta\mathbf{p}_{j}\right)& =\mathbf{R}_i^T\left(\mathbf{p}_j+\mathbf{R}_j\cdot\delta\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac{1}{2}\mathbf{g}\cdot\Delta t_{ij}^2\right)-\Delta\mathbf{\hat{p}}_{ij} \\
&=\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\Delta\mathbf{\hat{p}}_{ij}+\mathbf{R}_i^T\mathbf{R}_j\cdot\delta\mathbf{p}_j \\
&=\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{p}_j\right)+\mathbf{R}_i^T\mathbf{R}_j\cdot\delta\mathbf{p}_j \\
&=\mathbf{r}_{\Delta\mathbf{p}_{ii}}+\mathbf{R}_{i}^{T}\mathbf{R}_{j}\cdot\delta\mathbf{p}_{j}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{p}_j}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\mathbf{p}_j}=\mathbf{R}_i^T\mathbf{R}_j
$$

##### :three:$\delta\mathbf{v}_i$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{v}_{i}+\delta\mathbf{v}_{i}\right)& =\mathbf{R}_i^T\Bigg(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\delta\mathbf{v}_i\cdot\Delta t_{ij}-\frac{1}{2}\mathbf{g}\cdot\Delta t_{ij}^2\Bigg)-\Delta\mathbf{\hat{p}}_{ij} \\
&=\mathbf{R}_i^T\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)-\Delta\mathbf{\hat{p}}_{ij}-\mathbf{R}_i^T\Delta t_{ij}\cdot\delta\mathbf{v}_i \\
&=\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{v}_i\right)-\mathbf{R}_i^T\Delta t_{ij}\cdot\delta\mathbf{v}_i \\
&=\mathbf{r}_{\Delta\mathbf{p}_{ij}}-\mathbf{R}_{i}^{T}\Delta t_{ij}\cdot\delta\mathbf{v}_{i}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\mathbf{v}_i}=\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\mathbf{v}_i}=-\mathbf{R}_i^T\Delta t_{ij}
$$


##### :four:$\delta\mathbf{\phi}_i$ 的雅可比

$$
\begin{aligned}
\mathbf{r}_{\Delta\mathbf{p}_{ij}}\left(\mathbf{R}_i\mathrm{Exp}\big(\delta\vec{\phi}_i\big)\right)& =\left(\mathbf{R}_{i}\mathrm{Exp}\left(\delta\vec{\phi}_{i}\right)\right)^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i}\cdot\Delta t_{ij}-\frac{1}{2}\mathbf{g}\cdot\Delta t_{ij}^{2}\right)-\Delta\hat{\mathbf{p}}_{ij} \\
\\
&{=}\mathbf{r}_{\Delta\mathbf{p}_{ij}}+\left[\mathbf{R}_{i}^{T}\cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i}\cdot\Delta t_{ij}-\frac{1}{2}\mathbf{g}\cdot\Delta t_{ij}^{2}\right)\right]^{\wedge}\cdot\delta\vec{\phi}_{i}
\end{aligned}
$$

于是
$$
\frac{\partial\mathbf{r}_{\Delta\mathbf{p}_{ij}}}{\partial\delta\vec{\phi}_i}=\left[\mathbf{R}_i^T\cdot\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\cdot\Delta t_{ij}-\frac12\mathbf{g}\cdot\Delta t_{ij}^2\right)\right]^\wedge
$$
