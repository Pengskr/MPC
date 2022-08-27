<center>MPC模型预测控制</center>
=

[toc]

# [资料1. DR_CAN：MPC模型预测控制器](https://www.bilibili.com/read/cv16891782)


MPC_Test.m,Copy_of_MPC_Matrics.m,Copy_of_Prediction.m是DR_CAN提供的示例代码，例子中的参考信号R=0，且输出方程y=x,即矩阵C为单位阵。([代码地址](https://www.bilibili.com/read/cv16891782))

MPC_demo.mlx 以一个二阶系统为例演示MPC，对DR_CAN的代码进行了拓展，参考信号可设，输出方程中的c可设。([笔记推导](./MPC_notes.pdf))

在控制的教材中，常常考虑参考信号为0的简化情况。在参考信号不为0的情况下，可以通过引入误差$e=z-z_{d}$，将误差作为新的状态量，可以将问题重新转换为参考信号=0的情况(参考误差信号为0)，这在 资料3.无人驾驶车辆模型预测控制 的推导笔记中有所涉及，引入了误差，同时通过误差实现了非线性系统的线性化。

# [资料2. MATLAB中国【Model Predictive Control】](https://space.bilibili.com/1768836923/search/video?keyword=mpc)

## 1. 特点

[参考视频1](https://www.bilibili.com/video/BV16U4y1c7EG?spm_id_from=333.999.0.0&vd_source=be5bd51fafff7d21180e251563899e5e)
[参考视频2](https://www.bilibili.com/video/BV1Qu411Z7DQ/?spm_id_from=333.788.recommend_more_video.-1&vd_source=be5bd51fafff7d21180e251563899e5e)

### 1.1. 优点

1. 可以处理MIMO，而PID只能处理SISO，虽然可以使用多个PID控制多个变量，但当变量之间存在耦合时，PID参数的调节会很困难；
2. 可以处理约束条件，由于模型预测控制是通过构建优化问题来求解控制器的动作的，所以可以非常自然的将这些约束建立在优化问题中以此来保证这些约束的满足。；
3. 使用了未来的预测信息。

### 1.2. 缺点

要求强大的计算力，因为在每一个时间步都需要求解优化问题。

## 2. 参数设置

采样时间：设置为开环系统响应上升时间Tr的1/20~1/10
$$\frac{Tr}{20} \leqslant Ts \leqslant \frac{Tr}{10}$$
预测区间：20～30个时间步
控制区间：预测区间的10%～20%，并且至少有2～3个时间步
约束条件：约束分为硬约束和软约束，硬约束不可违背，软约束可以违背。不建议对输入和输出都进行硬约束，因为两者可能冲突以致无法求解优化问题。建议将输出设为软约束，并避免对输入和输入变化率都有硬约束
权重：取决于实际情况

[参考视频](https://www.bilibili.com/video/BV1b44y1v7Xt/?spm_id_from=333.788.recommend_more_video.-1&vd_source=be5bd51fafff7d21180e251563899e5e)

## 3. 自适应MPC，增益调度MPC，非线性MPC

适用于处理非线性系统，其中自适应MPC和增益调度MPC的本质是将系统线性化

[参考视频](https://www.bilibili.com/video/BV1ZL411g7Ya/?spm_id_from=333.788.recommend_more_video.-1&vd_source=be5bd51fafff7d21180e251563899e5e)

### 3.1. 自适应MPC(Adaptive MPC)

处理非线性系统时，在每个工作点附近对系统作线性化，得到一个新的线性模型，使用的前提是优化问题的结构在每个工作点不变，即在约束范围内，状态数量和约束数量不变。

### 3.2. 增益调度MPC(Gain-scheduled MPC)

优化问题的结构改变，可使用增益调度MPC。
离线线性化：在感兴趣工作点对系统进行线性化，并为每个感兴趣工作点设计一个线性MPC，每个线性MPC彼此独立，并且具有不同数量的状态和约束。
由于需要储存多个MPC控制器，因此消耗的内存比自适应MPC多。

### 3.3. 非线性MPC

无法很好的将非线性系统线性化的情况下使用非线性MPC。
控制会更加准确，求解会更加困难。

### 3.4. 总结

![总结](./imgs/1.png)

## 4. 加快MPC运行速度的方法

1. 模型降阶(Model Order Reduction)
2. 舍弃对系统动力学没有贡献的状态量
3. 缩短预测区间和控制区间
4. 减少约束的数量
5. 使用更低的数据精度
6. 使用显式MPC(Explicit MPC):通过离线预计算最优解，来大大减少运行时间
7. 使用次优解

[参考视频](https://www.bilibili.com/video/BV1Ar4y1y7Tp/?spm_id_from=333.788.recommend_more_video.-1&vd_source=be5bd51fafff7d21180e251563899e5e)

## 5. 示例

使用自动驾驶汽车示例来演示控制器的设计。

[参考视频1](https://www.bilibili.com/video/BV1xP4y1s7pJ?spm_id_from=333.999.0.0&vd_source=be5bd51fafff7d21180e251563899e5e)
[参考视频2](https://www.bilibili.com/video/BV1Wr4y1k76z?spm_id_from=333.999.0.0)


# [资料3. 无人驾驶车辆模型预测控制 (龚建伟)](https://www.bilibili.com/video/BV1HQ4y1P7bJ?p=1&vd_source=be5bd51fafff7d21180e251563899e5e)

MPC从输⼊ 参考信号以及测量/估计的系统状态量到输出 控制信号，中间的优化过程⽤到了被控系统的物理模型，所以更复杂，控制效果也更好，对于不同被控系统，都需要重新设计MPC。⽽PID属于通⽤控制器，使⽤过程中只需要考虑如何调整控制器参数，不需要考虑被控系统的物理模型(但还是需要结合模型考虑加⼊控制器后的系统稳定性)。
MPC适⽤于MIMO系统，⽽PID更适⽤于SISO系统； PID只考虑了输出的准度，⽽MPC不但可以考虑输出的准度，还考虑了输⼊。
([笔记推导](./MPC_notes.pdf))