# 变曲率贴边几何实验：运行前协议

2026-10-31，新增独立实验；不修改 NARROW_BENCHMARK_PROTOCOL.md、现有闭环或 latex_src。

## 范围与停止条件

本实验是静态几何可行域评估，不是闭环任务，不输出完成率、规划性能或计时排名。现有 SampledRoad/动力学支持位置曲率，但 run_paper_comparison 的建路、初态、评价器，Xing 的 project/connection/_extents，以及 exact road sensitivity 均硬编码圆弧。公平闭环需要统一可微参考线、可靠多点投影、Xing 连接雅可比/包络、初态和独立评价器，目前不以局部替换冒充完成。

## 冻结场景

bus/SUV 尺寸取现有 paper_comparison.vehicle 的 BUS/SUV；道路半宽 3 m，安全余量 0.2 m。s∈[0,60] m，间隔 5 m；航向误差 {-0.08,0,0.08} rad；横向位置 [-2.5,2.5] m，间隔 0.25 m，两侧等权。共 4914 个姿态。

k(s)=A+B tanh((s-30)/10)，进入弯道 A=B=0.015；退出 A=0.015,B=-0.015；S弯 A=0,B=0.03（1/m）。参考航向为 k 的解析积分，中心线对 cos(theta),sin(theta) 数值积分；范围 [-40,120] m，步长 0.02 m，禁止常曲率替代真实参考线。

共同初态 s=n=alpha=0，速度 5 m/s，a=0；初始车辆曲率等于 k(0)。检查全车身净空 >=0.2 m、横向加速度 <=5 m/s²、横向 jerk <=5 m/s³、转角及转角速度不越各车型上限。另检查 n=0、恒速 5 m/s 沿参考线的动力学量；这是见证轨迹的解析必要检查，不是优化器动态可行性证明或贴边轨迹完成证明。

## 方法与公平性

1. rear_axle_point：参考点，仅作原始弱基线。
2. fixed_worst_curvature：固定对称膨胀，在 |k|<=0.03、|alpha|<=0.08、|n|<=2.5 的圆弧族上预先校准全车身最坏包络。使用 k∈{-0.03,0,0.03}、alpha∈{-0.08,0,0.08}、n∈{-2.5,0,2.5}；这是离散校准的强静态基线，不宣称连续区间保守性定理。
3. local_curvature_adaptive：当前 k(s)、n、alpha 的圆弧精确矩形包络；使用全部矩形边采样，包括内侧边中点。它假设局部常曲率，独立真值可检出其误差。
4. xing_rectangle_generalized：借用 Xing 的全矩形包络思想，真实变曲率投影并逐边优化横向极值；是广义几何基线，不是现有 Xing 闭环算法原样运行。报告其与独立密集边采样的差异。
5. paper_heading_difference：n+lateral/cos(alpha)+integral tan(alpha-[theta(s+u)-theta(s)])du；ds=longitudinal*cos(alpha)-lateral*sin(alpha)，沿正负 ds 使用 5 段左矩形积分。**method change**：把原 approximate_road_offsets 中 k(s)*u 换为 theta(s+u)-theta(s)，不声称原公式已支持变曲率。100 段仅用于积分收敛诊断，不作为主方法。
6. paper_original_local：原 5 段常曲率近似，诊断项。

## 真值与指标

独立真值：四条刚性矩形边每边 257 点，Cartesian 点对真实参考线求正交投影，检查投影残差、道路域与 1-k*n 正则性；净空为 half_width-max|projected n|。这是高精度数值真值，非解析精确证明。Xing 广义基线使用每边 17 点定位并对每个候选极值区间做有界一维优化。比较 min/max 横向范围；报告最大包络误差、false-safe（方法判安全但真值不安全）、false-reject、真值可行数及方法可行数。首轮容差 1e-6 m；审阅发现 257 点边采样误差最大约 2.53e-6 m，运行后修订：最终采用 1e-5 m 数值误差预算，并逐姿态比较优化边界与采样真值，若差异超过预算则标为不确定（不将这种差异计为方法胜负）。另用独立最近距离优化验证投影，不仅使用共享投影往返。该修订不改任何方法公式。落在判界误差预算内的姿态另计不确定数，不凭不确定数作优劣结论。保留逐姿态 CSV、summary JSON、协议及脚本 SHA256。

## 资源和解释限制

串行计时后台任务仍运行时只运行单线程轻量几何评估，不编译或调用优化器，不报告本脚本耗时。任何方法优势必须同时对比固定膨胀、局部自适应与矩形强基线；若本文不占优如实报告。没有障碍物、控制反馈、warm start、求解成功率、目标到达或贴边动态轨迹证据，不能用此实验替代新增闭环完成率。后续闭环几何消融统一使用新增 `run_variable_curvature_closed_loop.py` 的 SLSQP direct-shooting：同一 RK4 Frenet 动力学、同一 horizon/dt/maxiter、逐周期 warm start；方法仅替换道路包络，Xing 项明确是 generalized rectangle geometry ablation，不是原 Xing solver。正式运行必须覆盖完整 0--60 m 进入/退出/S 弯、bus/SUV、左右两侧，逐节点使用独立 1025 点全边采样与逐边极值检查；记录失败、迭代预算、停止原因、完整轨迹，不预设优胜。当前仅完成 N=2、2 steps smoke，不是正式结果；串行 timing job 结束前禁止密集 solver。
