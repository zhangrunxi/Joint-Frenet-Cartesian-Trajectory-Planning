# Major Revision 实验验收报告

> 历史快照：本报告保留 2026-10-29 的实验与限制。2026-10-30 用户提供
> Xing 全文后，已新增原文方程重建并完成三方对比；最新结论和数值见
> [FINAL_COMPARISON_REPORT.md](FINAL_COMPARISON_REPORT.md)。
> 以下关于 Xing 全文缺失、只有代理的判断描述的是此前状态。

日期：2026-10-29。工作环境：当前 Mac，ARM64，Python 3.11.15，CasADi 3.8.1，acados + HPIPM/BLASFEO GENERIC，单线程。没有修改论文 PDF，也没有把业务仓库复制到本机。

## 1. 验收结论

**本文方法与 Reiter [20] 的独立重建比较已运行；严格的三篇论文方法比较尚未完成。** Xing [26] 的全文和作者实现仍未取得。目前第三类代码是明确标注的 Xing-inspired 代理，不能用来完成评审要求的 Xing 原方法基线验证。

已完成的主要证据：

1. 相同公交车、场景、时域和独立评估器下，8 个正式配置 × 4 个场景完成 3,840 个闭环周期，无采样碰撞或越界。
2. 窄路场景中，混合几何版本完成超越，圆形近似版本跟车；同一六状态模型的几何消融也呈现这一差异，支持研究碰撞表示的保守性，而不只是比较不同求解器。
3. 同物理模型、同成本、同约束的 6/9 状态消融，以及 N=20/40/80 的固定状态回放，实际测得减少三个冗余坐标的开销收益。
4. 独立 AL-CiLQR 与 SQP 得到接近的轨迹，但 Python CiLQR 尾延迟较大，不能据此声称实时性能优于 Reiter。
5. Xing-inspired 代理在宽路完成运行，窄路因收敛条件停止。它不是 Xing 原方法，失败不能归因于 Xing 论文。

本报告的秒数、净空和进度来自存储结果；不是论文原表数值，不是原作者车载性能。只有确定性的少量合成场景，没有统计显著性或一般优越性结论。

## 2. 算法与配置数量

| 类别 | 配置 | 用途 |
| --- | --- | --- |
| Reiter 重建 | Direct 5 状态 / Lifted 8 状态，各配 ellipse / 3-circle，共 4 个 | 论文 [20] 方程的适配比较 |
| 本文重建 | 6-state hybrid SQP | 共同求解器下的方法比较 |
| 本文消融 | 9-state hybrid SQP | 只增加 Cartesian 三状态的维数消融，不是 Reiter 算法 |
| 本文消融 | 6-state circles SQP | 只替换碰撞表示的几何消融 |
| 本文重建 | 6-state hybrid AL-CiLQR | 独立实现论文类求解器，不是工程部署二进制 |
| Xing-inspired，未核验 | algebraic joint-frame QP，circle / ellipse，共 2 个 | 相关架构的接口与可行性探索，不能代替 Xing [26] 基线 |

因此是 **3 类架构、10 个配置，其中 8 个正式比较配置＋2 个探索性代理配置**，不是 10 篇论文、10 个独立算法，也不是三个均已忠实复现的论文基线。

## 3. 来源及实现映射

### 本文方法

- 论文：本目录上两级的 `Joint_Frenet_Cartesian_Trajectory_Planning.pdf`，每次运行记录 SHA-256。
- 工程参考：用户给出的 `igoliao/planning` 的 `2.1` 分支，提交 `bee02d33c0cb50a5605db98b3cac44b816cdfecf`。仅在开发机上以浅克隆、no-checkout 方式读取，没有直接运行或整体移植。
- 核查文件：`trajectory_gen/algorithm/cilqr_trajectory/vehicle_model.{cc,h}`、`cilqr_traj_data.{cc,h}`、`cilqr_traj_cost.{cc,h}`、`cilqr_traj_solver.cc`、`cilqr_traj_problem.cc` 和 `proto/cilqr_traj_optimizer.proto`。
- 状态 `[s,n,alpha,v,a,kappa]`，输入 `[jerk,kappa_dot]`；Frenet 动力学、Cartesian 碰撞几何；道路使用四角的离散近似投影。
- `paper_comparison/model.py` 实现模型、混合几何和 SQP；`paper_comparison/cilqr.py` 实现 AL/PHR 罚项、Riccati 反向递推、正则化和回溯。

工程参考采用 Euler、圆形碰撞和软屏障类 CiLQR，与论文写法并不完全一致。因此这里是“论文公式为主、工程代码作参考的独立重实现”，不能称为生产算法直接移植。

### Reiter [20]

- [作者稿](https://publications.syscop.de/Reiter2023a.pdf)，[obstacle_avoidance](https://github.com/RudolfReiter/obstacle_avoidance)，[vehicle_gym](https://github.com/RudolfReiter/vehicle_gym)。
- 固定提交：obstacle_avoidance `647f3582dad37d7e52bb7e7e560e7b084b0ddfb9`；vehicle_gym `8f2aa31f4d45660813c10fb276eb4b082b145afe`；acados `a58e76ad328d97aea180b55344e80eae346f8d15`。
- 当前公开依赖缺少原 `VehiclePlannerAcados20222`。`reiter2023/trajectory_planner_acados_20222.py` 是独立方程重建，旧的原车参数试验保留于 `results/reconstructed_sqp`，没有混入公交车比较。
- 公交车比较在 `scripts/run_paper_comparison.py` 的 `BusReiter` 中适配，未修改旧重建模块。保留 force/steering-rate、Direct/Lifted、ellipse/circles 结构。

### Xing [26]

[论文 DOI](https://doi.org/10.1109/LRA.2022.3194330)。目前的论文全文访问、开放元数据检索、相关专利及代理差异，详见 `sources/XING_ACCESS_AND_FIDELITY.md`。未发现可确认属于该论文的公开仓库；检索到的同名 Cartesian/Frenet 项目不能因此认作原实现。

## 4. 共同条件与不能隐藏的适配

共同条件：N=40、dt=0.1 s、4 s 预测时域、12 s 闭环；自车初速 12 m/s，目标 15 m/s，障碍物后轴 `s=35+8t,n=0`。直路 kappa=0；弯道 kappa=1/60 m^-1。宽路半宽 8.5 m、横向目标 5.5 m；窄路半宽 4.6 m、横向目标 2.6 m。横向绕行方向和目标由实验提供，没有复现完整行为决策层。

车辆长 6.50 m、宽 2.30 m、轴距 4.32 m、前悬 1.18 m、后悬 1.00 m。几何中心位于后轴前方 2.25 m；修正 vehicle_gym 对称车身中心假设的 0.09 m 偏移。加速度 [-3,2.5] m/s²，车速 [0,20] m/s，转角 29.32°、转角速率 24.55°/s；优化器的道路/碰撞裕量参数均为 0.2 m，**道路约束使用近似投影，不保证实际全车体净空达到 0.2 m**。

以下差异需在论文或补充材料披露：

- Reiter 为适配公交车关闭风、滚阻、风阻，令 a=F/m；质量沿用库的 1160 kg 作为 force 参数缩放，**不是声称公交车质量为 1160 kg**。其 force 代价会随此缩放改变，必须重新调参才能比较真实动力经济性。
- Reiter 原来较简化的道路约束统一增强为四角道路近似；其终端仍保留原动力学相关约束，未新增终端障碍物/轮廓约束。本文在终端也检查碰撞和道路，并令终端输入为零。
- 本文有 jerk、lateral jerk 硬界；Reiter 没有加速度状态，未给其静默添加同等 jerk 状态。不同模型的舒适性差异不能全部归因于 frame 表示。
- 本文代价保留曲率、曲率变化率、加速度、jerk、横向加速度/jerk，同时参考工程代码添加进度、横向位置、航向和速度跟踪。本文权重存于每个 case 的 `settings`，**不是原论文部署调参**。旧 Reiter case 的 `settings` 仅是共同实验设置，里面的本文权重/jerk 界并不生效；实际方法配置补充记录在 `results/effective_method_configurations.json`，未改写原记录。
- 两类模型都使用 RK4；工程参考原本为 Euler。本文终端用软跟踪，不是未给出具体目标时刻的全状态等式。
- 论文 Eq.34e 的印刷形式存在量纲问题；这里使用 `delta_dot=L*kappa_dot/(1+(L*kappa)^2)` 的物理转角速率界。
- 混合几何为中心胶囊＋四角圆，所有 25 个部件对均检查；未实现说明不充分的部件筛选。因此不声称复现了碰撞对裁剪加速。
- 接受条件为实现返回 status=0 且模型最大残差 <=1e-4。CiLQR 的 status=0 **只表示可行性通过，不等价于 SQP 的收敛状态或相同最优性容差**；其内层可能因预算或线搜索停止，未保存足够信息来独立认证驻点。以下称其为 feasibility-accepted 轨迹。失败预测保存在 `*_rejected.json`，不会静默重试后只留下成功解。

## 5. 主要数值结果

以下原生时间为去掉首次求解后的 acados `time_tot` 平均值。它是共同编译后端下较可比的数值，**不是完整端到端时间**。Reiter runner 的 wall time 还包含较多 Python 诊断操作；不能将其与本文只覆盖参数设置/solve 的 wall time 直接作加速比。

### 窄路：半宽 4.6 m，目标横向 2.6 m

所有表中配置均完成 120 步且未报告采样碰撞/越界。进度是最终 Frenet s；障碍物最终 s=131 m。“超越”使用自车后轴超过障碍物后轴 6.5 m 的单独判据。

| 配置 | 直路进度 m | 弯道进度 m | 直/弯超越 | 直/弯原生均值 ms |
| --- | ---: | ---: | --- | --- |
| Reiter Direct / ellipse | 121.377 | 120.817 | 否 / 否 | 5.213 / 7.355 |
| Reiter Lifted / ellipse | 121.377 | 120.817 | 否 / 否 | 7.793 / 12.306 |
| Reiter Direct / circles | 122.489 | 121.710 | 否 / 否 | 9.720 / 14.397 |
| Reiter Lifted / circles | 122.489 | 121.710 | 否 / 否 | 13.057 / 16.649 |
| 本文 6 / hybrid SQP | 177.342 | 183.820 | 是 / 是 | 3.124 / 4.902 |
| 本文 9 / hybrid SQP | 177.342 | 183.820 | 是 / 是 | 3.790 / 5.862 |
| 本文 6 / circles SQP | 122.093 | 121.308 | 否 / 否 | 8.678 / 14.352 |

本文 6/hybrid 相比 Reiter Lifted/ellipse，在此算例最终 s 分别增加 55.965 m 和 63.004 m；相比自身 circles 消融增加约 55.25 m 和 62.51 m。**这是本初始化下局部优化轨迹的差异，不证明其他表示全局无法超越。** 安全跟车不应标为求解失败。

本文 6/hybrid 的实际矩形最小障碍物净空为 0.625 m / 0.616 m，最小道路净空为 0.430 m / 0.258 m；不是仅看优化器自身的胶囊距离。弯道中接近道路边界，后续应加入地图/定位误差裕量，不能把确定性净空直接解释为真实驾驶安全保证。

### 宽路：半宽 8.5 m，目标横向 5.5 m

8 个配置均完成超越。Reiter Lifted/ellipse 最终 s=176.594/190.591 m，本文 6/hybrid 为 177.127/191.096 m，轨迹接近；进度仅约增加 0.53/0.50 m，不能写成显著规划质量提升。

Reiter Direct/ellipse 的原生均值 0.754/0.864 ms，本文 6/hybrid 为 1.839/2.154 ms；这里本文完整混合几何 **更慢**。本文 circles 消融为 1.351/1.478 ms，说明全 25 对 hybrid 检查有额外开销。不能以窄路结果推导“所有场景本文更快”。

### CiLQR：保留不利结果

| 场景 | 平均 wall ms | P95 wall ms | 超过 100 ms 的暖启动比例 |
| --- | ---: | ---: | ---: |
| 宽路直线 | 200.143 | 1614.05 | 15.97% |
| 宽路弯道 | 196.695 | 1635.36 | 15.97% |
| 窄路直线 | 253.783 | 1636.37 | 36.97% |
| 窄路弯道 | 277.063 | 1813.34 | 34.45% |

CiLQR 四个场景均完成 120 步、无采样碰撞/越界，最大接受残差 9.901e-5；窄路最终 s=177.340/183.846 m，接近 SQP。Jacobian/代价部分通过 CasADi JIT 编译，但 Riccati 及线搜索是 Python/NumPy；没有对应的“原生 solver-only 时间”。**现有实现未满足每周期 100 ms 的实时性要求，也不是原车载 CiLQR 性能基准。**

### Xing-inspired 代理：只报告探索性结果

宽路两种几何、两种道路均完成 120 步，进度约等于 Reiter；circle wall 均值 54.10/57.26 ms，ellipse 51.47/55.80 ms。窄路 circle 在直/弯完成 8/6 步后停止，ellipse 完成 10/6 步后停止；停止前没有采样碰撞，停止原因见逐周期 `qp_status` 和 rejected 文件。

这是代理的数值和收敛限制，不是 Xing 原方法的基准结果；因此主结论中不使用这些时间或失败来声称超越 Xing。

## 6. 维数是否真的测量

N=40、2 维控制时，直接多重打靶的未消元变量数 `nx*(N+1)+nu*N` 为：

| 状态表示 | 状态维数 | 未消元变量数 |
| --- | ---: | ---: |
| Reiter Direct | 5 | 285 |
| 本文 direct | 6 | 326 |
| Reiter Lifted | 8 | 408 |
| 本文 lifted 消融 | 9 | 449 |

本文 6 vs 自身 9 消融减少 123 个变量，即 27.39%；相对 Reiter Lifted 减少 20.10%，但比 Reiter Direct **多 14.39%**。这不是 HPIPM condensing 后的 QP 维数，也不是内存测量。

`scripts/run_dimension_scaling.sh` 在四个共同场景的相同物理状态序列上回放：初态＋随后 20 个状态，3 次重复，N=20/40/80，每个时域每个表示 240 次 warm solve，另列 cold。不是重新随机采样场景，重复计时不提供轨迹成功率置信区间。

| N | 6-state 暖启动原生均值 ms | 9-state 暖启动原生均值 ms | 6-state 耗时减少 | 6/9 各自接受 warm 次数 |
| --- | ---: | ---: | ---: | --- |
| 20 | 2.459 | 2.922 | 18.83% | 240/240 |
| 40 | 8.594 | 10.295 | 19.79% | 240/240 |
| 80 | 29.309 | 35.065 | 19.64% | 237/240 |

均值包含失败调用耗时，四场景等权汇总。总计 1,512 次 solve，1,482 次接受：N=80 的 24 次 cold 全部失败，另外窄路弯道的 6 次 warm 未通过状态码。N=80 每个表示的 warm 阶段还包含 15 次失败后的零控制 rollout 重新初始化，不全是平移上一解的暖启动；恢复策略在 provenance 中列出。不能省略这些失败后宣称全范围稳定。741 对双方均接受的比较中，最大物理状态差 9.48e-7，最大首个 jerk 差 2.91e-6，首个 curvature-rate 差 7.42e-10，支持该消融基本保持同一物理解。

有证据支持“避免冗余 Cartesian 状态的可量化开销”，没有证据支持“维数爆炸”。增加三状态对应随 N 线性增加 `3*(N+1)` 个变量；应删除或收窄 dimensionality explosion 的措辞。

## 7. 道路宽度敏感性

增加半宽 4.0、4.3、4.6、4.9、5.2、5.5 m，横向参考同步取 `half_width-2 m`，每个宽度包含两种道路和三个配置。它是“道路宽度＋参考位置”的联合敏感性试验，不是只改变几何可行域、保持目标完全不变的单因素试验。

36 个 case 实际完成 4,105 个周期，34 个 case 完成 120 步，2 个因收敛条件停止，无已执行区间的采样碰撞/越界。不要把不完整运行的进度、净空与完整 12 s 运行直接作大小比较。

| 半宽 m | 本文 hybrid：直 / 弯 | 本文 circles：直 / 弯 | Reiter Lifted circles：直 / 弯 |
| --- | --- | --- | --- |
| 4.0 | 跟车 / 跟车 | 跟车 / 跟车 | 跟车 / 跟车 |
| 4.3 | 超越 / 9 步后中止 | 跟车 / 跟车 | 跟车 / 跟车 |
| 4.6 | 超越 / 超越 | 跟车 / 跟车 | 跟车 / 跟车 |
| 4.9 | 超越 / 超越 | 超越 / 超越 | 超越 / 16 步后中止 |
| 5.2 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 |
| 5.5 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 |

半宽 4.3 m 弯道中，本文 hybrid 的失败状态码为 4，最大约束残差约 0.224；半宽 4.9 m 弯道中，Reiter 达到 150 次 SQP 迭代，状态码 2，虽然可行性残差很小，仍按既定接受条件停止。失败预测保留，不追加特别初始化来单独救某一方法。

独立审计还发现：半宽 4.9 m 的本文 circles 弯道运行，实际最小道路净空 **0.194173 m**，低于优化器设定的 0.2 m，尽管模型残差仅约 1.47e-8。这是道路投影近似的误差，不是求解器容差保证失效；“没有采样越界”仍成立，但“保证 0.2 m 实际道路裕量”不成立。若用于安全裕量结论，须改用精确道路投影或经过验证的误差缓冲，并重新运行。本轮保留该结果，不悄悄修改模型掩盖短缺。

扫描说明：混合几何的收益集中在部分紧空间范围；路更宽时各方法均可超越，更窄时混合几何也会跟车或不收敛。不能据此声称混合几何始终成功，或精确定位某种表示的全局可行性阈值。图和原始标量见 `results/width_sweep/plots/width_sweep.png`、`width_sweep.csv`。

### 与优化器无关的几何保守性测量

为避免把所有跟车差异都归因于几何，另做不运行规划器的几何审计：两辆同向、长 6.5 m、宽 2.3 m 的矩形，在相对后轴位置 `ds∈[-12,12] m,dn∈[-7,7] m` 上按 0.05 m 采样，共 135,161 点。真实矩形净空的解析式为 `hypot(max(abs(ds)-6.5,0),max(abs(dn)-2.3,0))`，以 >=0.2 m 为安全参照。

| 碰撞表示 | 安全网格点被保守拒绝比例 | 同纵向位置的最小侧向分离 m |
| --- | ---: | ---: |
| 真实矩形（解析参照） | 不适用 | 2.500 |
| 本文 hybrid | 4.52% | 2.976 |
| 3-circle | 10.48% | 3.453 |
| Reiter ellipse surrogate | 25.92% | 5.274 |

三种近似在该网格均无 unsafe-accepted 点；hybrid 相比 circles 的最小同纵向侧向分离降低约 0.476 m。这提供了不依赖求解器的“较少保守性”测量，但 hybrid 本身仍比精确矩形保守，不能称作精确矩形碰撞检测。比例是该有限网格内的面积抽样比例，**不是驾驶成功概率**；仅覆盖同向相对姿态，不能推广到所有角度的连续保证。

脚本 `scripts/measure_geometry_conservatism.py`；结果、完整网格和图分别为 `results/geometry_conservatism/summary.csv`、`grid.npz`、`geometry_conservatism.png`。

### Reiter 输入代价敏感性

审计发现 Reiter 原 `0.001*F²` 在本次质量缩放下等效为 `1345.6*a²`，本文加速度项为 `1*a²`。因此另跑 16 个 case，只把 Reiter force 权重改为 `1/mass²`，使其加速度平方项为 1，其他成本和约束不变；这是单项权重敏感性，不是完全匹配了 jerk 状态、全部成本或终端约束。

实际执行 1,795 个周期，14/16 个 case 完成：宽路 8 个均完成超越，最终 s=178.108/191.990 m，略大于本文 177.127/191.096 m。**宽路的微小进度差方向会随权重改变，更不能作为方法一般优越性的依据。**

窄路 circles 的 Direct/Lifted 四个 case 均完成但仍跟车，最终 s=121.954/121.156 m；ellipse 的 Lifted/直路和 Direct/弯道完成但跟车，Direct/直路与 Lifted/弯道分别在 57/58 步后中止。说明当前窄路的 hybrid/circles 超越差异并非只由这个加速度罚项造成，但仍不能排除其他权重和局部初始化影响，也不能据此证明全局可行性边界。

脚本 `scripts/run_reiter_cost_sensitivity.py`；原始结果 `results/reiter_acceleration_cost_sensitivity/summary.csv`，完整日志 `logs/reiter_cost_sensitivity.log`。原始正式结果未覆盖。执行时源码快照 `sources/run_reiter_cost_sensitivity_executed.py` 与 provenance 匹配，随后只作了 import 格式整理。

### 精确道路约束敏感性（不是原论文近似约束）

针对审计发现的误差，另实现 `scripts/run_exact_road_sensitivity.py`：只替换本文道路约束，直路用真实角点上下界；圆弧用整个矩形到圆心的最小边段距离和最大角点距离，处理圆心落在矩形内部的情况。其余动力学、成本、碰撞几何、初始化不变。三个曲率下共 300 个随机状态与独立道路评估器核对至 1e-10。

在半宽 4.3/4.6/4.9 m、两种道路、hybrid/circles 两种配置下运行 12 个 case，实际执行 1,219 个周期，10 个完成。4.3 m 的 hybrid 弯道 5 步后中止，4.6 m 的 circles 弯道 14 步后中止，其余完成。4.6 m hybrid 直/弯仍完成超越，s=177.342/183.820 m；4.9 m circles 弯道完成超越，最小物理道路净空从原近似约束的 0.194173 m 改为 **0.200000029 m**。

精确的离散节点约束仍不是连续时间裕量保证：4.3 m hybrid 直路的子步采样最小净空为 0.199988772 m，约比 0.2 m 少 11 微米；这与节点间传播/数值误差有关。若要严格连续时间保障，还需收紧裕量或连续碰撞/边界验证。该敏感性版本也有额外的非光滑约束和收敛失败，不能把它默认为已修复、已认证的原论文实现。

完整结果和图：`results/exact_road_sensitivity/summary.csv`、`results/exact_road_sensitivity/plots/trajectories.png`；原始正式与宽度扫描结果均未覆盖。

## 8. 评估、文件与复跑

- 独立评估器 `paper_comparison/evaluation.py` 使用真实矩形，障碍物用 SAT/边段距离，道路用直线或圆环全车体边缘净空；包括圆弧道路边内部点，不只四角。
- 每 0.02 s 检查实际闭环传播；这不是连续时间无碰撞证明。控制周期记录的舒适性 RMS 也不是 0.02 s 连续积分指标。
- 原始正式结果：`results/paper_comparison_wide_final`、`results/paper_comparison_narrow_final`；case JSON 保存已执行闭环状态/控制和每周期诊断，不包含每次接受解的完整预测时域；失败时另存 rejected 预测。摘要为 `summary.csv` / `summary.json`。
- 维数结果：`results/dimension_scaling/measurements.csv`、`paired_controls.csv`、`summary.csv`，失败调用未删除。
- 维数图：`results/dimension_scaling/dimension_scaling.png`，包括冷/暖阶段的接受计数；绘图脚本 `scripts/plot_dimension_scaling.py` 只读取结果。
- 探索性代理：`results/xing_inspired_wide_v2`、`results/xing_inspired_narrow_v2`；v1 失败记录原样保留。
- 图：`results/paper_comparison_plots` 为 8 配置；`results/three_family_comparison_plots` 合并 10 配置，并把 proxy 未核验和中止端点标出；各有轨迹、Cartesian 轨迹、性能图和全精度 `comparison.csv`。
- `results/comparison_source_audit.json` 验证正式运行源码哈希。新增代理前后的 runner 快照分别保存在 `sources/run_paper_comparison_pre_proxy.py` 和 `sources/run_paper_comparison_with_proxy_v2.py`，与原 provenance 匹配；没有改写旧 provenance。
- 当前依赖锁定 `requirements-comparison.lock`；其中本地 editable 包需按下面命令单独安装。构建前先阅读 `RECONSTRUCTION.md`，不要覆盖已有结果目录。

```bash
cd experiments/reiter20
uv pip install --python .venv/bin/python -r requirements-comparison.lock
uv pip install --python .venv/bin/python --no-deps -e vendor/vehicle_gym
uv pip install --python .venv/bin/python --no-deps -e vendor/acados/interfaces/acados_template

# 默认仅运行 8 个正式配置；新目录避免覆盖。
bash scripts/run_paper_comparison.sh --steps 120 --output results/new_wide
bash scripts/run_paper_comparison.sh --steps 120 --half-width 4.6 --lateral 2.6 --output results/new_narrow

# 探索性 proxy 必须显式选择，不混入默认正式配置。
bash scripts/run_paper_comparison.sh --steps 120 \
  --methods xing_inspired_proxy_circles xing_inspired_proxy_ellipse \
  --output results/new_proxy_wide

bash scripts/run_dimension_scaling.sh --output results/new_dimension_scaling
bash scripts/run_width_sweep.sh results/new_width_sweep
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/run_reiter_cost_sensitivity.py --output results/new_cost_sensitivity
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/run_exact_road_sensitivity.py --output results/new_exact_road
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" \
  .venv/bin/python scripts/measure_geometry_conservatism.py --output results/new_geometry_grid

.venv/bin/python scripts/plot_paper_comparison.py \
  --inputs results/paper_comparison_wide_final results/paper_comparison_narrow_final \
  results/xing_inspired_wide_v2 results/xing_inspired_narrow_v2 \
  --merge-by-width --cartesian --output results/new_plots
```

### 验证状态

最终运行 `REITER_RUN_INTEGRATION=1 ... python -m pytest -q tests`：**172 项通过**，无跳过；完整日志 `logs/comparison_final_tests_with_exact_road.log`。包含 acados 编译集成、独立矩形评估、混合几何覆盖、模型导数、物理转角速率、代数 frame 连接、代理 QP、迭代失败不接受、道路近似裕量短缺回归，以及精确道路敏感性模型的独立核对。CasADi/NumPy 兼容与 acados 弃用警告保留在日志；它们不等同于数值验证失败。之前的日志分别为 168 和 169 项通过，均保留。

归档校验：`.venv/bin/python scripts/verify_comparison_results.py` 不启动求解器，核对各 summary 与逐 case 数据、接受/拒绝条件、轨迹长度及运行源码哈希，覆盖 **104 个 case、11,469 个已执行闭环周期和 1,512 个维数回放调用**；输出 `results/comparison_archive_manifest.json`。104 个 case 包含代理、宽度扫描、输入代价及精确道路敏感性，不能全部称为正式论文基线。早期 pilot 不计入这些总数。独立只读审计记录见 `AUDIT_FINDINGS.md`，审计后的披露修正并不消除模型限制。

## 9. 可用于回复评审的结论边界

可写：新增 Reiter 方程重建基线；公平公交车适配；混合几何和状态维数两项分离消融；明确计数和实测时域扩展；报告复杂几何的额外开销和初始化失败；承认 joint-frame 思路不是本文首次提出。

不可写：已经完成原 Xing 基线；所有场景显著快于 prior art；Python CiLQR 达到原车载实时性能；Reiter 必然需要更多状态；窄路跟车等价于不安全；仅靠单组确定性结果证明一般优越性。

**仍待取得 Xing 全文并完成方程级核验，才可把三方比较作为完整回应提交。** 本次没有为凑三方结果而把另一个算法冒充 Xing，也没有修改论文贡献措辞；这些需结合核验后的结果再修订。

补充来源核验已归档于 `sources/xing_access_followup/README.md`：Semantic Scholar / OpenAIRE 未提供开放 PDF，arXiv 本次标题查询无结果，OpenReview 和 DBLP 新线索受访问验证限制。没有取得新的公式级材料；此项是严格三方验收的实际依赖，不是现有求解器尚未跑完。合法取得 Xing 全文后，再按该目录中的逐项核验清单替换代理并重新运行。
