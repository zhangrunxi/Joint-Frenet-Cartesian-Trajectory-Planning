# 本文、Reiter [20] 与 Xing [26] 最终实验报告

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent.
- Origin Date: 2026-10-30, Asia/Singapore.
- Verification Status: VERIFIED，限于本报告列明的独立方程重建、公交车适配及采样评估。
- Version Label: final-three-paper-bus-v2.
- 本报告取代 2026-10-29 报告中“Xing 全文缺失、只有代理”的当前状态判断。
- 论文正文尚未修改；不声称使用三篇论文的原作者部署代码或重现原车性能。

## 1. 结论先行

**三篇论文方法已在同一本机环境中完成独立重建与共同场景比较。
结果支持本文部分设计优势，但不支持本文整体优于 Reiter 和 Xing。**

1. 本文混合几何在所测窄路中完成超越，Reiter 的 ellipse/circles 重建及
   本文自己的 circles 消融跟车。相同本文模型中只替换碰撞几何的消融，
   加上独立几何网格审计，支持“相对这些碰撞近似降低保守性”。
2. **新加入的 Xing keep-left 走廊重建同样完成窄路超越，进度略高于本文。**
   因而必须撤回“只有本文能在窄路超越”或“优于所有 joint-frame prior art”的推断。
   旧 Xing-inspired 代理的中止，不能再作为 Xing 方法表现引用。
3. 同模型六/九状态消融支持减少冗余状态的开销收益，原生平均求解耗时
   减少约 16%。这不是相对 Xing 的实测加速比。本文也不是最低维：
   Reiter Direct 为五状态，本文为六状态，Xing 联合状态为七维。
4. 本文 SQP 版本在当前实现中耗时较小，但本机 Python AL-CiLQR 没有实时性优势。
   不同后端、成本、输入动态和诊断范围的耗时不能用来证明纯算法加速。
5. Xing 全文现在已经核对，原文未公开的数值权重、完整车体走廊构造和上游决策
   仍需适配。这与 Reiter、本文的重建边界一起披露，不把适配结果称为原作者复现表。

## 2. 比较内容与共同条件

本轮重新执行原有八个正式配置，并新增 Xing 七状态走廊配置和一个 yield 策略消融：

| 方法族 | 配置 | 数量 |
| --- | --- | ---: |
| Reiter 方程重建 | Direct/Lifted × ellipse/circles | 4 |
| 本文方程重建及消融 | 六状态 hybrid SQP、九状态 hybrid SQP、六状态 circles SQP、六状态 hybrid AL-CiLQR | 4 |
| Xing 方程重建 | 七状态 joint-frame QP，keep-left 主配置 | 1 |
| Xing 上游策略消融 | 同一规划器，改为始终 yield | 1 |

这是三类方法、十个配置；yield 不是另一个独立算法。旧代理只作历史保留，不混入本轮正式图表。

共同车辆为长 6.50 m、宽 2.30 m、轴距 4.32 m 的公交车，后悬 1.00 m；
N=40，dt=0.1 s，预测 4 s，闭环 12 s；初速 12 m/s，目标 15 m/s，
障碍物后轴 `s=35+8t,n=0`。直路曲率 0，弯道曲率 1/60 m^-1。
宽路半宽 8.5 m、横向目标 5.5 m；窄路半宽 4.6 m、横向目标 2.6 m。
三类方法共用这组环境和目标，但并非相同物理状态、成本及约束的优化问题。

所有规划调用顺序运行，数值库线程数为 1。主比较四十个 case 均完成 120 步，
共 **4,800 个执行周期**，独立真实矩形评估没有发现 0.02 s 采样处的碰撞或越界。
这是确定性合成场景，不能解释为统计成功率或连续时间安全证明。

## 3. Xing 原文实际实现了什么

全文为用户于 2026-10-30 提供的
`../../Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf`，
SHA-256 `197607c3606b992f2111bd32beeb9a59351031116403383185b0f35ff78c4825`。

新实现 `paper_comparison/xing2022.py` 采用：

- 七维状态 `[V,X,Y,psi,S,D,phi]`，输入 `[delta,a]`。
- Eq. (6) 的恒定输入精确离散模型，零转角用稳定连续延拓。
- Eq. (7) 的仿射动力学，以及 Eq. (17)/(19) 的完整坐标连接；
  相对航向行由 `phi=psi-theta_ref(S)` 推导，没有把它误当 Cartesian 航向。
- Eq. (12)--(15) 的先验 S 相关走廊，图 3 的输入与输入增量界，
  Eq. (20) 的未来状态跟踪和相邻输入增量代价。
- 逐轮更新先验、动力学、坐标连接和走廊，使用 OSQP；没有沿用旧代理的
  rollout 回溯线搜索或 Reiter 碰撞几何。

原文没有公布完整的车体走廊膨胀实现。本次以先验姿态下公交车矩形的 Frenet
包围范围进行道路收缩和障碍物扩张，圆弧道路考虑边内部径向极值。
主配置提供 keep-left、左侧被堵时 yield 的决策，另一配置始终 yield。
没有重现完整上游动态规划、任意多障碍拓扑或前方 overtake-wall 分支。
选择原文允许的硬约束版本，epsilon=0。

原文 Q/R 数值缺失，本次取
`Q=dt*diag(1000,0,0,0,1,500,1000)`、`R=diag(2e6,1)/dt`。
Q 为半正定，Cartesian 位置不单独跟踪；这是公开记录的适配，
不是声称与作者调参相同。另加入公交车横向加速度界和共同执行检查。
横向加速度约束和最终残差均检查每个恒定输入区间的起点、末端；
状态、输入及关键诊断必须为有限值。

停止同时要求先验位置平均变化 <=0.2 m 和非线性模型最大残差 <=1e-4；
后者是额外的严格接受检查，不能将其拒绝直接称为原论文自身失败。
完整原文审计及所有选择见
`sources/xing_paper/FIDELITY_REQUIREMENTS.md`、
`sources/xing_paper/IMPLEMENTATION_NOTES.md`。

审计曾发现 v1 的一个接受预测在区间末端达到 5.0084346 m/s²，
超过 5 的适配限值，而起点检查未发现；已执行轨迹没有该超限。
本报告采用修正两端检查和非有限值拒绝后**重新全量运行的 v2**。
旧源码、结果和报告快照保留，没有把 v1 改写成已修正的结果。
Xing 目标函数的整体归一化及原目标单位的对偶残差也已显式记录；
名义 QP 容差不能解释为与 SQP 相同的最优性精度。
Frenet 中增加 0.2 m 也不自动等价于曲线上 0.2 m 欧氏车体净空，
物理结果仍以独立真实矩形评估为准。

## 4. 规划表现

表内数值为 12 s 后的 Frenet s，单位 m。四场景中所有配置均完成；
“跟车”不是求解失败。

| 配置 | 宽路直线 | 宽路弯道 | 窄路直线 | 窄路弯道 |
| --- | ---: | ---: | ---: | ---: |
| Reiter Direct / ellipse | 176.594 | 190.591 | 121.377 | 120.817 |
| Reiter Lifted / ellipse | 176.594 | 190.591 | 121.377 | 120.817 |
| Reiter Direct / circles | 176.594 | 190.591 | 122.489 | 121.710 |
| Reiter Lifted / circles | 176.594 | 190.591 | 122.489 | 121.710 |
| 本文 6 / hybrid SQP | 177.127 | 191.096 | 177.342 | 183.820 |
| 本文 9 / hybrid SQP | 177.127 | 191.096 | 177.342 | 183.820 |
| 本文 6 / circles SQP | 177.127 | 191.096 | 122.093 | 121.308 |
| 本文 6 / hybrid AL-CiLQR | 177.145 | 191.102 | 177.340 | 183.846 |
| Xing 7 / keep-left corridor | 177.266 | 192.050 | 177.408 | 184.075 |
| Xing 7 / yield 策略消融 | 122.930 | 122.403 | 122.909 | 122.769 |

宽路中除主动 yield 消融外均完成超越。窄路中，本文 hybrid 和 Xing keep-left
完成超越，Reiter、本文 circles 和主动 yield 消融均跟车。
Xing 的 yield 行不能用来证明本文优于 Xing；它演示的是上游决策的影响。

本文 hybrid SQP 相比 Xing keep-left，四场景进度分别低
0.139、0.954、0.066、0.254 m。这些很小的差别受成本、控制变量和局部解影响，
**没有本文规划质量优于 Xing 的证据，也不据此宣称 Xing 一般优于本文。**

本文和 Reiter 的窄路进度差约 55--63 m，在本文自身几何消融中也存在。
原 Reiter force 代价等效为 `1345.6*a^2`，本文为 `1*a^2`；
既有单项权重敏感性把 Reiter 改为 `1*a^2` 后，circles 仍跟车，
但宽路的小进度差方向反转。因此上述差异不应全部归因于坐标表示。

### 窄路真实车体净空

| 配置 | 障碍物最小净空：直/弯 m | 道路最小净空：直/弯 m |
| --- | --- | --- |
| 本文 6 / hybrid SQP | 0.625 / 0.616 | 0.430 / 0.258 |
| Xing 7 / keep-left corridor | 0.539 / 0.396 | 0.495 / 0.478 |

本文在这两个 case 的障碍物净空更大，Xing 道路净空更大；
体现的是轨迹空间分配，不能把单个最小距离解释为方法整体更安全。

## 5. 效率与状态维数

下表 acados 列为去掉首次调用后的原生 `time_tot`；Xing/CiLQR 列为本机
Python 实现的规划 wall time。它们不是相同计时口径，不能相除形成方法加速比。

| 配置及计时口径 | 宽直 ms | 宽弯 ms | 窄直 ms | 窄弯 ms |
| --- | ---: | ---: | ---: | ---: |
| Reiter Direct / ellipse，acados 原生 | 0.793 | 0.889 | 5.255 | 7.247 |
| Reiter Lifted / ellipse，acados 原生 | 0.923 | 0.991 | 7.658 | 13.272 |
| 本文 6 / hybrid SQP，acados 原生 | 1.847 | 2.139 | 3.108 | 4.849 |
| 本文 9 / hybrid SQP，acados 原生 | 2.278 | 2.591 | 3.725 | 5.851 |
| Xing 7 / keep-left，Python wall | 20.710 | 27.349 | 19.729 | 26.086 |
| Xing 7 / yield，Python wall | 60.974 | 125.826 | 36.415 | 103.216 |
| 本文 AL-CiLQR，Python wall | 179.309 | 204.819 | 254.343 | 278.865 |

新主比较中 Xing keep-left 暖启动调用均小于 100 ms；yield 配置更慢，
宽路弯道超过 100 ms 的暖启动比例为 54.62%。本机实现的时序表现不是作者
车载性能，也不能仅用同场景均值推出尾延迟保证。
本文 AL-CiLQR 的窄路平均耗时约 254/279 ms，没有验证实时性优势；
其 status=0 仍仅代表可行性通过，没有独立驻点证书。

N=40 时，名义未消元多重打靶变量计数如下：

| 表示 | 状态维数 | `nx*(N+1)+nu*N` |
| --- | ---: | ---: |
| Reiter Direct | 5 | 285 |
| 本文 | 6 | 326 |
| Xing joint | 7 | 367 |
| Reiter Lifted | 8 | 408 |
| 本文 lifted 消融 | 9 | 449 |

本文该计数比 Xing 少 11.17%，比 Reiter Lifted 少 20.10%，
但比 Reiter Direct 多 14.39%。各方法包含的物理状态和代数变量不同，
这只是计数，不是控制其他因素的效率实验，也不是内存或 condensing 后维数。
AL-CiLQR 并不显式优化这些多重打靶状态变量，不能把这个公式说成其真实优化维数。

控制其他因素的证据来自既有**同模型六/九状态回放消融**：

| N | 六状态原生均值 ms | 九状态原生均值 ms | 减少 |
| --- | ---: | ---: | ---: |
| 20 | 2.459 | 2.922 | 15.85% |
| 40 | 8.594 | 10.295 | 16.52% |
| 80 | 29.309 | 35.065 | 16.42% |

这组回放共 1,512 次调用，含失败调用耗时，1,482 次接受；N=80 存在冷启动和
暖启动失败及失败后的重新初始化。双方接受的 741 对结果最大物理状态差
9.48e-7，支持其基本保持同一物理解。原始归档位于 `results/dimension_scaling/`。
因此建议使用“measured redundant-state overhead”，删除“dimensionality explosion”。

## 6. 几何保守性与道路宽度敏感性

既有与求解器无关的同向矩形网格审计中，真实净空 >=0.2 m 的安全位置，
被 hybrid/circles/ellipse 保守拒绝的比例分别为 **4.52% / 10.48% / 25.92%**。
三者在该有限网格均未接受不安全点。这支持 hybrid 相对这两种形状近似更少保守，
不支持所有姿态的连续保证，也没有测量 Xing 走廊的同一比例。
Xing 可以使用更贴近矩形的配置空间走廊，不能把 ellipse 的数值归给 Xing。

新增 Xing keep-left 六宽度、两道路扫描，与之前三配置扫描合并如下：

| 半宽 m | 本文 hybrid 直/弯 | 本文 circles 直/弯 | Reiter Lifted circles 直/弯 | Xing keep-left 直/弯 |
| --- | --- | --- | --- | --- |
| 4.0 | 跟车 / 跟车 | 跟车 / 跟车 | 跟车 / 跟车 | 12 步中止 / 4 步中止 |
| 4.3 | 超越 / 9 步中止 | 跟车 / 跟车 | 跟车 / 跟车 | 超越 / 28 步中止 |
| 4.6 | 超越 / 超越 | 跟车 / 跟车 | 跟车 / 跟车 | 超越 / 超越 |
| 4.9 | 超越 / 超越 | 超越 / 超越 | 超越 / 16 步中止 | 超越 / 超越 |
| 5.2 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 |
| 5.5 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 | 超越 / 超越 |

横向参考同步取 `half_width-2`，这是宽度和目标位置联合敏感性，不是单因素试验。
Xing 新增 12 个 case 中 9 个完成，执行 1,124 周期，停止前没有采样碰撞/越界：

- 4.0 m 直路 50 轮后先验平均变化仍约 0.659 m，未收敛。
- 4.0 m 弯道 50 轮后平均变化约 0.0103 m，但精确模型最大残差约 0.0107，
  被额外非线性可行性检查拒绝，不能说原论文的位置变化标准也判失败。
- 4.3 m 弯道下一个周期第 3 轮 QP 返回 primal infeasible inaccurate，停止。

这些是当前硬走廊、公交车适配、初始化和接受规则下的结果，不证明 Xing 原方法
在这些道路必然失败。尤其原文允许软约束和外部决策，本实验没有用额外策略
专门救某个失败 case。本文 hybrid 自身也有中止，不能宣称始终成功。

历史道路投影审计发现本文 circles 在 4.9 m 弯道物理道路净空仅 0.194173 m，
尽管名义裕量为 0.2 m。已有精确道路敏感性版本把对应 case 提高到
0.200000029 m，但其他 case 仍有节点间微小短缺和收敛失败。
本轮没有偷偷替换本文原模型；这个限制继续保留。

## 7. 验证、结果和复跑

完整测试 **254 passed, 147 warnings**；其中 Xing 新增 82 项独立测试及审计回归。
日志 `logs/final_with_xing_tests_v2.log`。警告主要为已有 NumPy/CasADi 和 acados
兼容/弃用警告；求解失败测试也会保留预期的原生错误输出。

`scripts/verify_final_comparison.py` 已核对主比较和新宽度扫描的
**52 个 case、5,924 个执行周期**，检查 summary、接受判据、源码和论文 SHA，
并独立用 scipy DOP853 重放全部 **2,084 个 Xing 执行周期**，重新计算 0.02 s
真实车体净空。主比较 960 周期重放最大状态误差约 5.7e-14。
清单为 `results/final_three_paper_manifest_v2.json`。
还检查了所有接受的 Xing 预测状态/输入均有限，以及全预测时域每个区间的
两端横向加速度，实际归档最大值为 **4.999999882 m/s²**。
没有认证非线性驻点，也没有对全部预测进行连续碰撞证明。

主要文件：

- 主比较原始结果：`results/final_three_paper_wide_v2/`、
  `results/final_three_paper_narrow_v2/`；均有全精度 summary CSV/JSON 和 provenance。
- 主比较图：`results/final_three_paper_plots_v2/trajectories.png`、
  `trajectories_cartesian.png`、`performance.png`；全精度合并表 `comparison.csv`。
- 宽度扫描：`results/xing_paper_width_sweep_v2/`；
  与历史扫描合并的图表 `results/final_width_comparison_plots_v2/width_sweep.png`、
  `width_sweep.csv`。
- 六/九维消融：`results/dimension_scaling/dimension_scaling.png`。
- 几何审计：`results/geometry_conservatism/geometry_conservatism.png`。
- 每个 Xing case 的 `*_predictions.json` 保存全部接受预测、控制和重算走廊；
  case 内保存逐轮先验误差与 QP 状态。其他方法仍沿用执行状态/控制和失败预测归档。
- 原有 Reiter 有效配置的补充见 `results/effective_method_configurations.json`；
  case 中共同 Settings 的本文权重不应被当成 Reiter 实际权重。
- `results/xing_paper_wide_v1/` 的首次 pilot 在 yield 结果序列化时遇到 NumPy
  bool 错误，已记录为 Python 归档错误并保留日志和精确源码快照；
  本报告采用后续完整重跑，没有把它计作算法失败。

复跑应使用新输出目录：

```bash
cd experiments/reiter20
bash scripts/run_final_comparison.sh --steps 120 --output results/recheck_wide
bash scripts/run_final_comparison.sh --steps 120 \
  --half-width 4.6 --lateral 2.6 --output results/recheck_narrow
.venv/bin/python scripts/run_xing_width_sweep.py --output results/recheck_xing_width

.venv/bin/python scripts/plot_paper_comparison.py \
  --inputs results/recheck_wide results/recheck_narrow \
  --merge-by-width --cartesian --output results/recheck_plots
```

所有方法都有权重、约束和边界适配；Reiter 原作者缺失类的重建说明、
终端约束差异、本文 Eq.34e 修正和道路投影限制仍见历史
`ACCEPTANCE_REPORT.md`。不得因为三方现在能运行，就删掉这些复现边界。

## 8. 对评审意见的最终回答

现在可以回应：新增 Reiter 和 Xing 的原文方程重建基线，按共同公交车场景评估，
并用相同物理模型的状态维数消融和几何消融分离两项设计作用。

必须同时承认：

- joint-frame 思路不是本文首创，不能把它本身作为区别于两篇工作的贡献。
- “leaner state”只相对某些联合/提升表示成立；Reiter Direct 更低维。
- “dimensionality explosion”没有依据，应改成量化的冗余状态开销。
- 混合几何的优势针对部分形状近似和所测紧空间，Xing 走廊也能超越；
  没有证明本文对所有 joint-frame 方法的普遍优势。
- 本机 Python AL-CiLQR 没有实时性优势，不能把 SQP 消融计时写成 CiLQR 成绩。

建议把贡献重点放在**长车的混合车体约束、特定空间条件下较少保守的规划结果，
以及经过受控实验测量的状态组织开销**。这比“首次联合坐标、全面更快、
窄路普遍更强”更符合本轮实验实际。

## 9. 补充：Reiter 原文 Table 2 的核对结果（2026-10-30）

为准备论文修改稿，逐条核对了 Reiter 原文（`sources/Reiter2023a.pdf`）中与本文
相关工作表述对应的内容，结果记录如下，供正文引用时使用。

1. **Table 2（CF 表示形式对比）** 中，"Lifted ODE Frenet"（8 状态）的问题一栏写为
   **"redundant states"**，"Direct Elimination Frenet"（5 状态）该栏为空。
   即「提升表示产生冗余状态」这一观察出自 Reiter 自己的对比表。
   本文的六状态表示处于这两者之间。

2. **Table 4（参数）** 中道路侧只有一个 "road bounds" 参数（道路半宽），
   其约束作用于参考点，不是整车轮廓。因此本文原稿中
   「both [26] and [20] fail to explicitly model the complete vehicle contour in
   road constraint formulations」对 [20] 成立，但表述宜更精确。

3. **Table 5 与正文** 报告了各障碍物几何形式（椭圆、覆盖圆、分离超平面）的
   计算耗时与进度，并指出分离超平面形式在耗时与进度上都更差。
   本文的重建只覆盖椭圆与覆盖圆两种，未实现分离超平面形式。

4. **第 2.3 节** 说明 Frenet 坐标下的障碍物形状变换在 SQP 下不能保证安全过近似，
   这是 [20] 的核心论点之一，与本文「Frenet 障碍物过近似」的批评方向一致。

这些核对结果使相关工作可以改写为有出处、可测量的表述，取代原来无依据的
"dimensionality explosion" 一语。相关改写草稿见
`../../论文修改稿_评审意见2和3.md`。

## 10. 补充：Xing 原文状态定义的核对结果（2026-10-30）

核对 `Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf`：

- 原文 Eq. (2) 定义笛卡尔状态 `p_c = [V, X, Y, psi]ᵀ`（含惯量），
  Eq. (3) 定义控制 `u = [delta, a]ᵀ`；
  Eq. (21) 的联合状态为 `[p_cᵀ, p_fᵀ]ᵀ`，Frenet 部分 `p_f = [S, D, phi]ᵀ`
  （见 Eq. (11)）。因此联合状态共 7 维，与本重建的
  `[V, X, Y, psi, S, D, phi]` 一致。
- Eq. (9) 给出 Frenet 状态对笛卡尔状态的确定性连接（9a–9c），
  说明联合状态中的 `S` 可由 `X, Y, psi` 恢复，属于确定性重复信息。
  这与 Reiter 原文 Table 2 把 lifted 表示的问题列为 "redundant states" 是同一现象。
- 因此论文原稿「redundant coupling of Cartesian and Frenet states causes state-space
  dimensionality explosion」中，前半句（冗余耦合）在两篇原文里都能找到依据，
  后半句（爆炸）在两篇原文里都没有依据。改写时应保留前半句、去掉后半句，
  并改为引用本文自己的受控消融数据。
