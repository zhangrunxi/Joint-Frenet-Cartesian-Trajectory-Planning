# VehiclePlannerAcados20222 独立重实现记录

## 结论与范围

已结合论文 [20] 和当前 `vehicle_gym` 代码，实现可在本机 Apple Silicon 上编译运行的 `reiter2023.VehiclePlannerAcados20222`。采用实际 acados / HPIPM 后端，不是 IPOPT 替代。该实现是依据论文的独立重实现，不是作者提供的原始 `20222` 类，也没有让原始 `comparisons_short.py` 直接恢复运行。

两种表示均已完成滚动规划测试：

- Direct Elimination：状态 `[s, n, alpha, v, delta]`，Cartesian 位姿由逆 Frenet 变换计算。
- Lifted ODE：上述状态加 `[x, y, phi]`，以一致的初值同时积分 Cartesian ODE。
- 障碍物在 Cartesian 坐标中约束，支持椭圆和覆盖圆；两种表示使用相同车辆、代价、边界和预测输入。
- 分离超平面、Conventional Frenet、论文完整随机场景集和原始对手车辆 NMPC 尚未实现。

当前结果足以支持继续开发对比平台，不足以声称完整复现原论文的统计结果、运行时间或安全保证。

## 代码与公式对照

| 论文内容 | 实现位置 | 当前处理 |
| --- | --- | --- |
| 式 (1)、(2)：力、转角率、横向加速度 | `reiter2023/trajectory_planner_acados_20222.py` 的 `_create_model` | 包含风速、风向、空气阻力和滚动阻力；物理控制单位为 N、rad/s |
| 式 (9)：Frenet ODE | 同上 | `s_dot=v*cos(alpha)/(1-n*kappa)`；使用后轴横向加速度 |
| 式 (12c)、(13)：增广初值及 Cartesian ODE | `augment_state`、`_create_model` | 每次重规划从测量的五状态构造一致初值；另行记录预测坐标一致性误差 |
| 式 (14)：椭圆约束 | `reiter2023/geometry.py` 的 `ellipse_constraint` | 障碍物矩形的覆盖椭圆加自车覆盖圆半径；使用同一后轴到车体中心偏移 |
| 式 (15)：覆盖圆约束 | `covering_circles`、`_create_model` | 复用当前 `vehicle_gym` 的圆心构造；检查圆数量足以覆盖车体；对每对圆施加距离约束 |
| 式 (17)：状态和控制代价 | `_create_solver`、`set_states` | 阶段权重使用表 4 的数值，由 acados 乘一次 `dt`；因 acados 的 `1/2` 约定，矩阵乘 2；终端代价不乘 `dt` |
| 式 (18)、(19)：多重打靶 | `_create_solver` | RK4、Gauss–Newton、PARTIAL_CONDENSING_HPIPM，condensing horizon 为区间数的一半 |
| 求解后约束检查 | `evaluate_solution` | 初值、RK4 缺陷、状态/控制边界、横向加速度、障碍物及坐标一致性分别记录 |

保留了 `set_states`、`set_obstacles`、`warm_start`、`solve`、`reset_solver`、`get_statistics` 以及当前 `Planner` 的部分输出接口，但并非旧工程的完全兼容替身。`set_obstacle_predictions` 接收明确的 Cartesian 后轴预测数组 `(n_obstacles, n_nodes, 3)`，优先使用该接口避免坐标歧义。

当前 `SampledRoad` 适配 `vehicle_gym.Track` / `Road`；采样道路的几何样条、航向和曲率并非严格解析一致，域外外推也尚未作为正式场景支持。正式比较前应固定有效参考线域并单独检查其导数一致性。本次主实验使用解析直线/圆弧，排除该因素。

## 实际测试结果

### 单次 RTI：原始失败记录保留

目录：`results/reconstructed_full/`；日志：`logs/reconstruction_full.log`。

默认每周期一次 RTI、中心线参考、80 个请求周期：4 个直路配置通过，4 个弯道配置在开始阶段被检查器拒绝。求解器状态为 0，但预测轨迹 RK4 动力学缺陷约为 `2.3e-4` 至 `3.4e-4`，大于当前 `1e-4` 门槛。

这不是证明原方法不可行，也不等同于物理碰撞；它说明求解器返回状态 0 不能代替非线性可行性检查。本次没有放宽门槛以掩盖失败。

### 完整 SQP：8 个配置全部通过

目录：`results/reconstructed_sqp/`；日志：`logs/reconstruction_sqp.log`。

每个配置滚动运行 120 周期、共 12 秒，8 个配置共 960 周期。无求解失败或检测到的矩形碰撞。以下耗时仅为本机求解器诊断，排除代码生成、编译、初始化及 Python 评估，不用于与论文部署数据直接比较。

| 表示 | 障碍物模型 | 道路 | 平均求解时间 (ms) | P95 (ms) |
| --- | --- | --- | ---: | ---: |
| Direct | 椭圆 | 直路 | 1.482 | 1.820 |
| Direct | 椭圆 | 弯道 | 10.804 | 18.500 |
| Direct | 三圆 | 直路 | 2.215 | 2.631 |
| Direct | 三圆 | 弯道 | 12.909 | 20.954 |
| Lifted | 椭圆 | 直路 | 1.741 | 2.099 |
| Lifted | 椭圆 | 弯道 | 11.686 | 21.665 |
| Lifted | 三圆 | 直路 | 2.579 | 3.119 |
| Lifted | 三圆 | 弯道 | 14.902 | 23.708 |

逐用例和逐周期数据以 JSON 为准，汇总为 `summary.csv`。SQP 测试中的最大预测 RK4 缺陷小于 `2e-13`；增广弯道的坐标一致性最大分量误差小于 `3e-7`。这些小误差并不构成安全保证。

椭圆模型轨迹图为 `results/reconstructed_sqp/trajectories.png`。Direct 和 Lifted 的轨迹基本重合，符合其描述同一物理模型的预期。

### 显式 RTI 可行性细化：8 个配置全部通过

目录：`results/reconstructed_rti_refined_run/`；日志：`logs/reconstruction_rti_refined_run.log`。

同样运行 8 × 120 周期，但每周期允许最多 5 次 RTI，仅在残差超过门槛时继续，实际最多使用 3 次。所有调用、状态、累计求解时间及残差均写入逐周期记录。

这不是论文单次 RTI 配置，不能将其耗时冒充单次调用耗时。弯道增广坐标的一致性最大分量误差约为 `1.55e-3`，明显高于完整 SQP；后续几何评价应继续使用独立真实轮廓，而不是只检查优化器内部约束。

### 自动化检查

`logs/reconstruction_all_tests.log`：15 项通过，包括 7 项重实现模型/几何检查、5 项本机生成代码集成检查、3 项原有运行工具检查。

集成检查包括：四个模型组合的编译及可行性、故意设置不可行横向加速度初值后的失败识别、当前 `vehicle_gym.Track` 的采样道路接入。不可行性测试中的 QP 报错是预期的拒绝路径；上游接口同时输出若干弃用警告，不能将这些警告视为无风险证据。

最终回归同时包含上游 21 项测试，共 36 项通过，见 `logs/final_regression_tests.log`。本机启动脚本另做了 5 周期弯道编译运行检查，见 `results/reconstructed_wrapper_check/summary.json`。源码、依赖提交和主结果文件的 SHA-256 记录在 `results/reconstruction_provenance.json`。

## 相对论文的差异和待补工作

1. 主测试为一个恒速障碍物、直线和曲率 `1/60 m^-1` 的圆弧，不是论文每类 500 次随机、三个对手车辆、20 秒的实验。自车初速为 12 m/s，目标速度 25 m/s，障碍物初始 `s=35 m`、速度 8 m/s，绕行横向参考为 4 m；这些是验证用设置，不是声称恢复原始算例。
2. 采用 40 个控制区间、41 个状态节点、`dt=0.1 s`。论文表 4 写 `N=40`，而旧脚本的节点/区间计数存在差别，当前明确记录而不假定完全一致。
3. 车辆轴距、质量、车体尺寸、物理边界及代价值主要沿用表 3、表 4；阻力系数来自当前 `vehicle_gym` 参数默认值，原文没有提供足够数值确认其完全一致。风速为论文的 20 m/s、风向 0。
4. 当前实现使用硬约束、没有 slack。公开脚本中的 slack 权重接口与缺失求解器如何组合无法核实，因此未推断为原作者代码。新增 Frenet 有效域约束 `1-n*kappa>=0.1` 防止奇异；不能在以后消融时只给一方加该约束。
5. 道路边界约束作用于后轴 Frenet 点，不是完整车体轮廓。测试的「无碰撞」针对自车和障碍物的 Cartesian 矩形，不能解释为车身没有越过道路边界。
6. 碰撞检查每 0.02 秒检查一次矩形，仍非连续时间碰撞证明；输出的 `min_center_distance_m` 是车体中心距，不是净空。椭圆半轴膨胀遵循论文方案，不额外宣称其为任意几何条件下的精确 Minkowski 和。
7. Lifted 仅通过一致初值及 ODE 传播关联两套坐标，没有逐节点添加坐标代数等式；单独记录数值漂移。仿真每周期以五状态测量重建增广初值，不能省略该语义。
8. 完整 SQP 及多次 RTI 是诊断变体。原文单次 RTI 的 warm start、精确初始化、约束软化和失败处理仍需核对，不能为了匹配时间反向调参。
9. acados 使用当前固定提交、BLASFEO/HPIPM 的 GENERIC 构建，OpenMP 关闭，与论文 2023 年环境和 Intel 硬件不同；当前 Direct 较快不能直接证明低维方案普遍优于 Lifted。

## 复跑命令

在 `experiments/reiter20/` 执行。结果目录采用固定文件名；需要保留旧记录时使用新的 `--output` 目录。

```bash
# 完整 SQP 验证，两种状态 × 两种模型 × 两种道路。
bash scripts/run_reconstruction.sh --steps 120 --solver SQP \
  --lateral-reference 4 --output "$PWD/results/reconstructed_sqp_repeat"

# 单次 RTI 的原始门槛检查；该设置可能退出 1。
bash scripts/run_reconstruction.sh --steps 80 \
  --output "$PWD/results/reconstructed_rti_repeat"

# 显式允许 RTI 可行性细化，不作为原论文单次 RTI 结果。
bash scripts/run_reconstruction.sh --steps 120 --lateral-reference 4 \
  --rti-iterations 5 --output "$PWD/results/reconstructed_rti_refined_repeat"

# 含本机 C 代码生成的测试。
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" \
REITER_RUN_INTEGRATION=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python -m pytest -q tests

# 仅重新绘制已有结果。
.venv/bin/python scripts/plot_reconstruction.py results/reconstructed_sqp
```

重建依赖可运行 `bash scripts/setup_acados.sh`。它要求本目录已有 `.venv` 和可用的 Xcode Command Line Tools，所有安装写入实验目录。固定 acados 提交为 `a58e76ad328d97aea180b55344e80eae346f8d15`；Python 包快照为 `requirements-reconstruction.lock`，CasADi 因 acados 兼容要求从先前 3.8.2 调整为 3.8.1。旧 `requirements.lock` 保留为首次 portable 检查的环境记录。

首次构建遇到两项环境问题，日志保留：CMake 找不到虚拟环境内的 Ninja，改为指定绝对路径；macOS 无法解析 `@rpath/libhpipm.dylib`，由启动脚本在 Python 进程启动前设置 `DYLD_LIBRARY_PATH` 解决。没有修改系统动态库路径或上游源码。

## 后续对比的使用方式

建议先扩大无随机歧义的测试覆盖，再确定是否将重实现作为正式基线。对比表应标注 `Our reimplementation of Reiter et al. [20]` 并披露求解设置；不能标成 `official implementation`。下一阶段可接入本文模型，但仍需确认本文工程算法的实际源码和 CiLQR 行为，再设计统一物理约束及独立几何评估器。
