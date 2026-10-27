# HANDOFF — PID 时间指标精度改进

日期：2026-10-26。工作区：`D:\control-pid`；MATLAB R2022b 9.13.0.2049777；非 Git 仓库。

## 状态与范围

本任务已实现并实际验证。修改 `pid_metrics.m`、`saturation_metrics.m`、`simulate_pid.m`、`simulate_saturation.m`、`stage2_check.m`；新增 `response_time_metrics.m`、`pid_time_metricsTest.m`。README §4.2 记录算法选择、独立参考、新旧结果、Refine 敏感性及成本；PROJECT、TASK、events 按项目规则更新。

控制律、RHS、物理参数、增益、积分器输出、容差及默认参数未变。原有效性判断代码块逐字比较相同。未覆盖旧实验表或生成实验结果文件，未安装软件、修改配置或初始化 Git。上一阶段 `pid_metricsTest.m` 未修改。

## 行为与兼容性

- 仿真入口把已有速度状态传入指标，位置曲线使用三次 Hermite 插值；旧 3/4 参数 `pid_metrics` 调用保持可用，缺少速度时用线性插值。
- 每段求内部极值并在单调段上定位交点，调节时间取最后入带；严格带外语义不变。新增 `peak_time`、`time_method`。
- 饱和时间按线性指令曲线计算，仍保留 `u_max+1e-9` 门限；首次退出时的积分状态／指令同步插值，新增 `saturation_time_method`。
- 原稳态／调节有效性门控不变。辅助函数只计算插值曲线，不建立稳态有效性；不得绕过 `pid_metrics` 用其原始诊断值报告正式指标。
- 扰动前段自动使用新位置指标；扰动后的独立恢复时间仍保留原采样算法。

## 实际执行与结果

通过现有 MATLAB MCP 执行；没有新增工具箱：

```matlab
rehash
clear pid_metrics response_time_metrics saturation_metrics simulate_pid simulate_saturation pid_time_metricsTest
precisionFinal = runtests({'pid_metricsTest.m','pid_time_metricsTest.m'});
% FINAL: 47 passed, 0 failed, 0 incomplete
assert(all([precisionFinal.Passed]) && ~any([precisionFinal.Incomplete]));

stage1_verify
verify_model
stage2_check
verify_disturbance
verify_saturation
% 五个脚本逐个执行，均完成且全部断言通过。
```

- 两个测试文件也分别经 MCP run_matlab_test_file 执行：28/0/0、19/0/0。
- 新增测试最初为 16 通过、3 失败、2 未完成；修复单区间重根的行列形状错误，并修正测试的 G2 上界求根区间和 MATLAB 错误标识。第二次 18/1/1，收紧 G2 求根括区后 19/0/0。
- 审查时临时把带外判断从 `>` 改为 `>=`，相切边界测试按预期 0 通过、1 失败、0 未完成；随后恢复源码，再运行上述完整最终验证。
- 7 个变更／新增 .m 文件 Code Analyzer 均为 0 条消息。
- 对基线 Refine=1/4/20 和三个默认饱和组，修改前后 t/x 完全相同；基线 convergence 通过 isequaln 比较相同。
- 独立参考为矩阵指数／分段矩阵指数，不通过生产 RHS 或指标生成预期值；覆盖饱和后的再次离带、后续极值、10 s 后模态包络。
- Refine=1 的 PID-15 基线正式指标仍无效：80–90% 窗口只有 2 个样本，原因 insufficient_tail_samples。比较算法时的原始交点只用于诊断。
- 默认 PID-15 三个新时间与参考最大差约 3.75e-9 s；G2/G3 约 2.93e-8、3.54e-9 s。测试验收限分别为基线 1e-6 s、饱和位置 2e-6 s、饱和退出 1e-6 s。
- 六组增益指标已重算；变化来自交点／极值插值，原饱和排序断言通过。具体数据和一次本机 timeit 成本对比见 README §4.2。

## 限制与下一步

有限数值记录上的插值极值／交点不等于严格连续时间证明，也不保证未来行为。积分误差、非光滑切换、平坦交点及尾段估计仍限制精度；不能将实测纳秒级差异宣传为普遍误差保证。未运行图形展示入口、视觉验收、正式性能测试或覆盖率报告。

本任务没有未完成项。后续如需改进扰动 recovery_time，应独立立项，保留其相对参考值和扰动时刻的定义，先建立对应独立参考；不要顺便改变本次稳态判据。

## Simulink PID-15 基线模型

日期：2026-10-27。新增 `pid_baseline_simulink.slx` 与 `build_and_verify_simulink_pid_baseline.m`。模型使用显式 Step、Sum、Gain、Integrator 和 To Workspace 模块，控制律为 `10*(r-x)+15*x3-3*v`，对象为 `x''=(u-v-4*x)`；初始状态全零，StopTime 30 s，ode45，RelTol 1e-8，AbsTol 1e-10。微分项直接使用测量速度。

MATLAB R2022b / Simulink 10.6 中实际运行构建脚本。由于 Simulink 与 MATLAB ode45 的内部步选择不同，模型保存为 `MaxStep=0.005` 以控制比较成本；统一 30001 点 pchip 比较得到：6006 个 Simulink 输出样本、1341 个 MATLAB 样本，最大位置／速度／控制力差分别为 `6.791e-06`、`3.127e-05`、`4.948e-05`；稳态位置分别为 `1.000000000000` 与 `1.000000000001`。脚本阈值为位置／速度／控制力 2e-3、稳态位置 2e-4，实际差异远低于阈值。Code Analyzer 对脚本无消息。

首次以模型默认最大步长运行时差异为 `1.615e-04 / 1.536e-03 / 1.406e-03`，仍通过工程阈值；随后将模型 MaxStep 设为 0.005 并再次运行，取得上面的最终结果。脚本若发现已有模型只加载并验证，不覆盖；删除或改名后可重新生成。

未修改 MATLAB PID 源码、指标、历史实验或配置；未运行图形展示入口。模型输出是 Structure With Time，脚本会检查非有限值和三条轨迹差异，失败时抛出明确断言。

## 10 s 常值扰动 Simulink 对照

日期：2026-10-27。新增 `pid_disturbance_simulink.slx` 与 `build_and_verify_simulink_pid_disturbance.m`。模型保留显式 PID-15 闭环结构，加入 `Step(Time=10, After=-4)` 外力；脚本逐次设置 P、PD、PID-15 的 `[Kp Ki Kd]`，并与 `simulate_disturbance` 的分段 MATLAB 参考比较。

MATLAB R2022b / Simulink 10.6 实际运行结果（Simulink `MaxStep=0.005`，统一 30001 点 pchip）如下：P 的最大位置／速度／控制力差为 `1.224e-05 / 5.512e-05 / 1.224e-04`，PD 为 `3.952e-06 / 1.297e-05 / 1.340e-04`，PID-15 为 `6.791e-06 / 3.127e-05 / 4.948e-05`。三组扰动前位置差分别为 `1.224e-05 / 3.952e-06 / 6.791e-06`，扰动后分别为 `4.780e-06 / 3.464e-06 / 3.458e-06`，均通过脚本的 2e-3 工程阈值。

MATLAB 后段稳态位置：P `0.428570562`、PD `0.428571429`、PID-15 `1.000000000`；对应 Simulink 值相同到输出精度。P/PD 因无积分无法回到参考带，恢复时间为 NaN；PID-15 恢复时间为 `1.70349 s`（沿用项目原有采样定义）。`verify_disturbance.m` 也实际通过。
## Simulink 执行器饱和与 clamping

2026-10-27 新增并实际运行 `pid_saturation_simulink.slx` 与 `build_and_verify_simulink_pid_saturation.m`。模型包含 G1 理想执行器、G2 饱和无抗积分、G3 饱和 clamping 三个并行分支；G3 使用原生 Product/Relational Operator/Switch 实现 `(u_des-u_act)*e > 0` 时 `dx3=0`。

模型设置为 MATLAB R2022b / Simulink 10.6、ode45、RelTol=1e-8、AbsTol=1e-10、MaxStep=0.005、30 s。三组位置/速度/积分状态最大差异：G1 `[4.464e-6,5.103e-5,4.397e-6]`，G2 `[3.517e-6,5.758e-5,6.539e-6]`，G3 `[3.517e-6,3.359e-5,6.108e-4]`，均通过 2e-3 状态阈值。G3 `u_des/u_act` 最大差异为 `4.141e-3/5.830e-3`，记录为非光滑切换数值限制，未宣称控制信号完全一致。

实际执行：新建脚本静态检查 0 条问题；模型保存并运行成功；新脚本验证通过；新脚本直接检查 Simulink G3 饱和推动区间的 `max|dx3|=0`；`verify_saturation.m` 全部通过。原 MATLAB 指标 G1/G2/G3：rise `0.4639/0.5290/0.6281 s`，overshoot `24.1931/33.2519/8.2636 %`，settling `2.6265/3.4735/2.7488 s`，IAE `0.5980/0.7779/0.6087`；饱和时间 `G2=0.5948 s`、`G3=0.2696 s`，G3 首次退饱和时 `x3=0`。
