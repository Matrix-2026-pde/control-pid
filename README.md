# PID 闭环仿真：质量–弹簧–阻尼系统

一个面向学习与工程仿真实践的 MATLAB 项目：从二阶质量–弹簧–阻尼对象出发，比较 PID 参数、检验常值扰动下的积分作用，并观察执行器饱和与 clamping anti-windup 的取舍。项目强调可复现的建模、数值仿真、指标分析和独立验证；结果仅适用于本文档列明的模型、参数、采样与仿真条件，不构成对任意系统的性能保证。

## 快速概览

对象模型为 `m*x'' + c*x' + k*x = u + d`，其中 `m=1 kg`、`c=1 N·s/m`、`k=4 N/m`，参考值 `r=1 m`。控制器使用测量速度实现 D 项，避免参考阶跃在 `t=0` 产生微分冲激；I 项作为第三个状态积分 `r-x`。基础路径只使用 MATLAB 内置的 `ode45`、`odeset`、`roots` 和绘图函数，不依赖 Control System Toolbox。

项目包含三个阶段：

1. 参数对比：P、PD、PI、PID-5、PID-15、PID-30 六组增益；`Ki=0` 使用二阶降阶解释，`Ki>0` 使用三阶极点与 Routh margin。
2. 常值扰动：在 `t=10 s` 施加 `d=-4 N`，比较 P、PD 与 PID-15 的稳态误差、恢复和力平衡。
3. 执行器饱和：PID-15 下比较 G1 理想执行器、G2 `u_max=5 N` 无 anti-windup、G3 `u_max=5 N` + clamping。

## 结果图

以下 PNG 均由本次在 MATLAB R2022b 中直接运行相应 `run_*` 入口生成，不是手工绘制的示意曲线。

| 实验 | 图 | 条件 |
|---|---|---|
| 参数对比 | [pid-comparison.png](results/pid-comparison.png) | `T=30 s`，六组增益，`r=1 m` |
| 常值扰动 | [pid-disturbance.png](results/pid-disturbance.png) | `d=0`（`t<10 s`），随后 `d=-4 N` |
| 饱和与 clamping | [pid-saturation.png](results/pid-saturation.png) | PID-15，G2/G3 `u_max=5 N` |

## 主要观察

- 参数对比中，P/PD 的 `Ki=0` 会留下约 `0.286 m` 的参考稳态误差；加入积分后可消除该误差，但三阶闭环不能用单一阻尼比概括。
- 扰动实验中，P/PD 对 `-4 N` 常值扰动保留约 `0.571 m` 的后稳态误差；PID-15 恢复到参考附近，验证了积分项的抗常值扰动能力。
- 饱和实验中，G2 的 windup 增大超调和调节时间；G3 的 clamping 减小超调并保持有限的积分状态，但上升时间较 G1 更长。这里的结论是本组参数下的观测，不是一般性控制保证。

## 复现入口

基础 MATLAB 实验入口：

```matlab
run_comparison
run_disturbance
run_saturation
```

模型、控制律、三阶段实验和结果口径见 [docs/model-and-experiments.md](docs/model-and-experiments.md)；指标定义、稳态有效性、插值和独立验证见 [docs/metrics-and-validation.md](docs/metrics-and-validation.md)。

Simulink 是独立的对照路径，不是基础 MATLAB 路径的依赖。三个模型分别为 `pid_baseline_simulink.slx`、`pid_disturbance_simulink.slx` 和 `pid_saturation_simulink.slx`，构建/验证脚本及其副作用说明见 [docs/simulink-comparison.md](docs/simulink-comparison.md)。

环境、工作目录、测试命令和当前验证状态见 [docs/reproducibility.md](docs/reproducibility.md)。

## 当前验证摘要

本次实际运行（MATLAB R2022b / 9.13，Windows 11）：`pid_metricsTest.m` 为 28/28 passed，`pid_time_metricsTest.m` 为 19/19 passed；`stage1_verify`、`verify_model`、`stage2_check`、`verify_disturbance`、`verify_saturation` 五个脚本均通过。验证输出中的误差与阈值、以及未运行的 Simulink 项目详见复现文档；历史记录不作为本次运行证据。

## 局限与深入阅读

仿真采用有限时长和有限采样记录；稳态指标、峰值和调节时间受窗口、采样密度、插值方法与相对误差带影响。基础实验未覆盖噪声、非线性摩擦、参数不确定性、斜坡/正弦扰动、饱和敏感性扫描、`u_max < k r` 的不可达参考和 back-calculation 对比。G3 在非光滑切换点保留 `u_des/u_act` 的数值差异，不应写成完全一致。

原始完整 README（改写前，可追溯保存）：[docs/archive/README-original-2026-10-27.md](docs/archive/README-original-2026-10-27.md)。项目协作入口与状态记录见 [AGENTS.md](AGENTS.md)、[PROJECT.md](PROJECT.md)、[TASK.md](TASK.md) 和 [events/](events/)。个人与 AI Agent 的具体分工未在项目证据中确定，暂不在展示文案中归属。
