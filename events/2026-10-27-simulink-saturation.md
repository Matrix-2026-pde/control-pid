# 2026-10-27 Simulink 执行器饱和与 clamping

## 背景
将第三阶段 MATLAB 饱和实验迁移到 Simulink，保持原控制律与实验条件。

## 实际操作
新增 `pid_saturation_simulink.slx` 和 `build_and_verify_simulink_pid_saturation.m`。使用三并行分支；G3 原生 Switch 条件严格对应 `(u_des-u_act)*e > 0`。

## 验证结果
MATLAB R2022b / Simulink 10.6 实际保存并运行。新脚本通过，位置/速度/积分状态差异均小于 2e-3；`verify_saturation.m` 全部通过。G3 控制信号最大差异 `u_des=4.141e-3`、`u_act=5.830e-3`，归因于非光滑切换的自适应步定位差异，未放宽阈值。

## 下一步
无必需动作；保留该数值限制，避免将其误报为完全一致。
