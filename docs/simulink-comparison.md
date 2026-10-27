# Simulink 模型与 MATLAB 对照

Simulink 路径是独立对照实验，需要 MATLAB R2022b / Simulink 10.6；基础 MATLAB 路径不需要 Simulink，也不需要 Control System Toolbox。模型文件为：

- `pid_baseline_simulink.slx` — PID-15 基线；
- `pid_disturbance_simulink.slx` — P、PD、PID-15 与 `t=10 s` 的 `-4 N` 扰动；
- `pid_saturation_simulink.slx` — G1/G2/G3 饱和对照。

对应脚本为 `build_and_verify_simulink_pid_baseline.m`、`build_and_verify_simulink_pid_disturbance.m`、`build_and_verify_simulink_pid_saturation.m`。脚本会加载已有模型；模型不存在时才新建并保存 `.slx`，因此运行前应确认工作副本可接受模型生成。脚本会运行仿真并将工作区输出用于与 MATLAB 参考轨迹比较。

## 2026-10-27 独立副本复验

在 `D:\control-pid\.tmp-simulink-verification` 副本中使用 MATLAB R2022b / Simulink 10.6、`ode45`、`StopTime=30 s` 运行三个脚本。原有模型文件存在，因此三个脚本均走加载已有 `.slx` 分支；副本中的三个模型 SHA-256 运行前后不变，未生成 `.mat`、`.slxc` 或日志文件。模型缺失时，脚本才会 `new_system` 并 `save_system` 覆盖/创建对应 `.slx`。

| 脚本 | 结果 | 实际最大误差 | 脚本验收容差 |
|---|---|---|---|
| `build_and_verify_simulink_pid_baseline.m` | 通过 | 位置 `6.791e-6`、速度 `3.127e-5`、控制力 `4.948e-5` | 各 `<2e-3`；稳态位置差 `<2e-4` |
| `build_and_verify_simulink_pid_disturbance.m` | 通过 | P/PD/PID-15 的位置最大差分别 `1.224e-5`、`3.952e-6`、`6.791e-6`；速度最大差 `5.512e-5`、`1.297e-5`、`3.127e-5`；控制力最大差 `1.224e-4`、`1.340e-4`、`4.948e-5` | 位置、速度、控制力各 `<2e-3` |
| `build_and_verify_simulink_pid_saturation.m` | 通过 | G1 `[y,v,x3,u_des,u_act]=[4.464e-6,5.103e-5,4.397e-6,8.493e-5,8.493e-5]`；G2 `[3.517e-6,5.758e-5,6.539e-6,3.195e-4,1.286e-4]`；G3 `[3.517e-6,3.359e-5,6.108e-4,4.141e-3,5.830e-3]` | `y/v/x3` 与参考的关键检查 `<2e-3`；G1 力 `<2e-3`；G2/G3 积分态 `<2e-3`；限幅误差 `1e-9`；clamping `|dx3|<1e-9` |

G3 的 `u_des/u_act` 差异在非光滑切换处最大为 `4.141e-3/5.830e-3`，脚本没有把这两个量作为 `<2e-3` 的统一判据；该差异与既有边界记录一致，不能表述为完全轨迹一致。上述结果是本次复验结果，不是历史记录的转述。
