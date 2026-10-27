# PROJECT — 二阶机械系统 PID 闭环仿真

> 本文件只记录“项目状态与决策”；模型推导、公式、实测数据、验证细节以 `README.md` 为准。

## 目标

用基础 MATLAB 仿真二阶质量-弹簧-阻尼系统的 PID 闭环，理解 P/I/D 各自作用、三阶系统的结构性权衡，并做可复现的数值验证。

## 当前进度

- [x] 第一阶段：模型与六组增益对比（P、PD、PI、PID-5/15/30），含解析解、稳态、容差、极点/Routh 验证。
- [x] 第二阶段：常值外部扰动实验（P、PD、PID-15 三组），验证 I 项抗扰。
- [x] 第三阶段：执行器饱和与积分饱和（G1 理想、G2 饱和无抗饱和、G3 饱和 + clamping）。
- [x] 2026-10-26 指标有效性：区分末端估计、稳态／调节时间状态和原因；28 个专用测试与原五个验证通过，六组 30 s 基线数值不变。
- [x] 2026-10-26 时间精度：位置／速度 Hermite 插值和线性饱和时间估计，新增独立矩阵指数参考及边界测试；47 项测试、五个原验证通过，历史实验表保留，新结果见 README §4.2。

## 技术选型

- MATLAB 基础函数：`ode45`、`odeset`、`roots`、`plot`。
- 不使用 Control System Toolbox、Simulink；Git 只用于版本管理，不参与仿真。
- 数值容差：`RelTol=1e-8`、`AbsTol=1e-10`。
- 运行方式：MATLAB 中 `run_comparison` / `run_disturbance`；验证用 `verify_model` / `verify_disturbance`；批量可用 `matlab -batch`。

## 关键决策

1. D 项作用在测量速度 `-Kd*x2` 上，避免阶跃参考在 `t=0` 产生微分冲激。
2. 把 I 项作为第三状态 `x3' = r - x1` 一起积分。
3. 保留 `stage1_*`、`stage2_check` 作为渐进实现历史；最终入口是 `run_comparison`。
4. 第二阶段是独立实验，不改变第一阶段的基线、参数和验证逻辑。
5. 用 `matlab -batch` 规避桌面退出时的 `0xc0000374` 堆损坏（不影响计算结果）。
6. 第三阶段执行器上限取 `u_max = 5 N`：稳态保持力 `k*r = 4 N` 的 1.25 倍，初始未饱和指令 `Kp*r = 10 N` 的 50%，保证参考可达且饱和清晰。
7. 第三阶段抗饱和采用 clamping（饱和且误差同向时冻结积分）；back-calculation 未纳入本轮主实验。控制力、饱和和积分导数统一由 `pid_control_sat.m` 实现。
8. 指标有效性只表示有限采样记录在指定误差带精度下支持位置稳态估计，不证明未来或渐近收敛。判据、原因码和运行方式见 `README.md` §4.1。
9. 时间插值不改变积分输出和有效性判据；不用增密样本替代有效性证据。`peak_time` 和 `time_method` 为新增字段，旧无速度调用使用线性插值；连续数值解 `deval` 仅作为方案对照。
10. Git 只做版本管理与公开备份，不参与仿真；本机个人文件（安装汇报、`.claude/settings.local.json`）不进入版本库，规则见 `.gitignore`。实验结果仍以 `README.md` 表格为准。

## 已知问题 / 限制

- 第三阶段只做 PID-15、常值参考、`u_max = 5 N`；无 back-calculation 对比、无 `u_max` 敏感性扫描、无 `u_max < k*r` 的不可达反例、无扰动与饱和的组合。
- G3（clamping）的超调和调节时间优于 G2，但上升比 G1 慢、调节时间和 IAE 略高于 G1；clamping 的收益是抑制 windup，不是完全恢复理想响应。
- 无噪声、非线性摩擦、参数不确定性。
- 无斜坡/正弦扰动，只做了常值扰动。
- 指标仍受积分误差、插值、窗口及相对误差带分辨率限制；基线 Refine=1 的尾段样本不足会保持无效。扰动后的独立恢复时间仍按原采样算法计算。
- 已启用 Git，公开仓库 `github.com/Matrix-2026-pde/control-pid`；历史只有初始提交，跨会话上下文仍以本文件与 `events/` 为准。
- MATLAB 桌面退出偶发 `0xc0000374`，计算本身正常。

## 下一步

1. 可选：back-calculation 与 clamping 的对比。
2. 可选：`u_max` 敏感性扫描；`u_max < k*r` 的不可达反例。
3. 可选：扰动与执行器饱和的组合实验。
4. 可选：斜坡/正弦扰动、噪声、参数摄动。

## 索引

- 详细技术记录与实验数据：`README.md`
- 当前任务：`TASK.md`
- 事件日志：`events/`
- 协作与记忆规则：`AGENTS.md`
# 2026-10-27 Simulink 饱和模型

- [x] 新增并验证 `pid_saturation_simulink.slx` 与 `build_and_verify_simulink_pid_saturation.m`。
- [x] G1/G2/G3 位置、速度、积分状态与 MATLAB 参考在 2e-3 内一致；原 `verify_saturation.m` 通过。
- [ ] G3 非光滑切换处的 `u_des/u_act` 峰值差异仍需作为数值限制保留，未扩大验收阈值。
