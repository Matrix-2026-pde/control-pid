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
