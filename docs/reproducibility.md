# 环境、运行与测试

## 环境

本次实际运行环境为 MATLAB R2022b（9.13.0.2049777），Simulink 10.6 已安装。基础 MATLAB 实验只使用 MATLAB 内置 ODE、数值和绘图能力；Simulink 对照额外需要 Simulink。工作目录应为项目根目录 `D:\control-pid`，或在 MATLAB 中先 `cd` 到该目录。

## 入口与测试命令

```matlab
run_comparison
run_disturbance
run_saturation

results = runtests({'pid_metricsTest.m','pid_time_metricsTest.m'});
assert(all([results.Passed]) && ~any([results.Incomplete]));

stage1_verify
verify_model
stage2_check
verify_disturbance
verify_saturation
```

图由本次运行三个 `run_*` 脚本后，用 `exportgraphics(...,'results/<name>.png','Resolution',160)` 保存。PNG 只表示当前固定条件下的真实仿真输出。

## 本次验证状态

| 项目 | 本次结果 |
|---|---|
| `pid_metricsTest.m` | 28 passed / 0 failed / 0 incomplete |
| `pid_time_metricsTest.m` | 19 passed / 0 failed / 0 incomplete |
| 五个既有 `.m` 验证脚本 | 5/5 通过 |
| 三个 Simulink 构建/验证脚本 | 独立副本中 3/3 通过；模型哈希不变，详细误差见 `docs/simulink-comparison.md` |

未执行 Git commit、push 或 PR；未修改远程仓库。
