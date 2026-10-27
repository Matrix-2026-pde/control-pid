# 2026-10-27 项目展示版整理

## 背景
将 control-pid 整理为适合 GitHub 作品集和远程工程项目申请的本地展示版本，同时区分基础 MATLAB 实验与 Simulink 对照路径。

## 实际操作
保存原始 README，重写首页并新增模型/指标/Simulink/复现文档；运行三个 `run_*` 入口导出三张 PNG；执行两组 MATLAB 测试和五个既有验证脚本。

## 涉及文件
`README.md`、`AGENTS.md`、`TASK.md`、`docs/`、`results/`、本事件记录。

## 验证结果
MATLAB R2022b：测试 47/47 通过；五个既有验证脚本通过。三个 Simulink 构建/验证脚本本次未运行。

## 下一步
由用户决定是否在独立副本中运行 Simulink 构建/验证脚本，并确认个人与 AI Agent 的贡献归属文案。
