# 用户工程区

`user/` 是统一后的唯一工程区。

- `user/active_project` 保存当前活跃工程名
- `user/<project>/components/` 保存该工程的自定义组件
- `user/<project>/config/mission.json` 保存该工程的默认任务
- `user/outputs/` 保存仿真输出
- `user/config/missions/default.json` 作为 legacy 兜底任务保留

当前仓库内置的 starter projects：

- `user/example_01_minimal`
- `user/example_02_gravity_turn`
- `user/example_03_cavh_3dof`

其中 `user/example_02_gravity_turn` 是独立程序式样板，不走 `user/active_project + gnc_sim` 这条统一入口。
