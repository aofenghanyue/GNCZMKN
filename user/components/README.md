# legacy 组件目录

`user/components/` 已降级为过渡目录。

推荐结构：

- `user/<project>/components/`
- `user/<project>/config/mission.json`
- `user/active_project`

当前构建流程只会扫描活跃工程的 `components/` 目录。
如果某个组件已经成为跨项目复用、语义稳定的仓库公共组件，应考虑沉淀到 `framework/include/gnc/components/`，而不是继续堆在这个 legacy 目录里。

这个目录只建议用于尚未迁移进具体工程目录的过渡文件。
