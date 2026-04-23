# 旧 User Components 目录

`user/components/` 是旧过渡目录，不是当前推荐的项目扩展入口。

当前推荐布局是：

```text
user/<project>/
  components/
  config/
    mission.json
```

构建系统只会自动扫描 active project：

```text
user/<active_project>/components/
```

active project 由 `user/active_project` 选择。切换 active project 或新增组件头文件后，需要重新运行 CMake 配置和构建。

如果一个组件变得稳定、通用，并且应该成为 repository-wide API，通常应移动到对应的 framework 包中：

```text
framework/include/gnc/forms/
framework/include/gnc/environment/
framework/include/gnc/vehicle/common/
framework/include/gnc/vehicle/input/
framework/include/gnc/vehicle/process/
framework/include/gnc/vehicle/output/
framework/include/gnc/interactions/
```

不要把这个目录当作长期扩展模型，也不要通过它绕过 active project 的显式注册链。
