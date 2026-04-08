# 内建组件目录说明

这个目录当前存放的是仓库随附的 starter components。

它们的作用主要是：

- 帮助默认任务和最小任务直接跑通
- 为示例和联调提供基础实现
- 为用户提供可参考的最小组件写法

不应把这里的组件默认理解为：

- 框架核心协议本身
- 所有项目都应长期依赖的领域标准件
- 比 `user/components/` 更推荐的扩展位置

## 当前建议

如果你是：

- 想运行仓库现有任务：直接使用这些组件即可
- 想做自己的项目扩展：优先在 `user/components/` 下新增组件
- 想重写某个 starter component：更推荐复制思路到 `user/components/`，而不是直接把它当作唯一官方模型继续堆逻辑

## 为什么这个目录现在还在 `framework/` 下面

这是当前仓库布局的历史结果。

在逻辑定位上，更准确的理解方式是：

- `gnc/core` 和 `gnc/interfaces` 是框架基座
- `gnc/components` 是随仓库附带的起步件集合

也就是说，这里更接近 starter kit，而不是 framework core。
