# Volume 5: 数据记录与可观测性 (Data & Observability)

## 1. 传统日志的问题与解决
在很多仿真项目中，算法工程师习惯在代码中插入 `fprintf` 或 `csv << value`。这带来的问题是：
- 污染了纯粹的算法代码。
- 如果只想查看某些特定的变量，需要重新编译。
- 落盘性能差，容易成为仿真的性能瓶颈。

本框架通过 `IObservable` 接口与 `AutoDataLogger` 彻底解决了这一痛点。

## 2. IObservable 接口：暴露内部状态
任何希望被记录的组件，只需实现 `gnc::interfaces::IObservable` 接口。该接口要求返回一个 `std::vector<ObservableField>` 数组，其中包含字段名和一个返回 `double` 的 Lambda `getter`。

```cpp
std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
    gnc::core::ObservableFieldBuilder builder;
    
    // 暴露一个简单变量
    builder.addScalar("CL", [this]() { return currentCoefficients().CL; });
    
    // 暴露一个计算值（延迟求值，零额外开销）
    builder.addScalar("lift_to_drag", [this]() {
        const auto coeffs = currentCoefficients();
        return std::abs(coeffs.CD) > 1e-9 ? coeffs.CL / coeffs.CD : 0.0;
    });
    
    return builder.build();
}
```

## 3. AutoDataLogger 机制：自动化的高性能 CSV 记录
`AutoDataLogger` 位于 `gnc/core/auto_data_logger.hpp` 中。它在仿真初始化阶段介入，根据 JSON 配置决定哪些组件和字段需要被记录。

### 3.1 字段发现 (Discover Fields)
- **如果配置了 `"record": "all"`**：`AutoDataLogger` 会遍历所有实现了 `IObservable` 的组件，提取所有的 getter 函数，并将它们推入一个连续的函数指针数组（`std::vector<ActiveField>`）。
- **如果配置了对象形式**：它只会收集指定组件（如 `"dynamics"`）的指定字段（如 `["altitude", "velocity"]`）。这极大地减少了内存分配和 CSV 文件体积。

### 3.2 运行时记录 (Record Step)
在 `Simulator::run()` 的主循环中，每一步的结尾会调用 `auto_logger_.recordStep(current_time_)`。
1. `AutoDataLogger` 清空 `row_buffer_`（预分配的 `std::vector<double>`）。
2. 将当前时间压入 buffer。
3. 顺序执行所有预先收集好的 Lambda `getter` 函数，将返回值压入 buffer。
4. 调用 `sink_->writeRow(row_buffer_)`，将其委托给底层的 `CsvRecordSink`。

### 3.3 CsvRecordSink：高效落盘
在 `csv_record_sink.hpp` 中，我们并没有每步直接 `std::ofstream <<`。
- **高精度**：默认设置 `precision=12`。
- **缓冲机制**：利用 C++ 自身的缓冲，且可通过 `flush_every_step` 配置是否每步强行刷盘。

## 4. 自动化 Python 绘图工具
配合自动日志，框架自带了一个通用绘图工具 `tools/plot_results.py`。
- **纯文本摘要**：即使您的环境中没有 `matplotlib`，它也会打印 CSV 中的最小值、最大值和最终值，非常适合 CI/CD 或服务器环境。
- **2x2 视图**：如果您安装了 `matplotlib`，它将自动生成轨迹图 (X-Z)、速度随时间变化、位置随时间变化以及指令图，极大地加速了调参周期。