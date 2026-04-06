# Volume 11: 教程 - 从零编写自定义控制器并运行

本章将通过实际的代码和终端命令，带您体验“不改框架一行核心代码，完成自定义组件接入并运行”的全过程。

## 目标
我们要在 `user/components/` 目录下编写一个简单的控制器 `my_controller.hpp`，通过 JSON 配置启用它，并运行主程序 `gnc_sim` 进行验证。

## 步骤 1: 创建组件文件
请确保您在项目的根目录下。使用您喜欢的编辑器（如 `vim` 或 `nano`），创建文件 `user/components/my_controller.hpp`：
```bash
mkdir -p user/components
touch user/components/my_controller.hpp
```

将以下代码粘贴进 `my_controller.hpp`：
```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/gnc/i_controller.hpp"
#include "gnc/common/logger.hpp"

class MyController : public gnc::core::ComponentBase,
                     public gnc::interfaces::IController {
public:
    MyController() : ComponentBase("MyController") {}

    void configure(const gnc::core::ConfigNode& config) override {
        // 从 JSON 读取参数
        kp_ = config["kp"].asDouble(1.0);
        LOG_INFO("MyController configured with kp = {}", kp_);
    }

    void update(double dt) override {
        // 极简的控制逻辑示例
        cmd_.torque_cmd = gnc::Vector3d(kp_ * dt, 0, 0);
        cmd_.timestamp = getSimTime();
    }

    const gnc::interfaces::ControlCommand& getControlCommand() const override {
        return cmd_;
    }

    const gnc::interfaces::ActuatorCommand& getActuatorCommand() const override {
        return act_cmd_;
    }

    bool isActive() const override { return true; }

private:
    gnc::interfaces::ControlCommand cmd_;
    gnc::interfaces::ActuatorCommand act_cmd_;
    double kp_ = 1.0;
};

// 关键：将组件注册到工厂
GNC_REGISTER_COMPONENT(MyController, gnc::interfaces::IController)
```

## 步骤 2: 重新编译
因为我们在 `user/components/` 目录下新增了 `.hpp` 文件，我们需要让 CMake 重新扫描并生成自动注册代码。
在项目根目录执行：
```bash
cd build
cmake ..
cmake --build . --config Release
cd ..
```

## 步骤 3: 验证组件是否成功注册
框架的主程序 `gnc_sim` 提供了一个检查命令。执行：
```bash
./build/bin/gnc_sim --list-components
```
在终端输出的列表中，您应该能找到：
```text
  - MyController
```
这证明您的组件已经被系统动态识别了！

## 步骤 4: 编写 JSON 配置文件
在 `user/config/missions/` 目录下创建一个测试配置文件 `test_mission.json`：
```bash
mkdir -p user/config/missions
touch user/config/missions/test_mission.json
```

填入以下内容：
```json
{
    "simulation": {
        "dt": 0.1,
        "duration": 5.0
    },
    "components": [
        {
            "type": "MyController",
            "name": "roll_controller",
            "config": {
                "kp": 3.14
            }
        }
    ]
}
```

## 步骤 5: 运行仿真
将这个 JSON 文件传给 `gnc_sim` 运行器：
```bash
./build/bin/gnc_sim user/config/missions/test_mission.json
```

**预期的终端输出：**
```text
[INFO] === GNC Simulation Framework v2.0 ===
[INFO] Config: user/config/missions/test_mission.json
[INFO] Simulation config: dt=0.1, duration=5
[INFO] MyController configured with kp = 3.14
[INFO] Initializing simulator...
[INFO] Initializing component: roll_controller
[INFO] Starting simulation: dt=0.1, duration=5
[INFO] Simulation completed. Final time: 5.0, reason: completed
```

**成功！** 
您刚才体验了完整的开发闭环。通过这种模式，您的算法开发完全局限于 `user/components/` 目录下，实现了真正的算法与底层引擎隔离。