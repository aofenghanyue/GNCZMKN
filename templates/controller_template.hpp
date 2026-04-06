/**
 * @file my_controller.hpp
 * @brief 自定义控制算法模板
 *
 * 使用方法：
 * 1. 复制本文件到 `user/components/controller/`
 * 2. 重命名文件
 * 3. 按 `▲` 标记修改类名、参数和控制律
 * 4. 重新执行 `cmake` 与构建
 * 5. 在任务配置中把组件类型名改成你的类名
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/gnc/i_controller.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class MyController : public gnc::core::ComponentBase,
                     public gnc::interfaces::IController {
public:
    // ▲ 修改类名与默认执行频率
    MyController() : ComponentBase("MyController") {
        setExecutionFrequency(50.0);
    }

    // ▲ 在这里读取你的控制参数
    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    // 依赖注入：通常从 guidance / dynamics / nav 获取输入
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        guidance_ = registry.getByName<gnc::interfaces::IGuidance>("guidance");
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
        nav_ = registry.getByName<gnc::interfaces::INavigation>("nav");
    }

    // ▲▲▲ 在这里实现你的控制律 ▲▲▲
    void update(double dt) override {
        (void)dt;
        if (!guidance_ || !dynamics_) return;
        if (!guidance_->isActive()) return;

        const auto& guidance_command = guidance_->getGuidanceCommand();
        const gnc::Vector3d control_force = guidance_command.acceleration_cmd;
        ctrl_cmd_.force_cmd = control_force;
        ctrl_cmd_.timestamp = guidance_command.timestamp;
    }

    // 下面这些接口函数通常保持不变
    const gnc::interfaces::ControlCommand& getControlCommand() const override {
        return ctrl_cmd_;
    }

    const gnc::interfaces::ActuatorCommand& getActuatorCommand() const override {
        return act_cmd_;
    }

    bool isActive() const override {
        return true;
    }

private:
    gnc::interfaces::IGuidance* guidance_ = nullptr;
    gnc::interfaces::IDynamicsModel* dynamics_ = nullptr;
    gnc::interfaces::INavigation* nav_ = nullptr;

    // ▲ 可根据你的控制架构扩展内部状态
    gnc::interfaces::ControlCommand ctrl_cmd_;
    gnc::interfaces::ActuatorCommand act_cmd_;
};

// ▲ 可选：如果你希望框架自动记录该组件的数据，
// ▲ 可以让类额外继承 `public gnc::interfaces::IObservable`
// ▲ 并取消注释下面的方法示例。
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("force_cmd",
//         [this]() -> const gnc::Vector3d& { return ctrl_cmd_.force_cmd; });
//     builder.addVector3d("torque_cmd",
//         [this]() -> const gnc::Vector3d& { return ctrl_cmd_.torque_cmd; });
//     builder.addScalar("timestamp", [this]() { return ctrl_cmd_.timestamp; });
//     return builder.build();
// }

// ▲ 注册名必须与类名一致
GNC_REGISTER_COMPONENT(MyController, gnc::interfaces::IController)
