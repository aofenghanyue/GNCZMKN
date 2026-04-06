/**
 * @file my_navigation.hpp
 * @brief 自定义导航算法模板
 *
 * 使用方法：
 * 1. 复制本文件到 `user/components/navigation/`
 * 2. 重命名文件
 * 3. 按 `▲` 标记修改类名、参数和导航解算逻辑
 * 4. 重新执行 `cmake` 与构建
 * 5. 在任务配置中把组件类型名改成你的类名
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"

class MyNavigation : public gnc::core::ComponentBase,
                     public gnc::interfaces::INavigation {
public:
    // ▲ 修改类名与默认执行频率
    MyNavigation() : ComponentBase("MyNavigation") {
        setExecutionFrequency(100.0);
    }

    // ▲ 在这里读取你的导航参数
    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    // 导航一般会依赖 IMU，也可以按需增加 GPS 等输入
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        imu_ = registry.getByName<gnc::interfaces::IImuSensor>("imu");
    }

    // ▲▲▲ 在这里实现你的导航解算逻辑 ▲▲▲
    void update(double dt) override {
        if (!imu_) return;

        const auto& imu_data = imu_->getImuData();
        nav_state_.velocity += imu_data.acceleration * dt;
        nav_state_.position += nav_state_.velocity * dt;
        nav_state_.angular_velocity = imu_data.angular_velocity;
        nav_state_.timestamp = getSimTime();
    }

    // 下面这些接口函数通常保持不变
    const gnc::interfaces::NavState& getNavState() const override {
        return nav_state_;
    }

    double getTimestamp() const override {
        return nav_state_.timestamp;
    }

    bool isValid() const override {
        return true;
    }

private:
    gnc::interfaces::IImuSensor* imu_ = nullptr;

    // ▲ 可根据滤波器或解算器需要扩展内部状态
    gnc::interfaces::NavState nav_state_;
};

// ▲ 可选：如果你希望框架自动记录该组件的数据，
// ▲ 可以让类额外继承 `public gnc::interfaces::IObservable`
// ▲ 并取消注释下面的方法示例。
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("position",
//         [this]() -> const gnc::Vector3d& { return nav_state_.position; });
//     builder.addVector3d("velocity",
//         [this]() -> const gnc::Vector3d& { return nav_state_.velocity; });
//     builder.addScalar("timestamp", [this]() { return nav_state_.timestamp; });
//     return builder.build();
// }

// ▲ 注册名必须与类名一致
GNC_REGISTER_COMPONENT(MyNavigation, gnc::interfaces::INavigation)
