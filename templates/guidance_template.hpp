/**
 * @file my_guidance.hpp
 * @brief 自定义制导算法模板
 *
 * 使用方法：
 * 1. 复制本文件到 `user/components/guidance/`
 * 2. 重命名文件
 * 3. 按 `▲` 标记修改类名、参数和算法实现
 * 4. 重新执行 `cmake` 与构建
 * 5. 在任务配置中把组件类型名改成你的类名
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/data_types.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class MyGuidance : public gnc::core::ComponentBase,
                   public gnc::interfaces::IGuidance {
public:
    // ▲ 修改类名与默认执行频率
    MyGuidance() : ComponentBase("MyGuidance") {
        setExecutionFrequency(50.0);
    }

    // ▲ 在这里读取你的算法参数
    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    // 依赖注入通常只需要按名称获取已有组件
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        nav_ = registry.getByName<gnc::interfaces::INavigation>("nav");
    }

    // ▲▲▲ 在这里实现你的制导算法核心逻辑 ▲▲▲
    void update(double dt) override {
        (void)dt;
        if (!nav_ || !nav_->isValid()) return;

        const auto& state = nav_->getNavState();
        cmd_.acceleration_cmd = gnc::Vector3d{0.0, 0.0, 0.0};
        cmd_.timestamp = state.timestamp;
    }

    // 下面这些接口函数通常保持不变，只需按需补业务逻辑
    const gnc::interfaces::GuidanceCommand& getGuidanceCommand() const override {
        return cmd_;
    }

    // ▲ 如果你的算法需要目标点，可在这里保存外部传入目标
    void setTarget(const gnc::Vector3d& target) override {
        target_ = target;
    }

    bool isActive() const override {
        return true;
    }

private:
    gnc::interfaces::INavigation* nav_ = nullptr;
    gnc::interfaces::GuidanceCommand cmd_;

    // ▲ 可替换为你的目标、状态或内部参数缓存
    gnc::Vector3d target_;
};

// ▲ 可选：如果你希望框架自动记录该组件的数据，
// ▲ 可以让类额外继承 `public gnc::interfaces::IObservable`
// ▲ 并取消注释下面的方法示例。
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("acceleration_cmd",
//         [this]() -> const gnc::Vector3d& { return cmd_.acceleration_cmd; });
//     builder.addScalar("timestamp", [this]() { return cmd_.timestamp; });
//     return builder.build();
// }

// ▲ 注册名必须与类名一致
GNC_REGISTER_COMPONENT(MyGuidance, gnc::interfaces::IGuidance)
