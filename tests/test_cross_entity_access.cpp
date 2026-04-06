#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

#include <iostream>
#include <memory>

class DummyPositionComponent : public gnc::core::ComponentBase,
                               public gnc::interfaces::IPositionProvider {
public:
    explicit DummyPositionComponent(const gnc::Vector3d& position)
        : ComponentBase("DummyPositionComponent"), position_(position) {}

    void update(double) override {}

    gnc::Vector3d getPosition() const override {
        return position_;
    }

private:
    gnc::Vector3d position_;
};

int main() {
    gnc::core::ComponentRegistry registry;

    {
        auto component = std::make_unique<DummyPositionComponent>(gnc::Vector3d{1.0, 2.0, 3.0});
        registry.add("missile.dynamics", std::move(component));
    }

    {
        auto component = std::make_unique<DummyPositionComponent>(gnc::Vector3d{10.0, 20.0, 30.0});
        registry.add("target.dynamics", std::move(component));
    }

    gnc::core::ScopedRegistry scoped("missile", registry);
    auto* target = scoped.getByName<gnc::interfaces::IPositionProvider>("target.dynamics");
    if (!target) {
        std::cerr << "Failed to resolve cross-entity target position provider\n";
        return 1;
    }

    const auto position = target->getPosition();
    if (position.x != 10.0 || position.y != 20.0 || position.z != 30.0) {
        std::cerr << "Resolved wrong target position\n";
        return 1;
    }

    std::cout << "Cross-entity access works\n";
    return 0;
}
