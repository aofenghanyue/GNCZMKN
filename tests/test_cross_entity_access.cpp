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
        registry.add<DummyPositionComponent, gnc::interfaces::IPositionProvider>("missile.dynamics", std::move(component));
    }

    {
        auto component = std::make_unique<DummyPositionComponent>(gnc::Vector3d{10.0, 20.0, 30.0});
        registry.add<DummyPositionComponent, gnc::interfaces::IPositionProvider>("target.dynamics", std::move(component));
    }

    gnc::core::ScopedRegistry scoped("missile", registry);
    auto* own = scoped.getByName<gnc::interfaces::IPositionProvider>("dynamics");
    if (!own) {
        std::cerr << "Failed to resolve in-scope position provider by short name\n";
        return 1;
    }

    const auto own_position = own->getPosition();
    if (own_position.x != 1.0 || own_position.y != 2.0 || own_position.z != 3.0) {
        std::cerr << "Resolved wrong in-scope position\n";
        return 1;
    }

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

    auto scoped_all = scoped.getAll<gnc::interfaces::IPositionProvider>();
    if (scoped_all.size() != 1) {
        std::cerr << "Scoped getAll should only return components from the same scope\n";
        return 1;
    }

    const auto scoped_position = scoped_all.front()->getPosition();
    if (scoped_position.x != 1.0 || scoped_position.y != 2.0 || scoped_position.z != 3.0) {
        std::cerr << "Scoped getAll returned the wrong component\n";
        return 1;
    }

    std::cout << "Cross-entity access works\n";
    return 0;
}
