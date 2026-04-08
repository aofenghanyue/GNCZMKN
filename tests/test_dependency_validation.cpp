#include "gnc/core/component_registry.hpp"
#include "gnc/core/dependency_validator.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

#include <iostream>
#include <memory>
#include <vector>

class DummyPositionProvider : public gnc::core::ComponentBase,
                              public gnc::interfaces::IPositionProvider {
public:
    DummyPositionProvider() : ComponentBase("DummyPositionProvider") {}

    void update(double) override {}

    gnc::Vector3d getPosition() const override {
        return gnc::Vector3d{1.0, 2.0, 3.0};
    }
};

class DependencyConsumer : public gnc::core::ComponentBase,
                           public gnc::core::IDependencyDeclarer {
public:
    explicit DependencyConsumer(std::vector<gnc::core::DependencyDeclaration> deps)
        : ComponentBase("DependencyConsumer"), deps_(std::move(deps)) {}

    void update(double) override {}

    std::vector<gnc::core::DependencyDeclaration> getDependencies() const override {
        return deps_;
    }

private:
    std::vector<gnc::core::DependencyDeclaration> deps_;
};

int main() {
    {
        gnc::core::ComponentRegistry registry;

        auto provider = std::make_unique<DummyPositionProvider>();
        registry.add<DummyPositionProvider, gnc::interfaces::IPositionProvider>(
            "target.tracker", std::move(provider));

        auto consumer = std::make_unique<DependencyConsumer>(
            std::vector<gnc::core::DependencyDeclaration>{
                gnc::core::requireDependency<gnc::interfaces::IPositionProvider>(
                    "tracker", "an in-scope tracker position provider")
            });
        registry.add<DependencyConsumer>("missile.seeker", std::move(consumer));

        const auto result = gnc::core::DependencyValidator::validate(registry);
        if (result.success || result.errors.size() != 1) {
            std::cerr << "Expected one scoped dependency validation error\n";
            return 1;
        }

        const auto& message = result.errors.front();
        if (message.find("resolved as 'missile.tracker'") == std::string::npos) {
            std::cerr << "Validation message did not include resolved scoped name\n";
            return 1;
        }
        if (message.find("target.tracker") == std::string::npos) {
            std::cerr << "Validation message did not include registered provider candidates\n";
            return 1;
        }
    }

    {
        gnc::core::ComponentRegistry registry;

        auto provider = std::make_unique<DummyPositionProvider>();
        registry.add<DummyPositionProvider, gnc::interfaces::IPositionProvider>(
            "target.tracker", std::move(provider));

        auto consumer = std::make_unique<DependencyConsumer>(
            std::vector<gnc::core::DependencyDeclaration>{
                gnc::core::requireDependency<gnc::interfaces::IPositionProvider>(
                    "target.tracker", "an explicit cross-entity tracker position provider")
            });
        registry.add<DependencyConsumer>("missile.ideal_seeker", std::move(consumer));

        const auto result = gnc::core::DependencyValidator::validate(registry);
        if (!result.success || !result.errors.empty()) {
            std::cerr << "Explicit cross-entity lookup should pass dependency validation\n";
            return 1;
        }
    }

    std::cout << "Dependency validation works\n";
    return 0;
}
