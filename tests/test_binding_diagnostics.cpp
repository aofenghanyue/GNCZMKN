#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/state/i_altitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

#include <iostream>
#include <memory>
#include <string>

class MultiDependencyConsumer : public gnc::core::ComponentBase {
public:
    MultiDependencyConsumer() : ComponentBase("MultiDependencyConsumer") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(position_provider_, "tracker"),
            gnc::core::bind(velocity_provider_, "tracker"),
            gnc::core::bind(altitude_provider_, "altimeter"));
    }

    void update(double) override {}

private:
    gnc::interfaces::IPositionProvider* position_provider_ = nullptr;
    gnc::interfaces::IVelocityProvider* velocity_provider_ = nullptr;
    gnc::interfaces::IAltitudeProvider* altitude_provider_ = nullptr;
};

int main() {
    gnc::core::ComponentRegistry registry;
    auto consumer = std::make_unique<MultiDependencyConsumer>();
    auto* consumer_ptr = consumer.get();
    registry.addDynamic("consumer", std::move(consumer), {});

    gnc::core::ScopedRegistry::BindingDiagnostics diagnostics;
    gnc::core::ScopedRegistry scoped("", registry, "consumer", &diagnostics);
    consumer_ptr->injectDependencies(scoped);

    if (diagnostics.errors.size() != 3) {
        std::cerr << "Expected 3 aggregated required binding errors, got "
                  << diagnostics.errors.size() << "\n";
        return 1;
    }

    bool has_tracker_position = false;
    bool has_tracker_velocity = false;
    bool has_altimeter = false;
    for (const auto& error : diagnostics.errors) {
        has_tracker_position = has_tracker_position ||
                               (error.find("'tracker'") != std::string::npos &&
                                error.find("IPositionProvider") != std::string::npos);
        has_tracker_velocity = has_tracker_velocity ||
                               (error.find("'tracker'") != std::string::npos &&
                                error.find("IVelocityProvider") != std::string::npos);
        has_altimeter = has_altimeter ||
                        (error.find("'altimeter'") != std::string::npos &&
                         error.find("IAltitudeProvider") != std::string::npos);
    }

    if (!has_tracker_position || !has_tracker_velocity || !has_altimeter) {
        std::cerr << "Aggregated binding diagnostics did not preserve lookup/interface details\n";
        return 1;
    }

    std::cout << "Binding diagnostics aggregate real injectDependencies failures\n";
    return 0;
}
