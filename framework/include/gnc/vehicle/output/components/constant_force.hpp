#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"

#include <string>
#include <vector>

namespace gnc::vehicle::output {

class ConstantForce final : public gnc::core::ComponentBase,
                            public IForceProvider,
                            public gnc::interfaces::IObservable {
public:
    ConstantForce() : ComponentBase("ConstantForce") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto force = reader.requiredDoubleArray("force_n", 3);
        force_n_ = gnc::math::Vector3(force[0], force[1], force[2]);
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::math::Vector3 getForceN() const override { return force_n_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("force_n", [this]() -> const gnc::math::Vector3& {
            return force_n_;
        });
        return builder.build();
    }

private:
    gnc::math::Vector3 force_n_ = gnc::math::Vector3::Zero();
};

} // namespace gnc::vehicle::output
