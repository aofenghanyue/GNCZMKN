#include "test_support.hpp"

#define GNC_COMPONENT_REGISTRATION_FN gnc_register_cavh_programmed_aoa_guidance_test
#include "user/example_08_cavh_geographic_3dof_custom/components/cavh_programmed_aoa_guidance.hpp"
#undef GNC_COMPONENT_REGISTRATION_FN

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {

class StubNavigation final
    : public gnc::core::ComponentBase,
      public gnc::vehicle::process::INavigation3Dof {
public:
    StubNavigation() : ComponentBase("StubNavigation") {}

    void setMachNumber(double mach_number) {
        state_.mach_number = mach_number;
    }

    void update(double) override {}

    const gnc::vehicle::process::NavigationState3Dof& navigationState3Dof()
        const override {
        return state_;
    }

private:
    gnc::vehicle::process::NavigationState3Dof state_{};
};

gnc::core::ConfigNode validGuidanceConfig() {
    using namespace test_support;
    return object({
        field("navigation_lookup_name", string("navigation")),
        field("bank_angle_deg", number(5.0)),
        field("schedule_mach_number",
              array({number(2.0), number(4.0), number(6.0)})),
        field("schedule_angle_of_attack_deg",
              array({number(20.0), number(10.0), number(0.0)})),
    });
}

gnc::core::ConfigNode mismatchedScheduleConfig() {
    using namespace test_support;
    return object({
        field("bank_angle_deg", number(0.0)),
        field("schedule_mach_number", array({number(2.0), number(4.0)})),
        field("schedule_angle_of_attack_deg", array({number(20.0)})),
    });
}

double observableValue(const gnc::interfaces::IObservable& observable,
                       const std::string& field_name) {
    for (const auto& field : observable.getObservableFields()) {
        if (field.name == field_name) {
            return field.getter();
        }
    }
    throw std::runtime_error("Observable field not found: " + field_name);
}

void requireCleanScheduleDiagnostic() {
    cavh::components::ProgrammedAoAGuidance guidance;

    bool failed = false;
    std::string message;
    try {
        guidance.configure(mismatchedScheduleConfig(),
                           "vehicles[0].process[4].config");
    } catch (const std::exception& ex) {
        failed = true;
        message = ex.what();
    }

    test_support::require(failed,
                          "Mismatched guidance schedule arrays should fail.");
    test_support::require(
        message.find("schedule_mach_number") != std::string::npos,
        "Mach-scheduled guidance diagnostic should name schedule_mach_number.");
    test_support::require(
        message.find("schedule_altitude_m") == std::string::npos,
        "Mach-scheduled guidance diagnostic should not mention stale altitude keys.");
}

void requireMachScheduleInterpolation() {
    gnc::core::ComponentRegistry registry;
    auto navigation = std::make_unique<StubNavigation>();
    auto* navigation_ptr = navigation.get();
    navigation_ptr->setMachNumber(3.0);
    registry.add<StubNavigation, gnc::vehicle::process::INavigation3Dof>(
        "vehicle.navigation",
        std::move(navigation));

    cavh::components::ProgrammedAoAGuidance guidance;
    guidance.configure(validGuidanceConfig(), "vehicles[0].process[4].config");

    gnc::core::ScopedRegistry scoped("vehicle", registry, "vehicle.guidance");
    guidance.injectDependencies(scoped);
    guidance.update(0.1);

    test_support::requireNear(
        gnc::math::rad2deg(guidance.guidanceCommand3Dof().angle_of_attack_rad),
        15.0,
        1e-12,
        "Guidance should interpolate angle of attack from mach number.");
    test_support::requireNear(
        gnc::math::rad2deg(guidance.guidanceCommand3Dof().bank_angle_rad),
        5.0,
        1e-12,
        "Guidance should preserve configured bank angle.");
    test_support::requireNear(observableValue(guidance, "mach_number"),
                              3.0,
                              1e-12,
                              "Guidance observable should publish mach number.");

    navigation_ptr->setMachNumber(5.0);
    guidance.update(0.1);
    test_support::requireNear(
        gnc::math::rad2deg(guidance.guidanceCommand3Dof().angle_of_attack_rad),
        5.0,
        1e-12,
        "Guidance should update interpolation when mach number changes.");
}

} // namespace

int main() {
    try {
        requireCleanScheduleDiagnostic();
        requireMachScheduleInterpolation();

        std::cout << "CAVH programmed AoA guidance checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
