#include "test_support.hpp"

#include "gnc/services/coordinate_tree/components/coordinate_tree_builder.hpp"

#include <exception>
#include <functional>
#include <iostream>
#include <string>

namespace {

void requireFailure(const std::function<void()>& action,
                    const std::string& expected_fragment,
                    const std::string& message) {
    bool failed = false;
    try {
        action();
    } catch (const std::exception& ex) {
        failed = std::string(ex.what()).find(expected_fragment) != std::string::npos;
    }
    test_support::require(failed, message);
}

} // namespace

int main() {
    try {
        using gnc::services::coordinate_tree::CoordinateTreeBuilder;

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                (void)builder.seal();
            },
            "root was not set",
            "CoordinateTreeBuilder should reject seal() before a root is set.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("E");
                builder.addFrame("E");
            },
            "already registered",
            "CoordinateTreeBuilder should reject duplicate frame ids.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("N");
                builder.addDynamicEdge(
                    "N",
                    "E",
                    [](double) { return gnc::math::Matrix3::Identity(); });
            },
            "Parent coordinate system 'E' is not registered",
            "CoordinateTreeBuilder should reject edges whose parent frame is missing.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("E");
                builder.addFrame("N");
                builder.addStaticEdge("E", "I", gnc::math::Matrix3::Identity());
                builder.addStaticEdge("E", "N", gnc::math::Matrix3::Identity());
            },
            "already has a parent",
            "CoordinateTreeBuilder should reject a second parent edge for the same frame.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("E");
                builder.addStaticEdge("I", "E", gnc::math::Matrix3::Identity());
                (void)builder.seal();
            },
            "cannot have a parent",
            "CoordinateTreeBuilder should reject trees whose root frame also has a parent.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("A");
                builder.addFrame("B");
                builder.addFrame("C");
                builder.addStaticEdge("A", "B", gnc::math::Matrix3::Identity());
                builder.addStaticEdge("B", "C", gnc::math::Matrix3::Identity());
                builder.addStaticEdge("C", "A", gnc::math::Matrix3::Identity());
                (void)builder.seal();
            },
            "contains a cycle",
            "CoordinateTreeBuilder should reject cyclic frame graphs at seal time.");

        requireFailure(
            []() {
                CoordinateTreeBuilder builder;
                builder.setRoot("I");
                builder.addFrame("A");
                builder.addFrame("B");
                builder.addStaticEdge("A", "B", gnc::math::Matrix3::Identity());
                (void)builder.seal();
            },
            "does not trace to root",
            "CoordinateTreeBuilder should reject disconnected frame chains that do not reach the declared root.");

        std::cout << "coordinate tree builder checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
