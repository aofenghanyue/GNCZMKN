#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gnc::services::coordinate_tree {

class CoordinateTreeBuilder {
public:
    using TimedRotationProvider = internal::CoordinateTree::TimedRotationProvider;

    void setRoot(const std::string& frame_id) {
        ensureMutable();
        tree_.setRoot(frame_id);
        root_set_ = true;
    }

    void addFrame(const std::string& frame_id) {
        ensureMutable();
        tree_.addFrame(frame_id);
    }

    void addStaticEdge(const std::string& child_frame,
                       const std::string& parent_frame,
                       const gnc::math::Matrix3& rotation) {
        ensureMutable();
        tree_.registerEdge(
            child_frame,
            parent_frame,
            [rotation](double) { return rotation; },
            true);
    }

    void addDynamicEdge(const std::string& child_frame,
                        const std::string& parent_frame,
                        TimedRotationProvider provider) {
        ensureMutable();
        tree_.registerEdge(child_frame, parent_frame, std::move(provider), false);
    }

    std::shared_ptr<const internal::CoordinateTree> seal() {
        ensureMutable();
        if (!root_set_ || !tree_.hasRoot()) {
            throw std::runtime_error("Coordinate tree root was not set.");
        }
        tree_.validateOrThrow();
        sealed_ = true;
        return std::make_shared<internal::CoordinateTree>(std::move(tree_));
    }

private:
    void ensureMutable() const {
        if (sealed_) {
            throw std::runtime_error("Coordinate tree builder is already sealed.");
        }
    }

    internal::CoordinateTree tree_;
    bool root_set_ = false;
    bool sealed_ = false;
};

} // namespace gnc::services::coordinate_tree
