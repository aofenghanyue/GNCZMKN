#pragma once

#include "gnc/services/coordinate_tree/interfaces/i_coord_service.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gnc::services::coordinate_tree {

class CoordinateTreeService final : public ICoordService {
public:
    enum class State {
        Uninitialized,
        Building,
        Ready
    };

    CoordinateTreeService() = default;

    void beginBuild() {
        tree_.reset();
        state_ = State::Building;
    }

    void loadBuiltTree(std::shared_ptr<const internal::CoordinateTree> tree) {
        if (!tree) {
            throw std::runtime_error("CoordinateTreeService cannot load a null tree.");
        }
        tree_ = std::move(tree);
        state_ = State::Ready;
    }

    State state() const {
        return state_;
    }

    gnc::math::Vector3 transform(const gnc::math::Vector3& vector,
                                 const std::string& from_frame,
                                 const std::string& to_frame,
                                 double time) const override {
        return requireTree().transform(vector, from_frame, to_frame, time);
    }

    gnc::math::Matrix3 getRotation(const std::string& from_frame,
                                   const std::string& to_frame,
                                   double time) const override {
        return requireTree().getRotation(from_frame, to_frame, time);
    }

    bool hasFrame(const std::string& frame_id) const override {
        return requireTree().hasFrame(frame_id);
    }

    bool hasEdge(const std::string& child_frame,
                 const std::string& parent_frame) const override {
        return requireTree().hasEdge(child_frame, parent_frame);
    }

private:
    const internal::CoordinateTree& requireTree() const {
        if (state_ != State::Ready || !tree_) {
            throw std::runtime_error(
                "CoordinateTreeService is not ready; service wiring has not completed.");
        }
        return *tree_;
    }

    std::shared_ptr<const internal::CoordinateTree> tree_;
    State state_ = State::Uninitialized;
};

} // namespace gnc::services::coordinate_tree
