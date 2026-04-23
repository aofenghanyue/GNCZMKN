#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::services::coordinate_tree::internal {

class CoordinateTree {
public:
    using TimedRotationProvider = std::function<gnc::math::Matrix3(double)>;

    void setRoot(const std::string& root_frame) {
        if (root_frame.empty()) {
            throw std::runtime_error("Coordinate tree root frame cannot be empty.");
        }
        nodes_.clear();
        edge_index_.clear();
        root_frame_ = root_frame;
        nodes_.emplace(root_frame_, Node{});
    }

    void addFrame(const std::string& frame_id) {
        if (frame_id.empty()) {
            throw std::runtime_error("Coordinate tree frame id cannot be empty.");
        }
        if (nodes_.count(frame_id) > 0) {
            throw std::runtime_error("Coordinate tree frame '" + frame_id +
                                     "' is already registered.");
        }
        nodes_.emplace(frame_id, Node{});
    }

    void registerEdge(const std::string& child_frame,
                      const std::string& parent_frame,
                      TimedRotationProvider provider,
                      bool time_invariant) {
        if (child_frame.empty() || parent_frame.empty()) {
            throw std::runtime_error("Coordinate tree edge endpoints cannot be empty.");
        }
        if (child_frame == parent_frame) {
            throw std::runtime_error("Coordinate tree cannot create a self-parent edge.");
        }

        const auto parent_it = nodes_.find(parent_frame);
        if (parent_it == nodes_.end()) {
            throw std::runtime_error("Parent coordinate system '" + parent_frame +
                                     "' is not registered.");
        }
        const auto child_it = nodes_.find(child_frame);
        if (child_it == nodes_.end()) {
            throw std::runtime_error("Child coordinate system '" + child_frame +
                                     "' is not registered.");
        }
        if (!provider) {
            throw std::runtime_error("Coordinate tree edge '" + child_frame + " -> " +
                                     parent_frame + "' has no rotation provider.");
        }

        Node& node = child_it->second;
        if (node.has_parent) {
            throw std::runtime_error("Coordinate system '" + child_frame +
                                     "' already has a parent.");
        }

        node.has_parent = true;
        node.parent = parent_frame;
        node.provider = std::move(provider);
        node.time_invariant = time_invariant;
        node.cache_valid = false;
        edge_index_[child_frame] = parent_frame;
    }

    bool hasRoot() const {
        return !root_frame_.empty();
    }

    bool hasFrame(const std::string& frame_id) const {
        return nodes_.count(frame_id) > 0;
    }

    bool hasEdge(const std::string& child_frame, const std::string& parent_frame) const {
        const auto it = edge_index_.find(child_frame);
        return it != edge_index_.end() && it->second == parent_frame;
    }

    const std::string& rootFrame() const {
        return root_frame_;
    }

    void validateOrThrow() const {
        if (!hasRoot()) {
            throw std::runtime_error("Coordinate tree root was not set.");
        }

        const auto root_it = nodes_.find(root_frame_);
        if (root_it == nodes_.end()) {
            throw std::runtime_error("Coordinate tree root '" + root_frame_ +
                                     "' is not registered.");
        }
        if (root_it->second.has_parent) {
            throw std::runtime_error("Coordinate tree root '" + root_frame_ +
                                     "' cannot have a parent.");
        }

        for (const auto& [frame_id, _] : nodes_) {
            const auto lineage = lineageToRoot(frame_id);
            if (lineage.empty() || lineage.back() != root_frame_) {
                const std::string terminal_frame =
                    lineage.empty() ? std::string("(unknown)") : lineage.back();
                throw std::runtime_error("Coordinate tree frame '" + frame_id +
                                         "' does not trace to root '" + root_frame_ +
                                         "'; it terminates at disconnected frame '" +
                                         terminal_frame + "'.");
            }
        }
    }

    gnc::math::Matrix3 getRotation(const std::string& from_frame,
                                   const std::string& to_frame,
                                   double time) const {
        if (from_frame == to_frame) {
            return gnc::math::Matrix3::Identity();
        }

        const auto [path_up, path_down] = findPathViaLca(from_frame, to_frame);
        gnc::math::Matrix3 rotation = gnc::math::Matrix3::Identity();
        for (const auto& frame_id : path_up) {
            rotation = getToParent(frame_id, time) * rotation;
        }
        for (auto it = path_down.rbegin(); it != path_down.rend(); ++it) {
            rotation = getToParent(*it, time).transpose() * rotation;
        }
        return rotation;
    }

    gnc::math::Vector3 transform(const gnc::math::Vector3& vector,
                                 const std::string& from_frame,
                                 const std::string& to_frame,
                                 double time) const {
        return getRotation(from_frame, to_frame, time) * vector;
    }

private:
    struct Node {
        bool has_parent = false;
        bool time_invariant = true;
        mutable bool cache_valid = false;
        std::string parent;
        TimedRotationProvider provider;
        mutable gnc::math::Matrix3 cached_rotation = gnc::math::Matrix3::Identity();
        mutable double cache_time = 0.0;
    };

    gnc::math::Matrix3 getToParent(const std::string& frame_id, double time) const {
        auto it = nodes_.find(frame_id);
        if (it == nodes_.end() || !it->second.has_parent) {
            throw std::runtime_error("Coordinate system '" + frame_id +
                                     "' has no parent.");
        }

        Node& node = const_cast<Node&>(it->second);
        if (node.time_invariant && node.cache_valid) {
            return node.cached_rotation;
        }
        if (!node.time_invariant && node.cache_valid && node.cache_time == time) {
            return node.cached_rotation;
        }

        node.cached_rotation = node.provider(time);
        node.cache_time = time;
        node.cache_valid = true;
        return node.cached_rotation;
    }

    std::vector<std::string> lineageToRoot(const std::string& frame_id) const {
        std::vector<std::string> lineage;
        std::unordered_map<std::string, bool> visited;
        std::string current = frame_id;
        while (true) {
            if (visited.count(current) > 0) {
                throw std::runtime_error("Coordinate tree contains a cycle involving frame '" +
                                         current + "'.");
            }
            visited.emplace(current, true);

            const auto it = nodes_.find(current);
            if (it == nodes_.end()) {
                throw std::runtime_error("Coordinate system '" + current +
                                         "' is not part of the tree.");
            }
            lineage.push_back(current);
            if (!it->second.has_parent) {
                break;
            }
            current = it->second.parent;
        }
        return lineage;
    }

    std::pair<std::vector<std::string>, std::vector<std::string>> findPathViaLca(
        const std::string& from_frame,
        const std::string& to_frame) const {
        const auto from_lineage = lineageToRoot(from_frame);
        const auto to_lineage = lineageToRoot(to_frame);

        std::unordered_map<std::string, size_t> from_index;
        for (size_t i = 0; i < from_lineage.size(); ++i) {
            from_index.emplace(from_lineage[i], i);
        }

        size_t lca_from_index = 0;
        size_t to_index = 0;
        bool found = false;
        for (; to_index < to_lineage.size(); ++to_index) {
            const auto it = from_index.find(to_lineage[to_index]);
            if (it != from_index.end()) {
                lca_from_index = it->second;
                found = true;
                break;
            }
        }

        if (!found) {
            throw std::runtime_error("Coordinate systems '" + from_frame + "' and '" +
                                     to_frame + "' do not share a root.");
        }

        std::vector<std::string> path_up(from_lineage.begin(),
                                         from_lineage.begin() + lca_from_index);
        std::vector<std::string> path_down(to_lineage.begin(),
                                           to_lineage.begin() + to_index);
        return {path_up, path_down};
    }

    std::string root_frame_;
    mutable std::unordered_map<std::string, Node> nodes_;
    std::unordered_map<std::string, std::string> edge_index_;
};

} // namespace gnc::services::coordinate_tree::internal
