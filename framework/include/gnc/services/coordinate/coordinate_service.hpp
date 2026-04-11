/**
 * @file coordinate_service.hpp
 * @brief Generic rotation-tree coordinate service.
 */
#pragma once

#include "gnc/common/math/math.hpp"
#include "gnc/interfaces/coord/frame_id.hpp"
#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace gnc::services {

using gnc::coord::FrameId;
using gnc::coord::frameIdToString;
using gnc::math::Matrix3;
using gnc::math::Vector3;

class CoordinateService {
public:
    using RotationProvider = std::function<Matrix3()>;
    using TimedRotationProvider = std::function<Matrix3(double)>;

    CoordinateService() {
        initializeRoot(FrameId::ECI);
    }

    ~CoordinateService() = default;

    void setRoot(FrameId root) {
        initializeRoot(root);
    }

    void registerTransform(FrameId child, FrameId parent, RotationProvider childToParent) {
        registerTransformInternal(
            child,
            parent,
            [provider = std::move(childToParent)](double) { return provider(); },
            true);
    }

    void registerTransform(FrameId child, FrameId parent, TimedRotationProvider childToParent) {
        registerTransformInternal(child, parent, std::move(childToParent), false);
    }

    bool hasFrame(FrameId frame) const {
        return nodes_.find(frame) != nodes_.end();
    }

    Matrix3 getRotation(FrameId from, FrameId to, double time = 0.0) const {
        if (from == to) {
            return Matrix3::Identity();
        }

        const auto [pathUp, pathDown] = findPathViaLCA(from, to);

        Matrix3 result = Matrix3::Identity();
        for (FrameId frame : pathUp) {
            result = getToParent(frame, time) * result;
        }
        for (auto it = pathDown.rbegin(); it != pathDown.rend(); ++it) {
            result = getToParent(*it, time).transpose() * result;
        }

        return result;
    }

    Vector3 transform(const Vector3& v, FrameId from, FrameId to, double time = 0.0) const {
        return getRotation(from, to, time) * v;
    }

    void clear() {
        initializeRoot(FrameId::ECI);
    }

    void clearCache() const {
        for (auto& [_, node] : nodes_) {
            node.cacheValid = false;
            node.cacheTime = 0.0;
        }
    }

    std::string dumpTopology() const {
        std::ostringstream oss;
        oss << "=== Coordinate Tree (Root: " << frameIdToString(root_) << ") ===\n";
        for (const auto& [frame, node] : nodes_) {
            if (node.hasParent) {
                oss << "  " << frameIdToString(frame)
                    << " -> " << frameIdToString(node.parent) << "\n";
            }
        }
        oss << "=== Total Nodes: " << nodes_.size() << " ===\n";
        return oss.str();
    }

private:
    struct Node {
        bool hasParent = false;
        bool timeInvariant = true;
        bool cacheValid = false;
        FrameId parent = FrameId::ECI;
        TimedRotationProvider toParent;
        mutable Matrix3 cachedRotation = Matrix3::Identity();
        mutable double cacheTime = 0.0;
    };

    void initializeRoot(FrameId root) {
        nodes_.clear();
        root_ = root;
        nodes_.emplace(root_, Node{});
    }

    void registerTransformInternal(FrameId child,
                                   FrameId parent,
                                   TimedRotationProvider childToParent,
                                   bool timeInvariant) {
        if (child == parent) {
            throw std::runtime_error("Coordinate tree cannot self-parent frame '" +
                                     std::string(frameIdToString(child)) + "'.");
        }
        if (child == root_) {
            throw std::runtime_error("Root frame '" +
                                     std::string(frameIdToString(root_)) +
                                     "' cannot be assigned a parent.");
        }

        const auto parentIt = nodes_.find(parent);
        if (parentIt == nodes_.end()) {
            throw std::runtime_error("Parent frame not registered: " +
                                     std::string(frameIdToString(parent)));
        }

        for (FrameId current = parent; ; ) {
            if (current == child) {
                throw std::runtime_error("Registering '" +
                                         std::string(frameIdToString(child)) +
                                         "' under '" +
                                         std::string(frameIdToString(parent)) +
                                         "' would create a cycle.");
            }

            const auto it = nodes_.find(current);
            if (it == nodes_.end() || !it->second.hasParent) {
                break;
            }
            current = it->second.parent;
        }

        auto existing = nodes_.find(child);
        if (existing != nodes_.end() && existing->second.hasParent) {
            throw std::runtime_error("Frame already has a parent: " +
                                     std::string(frameIdToString(child)));
        }

        Node& node = nodes_[child];
        node.hasParent = true;
        node.parent = parent;
        node.timeInvariant = timeInvariant;
        node.toParent = std::move(childToParent);
        node.cacheValid = false;
        node.cacheTime = 0.0;
        node.cachedRotation = Matrix3::Identity();
    }

    Matrix3 getToParent(FrameId frame, double time) const {
        auto it = nodes_.find(frame);
        if (it == nodes_.end() || !it->second.hasParent) {
            throw std::runtime_error("Frame has no parent: " +
                                     std::string(frameIdToString(frame)));
        }

        Node& node = const_cast<Node&>(it->second);
        if (node.timeInvariant && node.cacheValid) {
            return node.cachedRotation;
        }
        if (!node.timeInvariant && node.cacheValid && node.cacheTime == time) {
            return node.cachedRotation;
        }

        node.cachedRotation = node.toParent(time);
        node.cacheTime = time;
        node.cacheValid = true;
        return node.cachedRotation;
    }

    std::vector<FrameId> getLineageToRoot(FrameId frame) const {
        std::vector<FrameId> lineage;
        FrameId current = frame;

        while (true) {
            const auto it = nodes_.find(current);
            if (it == nodes_.end()) {
                throw std::runtime_error("Frame not in tree: " +
                                         std::string(frameIdToString(current)));
            }

            lineage.push_back(current);
            if (!it->second.hasParent) {
                break;
            }
            current = it->second.parent;
        }

        return lineage;
    }

    std::pair<std::vector<FrameId>, std::vector<FrameId>>
    findPathViaLCA(FrameId from, FrameId to) const {
        const auto fromLineage = getLineageToRoot(from);
        const auto toLineage = getLineageToRoot(to);

        std::unordered_map<FrameId, size_t> fromIndex;
        fromIndex.reserve(fromLineage.size());
        for (size_t i = 0; i < fromLineage.size(); ++i) {
            fromIndex.emplace(fromLineage[i], i);
        }

        size_t toIndex = 0;
        size_t lcaFromIndex = 0;
        bool found = false;
        for (; toIndex < toLineage.size(); ++toIndex) {
            const auto it = fromIndex.find(toLineage[toIndex]);
            if (it != fromIndex.end()) {
                lcaFromIndex = it->second;
                found = true;
                break;
            }
        }

        if (!found) {
            throw std::runtime_error("Frames '" + std::string(frameIdToString(from)) +
                                     "' and '" + std::string(frameIdToString(to)) +
                                     "' are not connected to the same root.");
        }

        std::vector<FrameId> pathUp(fromLineage.begin(), fromLineage.begin() + lcaFromIndex);
        std::vector<FrameId> pathDown(toLineage.begin(), toLineage.begin() + toIndex);
        return {pathUp, pathDown};
    }

    FrameId root_ = FrameId::ECI;
    mutable std::unordered_map<FrameId, Node> nodes_;
};

} // namespace gnc::services
