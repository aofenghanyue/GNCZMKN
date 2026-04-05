/**
 * @file coordinate_service.hpp
 * @brief 坐标变换服务（树形结构版）
 * 
 * 提供坐标系之间的变换路径查找和组合
 * 使用树形结构存储拓扑，LCA算法查找路径
 * 
 * 架构定位：服务层（Domain Service）
 * - 不属于 core（引擎机制）
 * - 属于 GNC 仿真的领域基础设施
 * 
 * 设计特点：
 * - 树形强约束：每个坐标系只有一个父系
 * - LCA路径查找：O(Depth) 复杂度
 * - 节点级缓存：精细的变换矩阵缓存
 */
#pragma once

#include "gnc/interfaces/coord/frame_id.hpp"
#include "gnc/common/math/math.hpp"
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdexcept>
#include <sstream>

namespace gnc::services {

using namespace gnc::math;
using namespace gnc::coord;

/**
 * @brief 坐标变换服务（树形结构）
 * 
 * 管理坐标系树形拓扑和变换查找
 * 每个坐标系只有一个父节点（符合物理定义）
 */
class CoordinateService {
public:
    /// 变换提供者类型（返回旋转矩阵的Lambda）
    using RotationProvider = std::function<Matrix3()>;
    
    /**
     * @brief 设置树的根节点
     * 
     * @param root 根坐标系（通常是ECI或ECEF）
     */
    void setRoot(FrameId root) {
        root_ = root;
        nodes_[root] = Node{};
    }
    
    /**
     * @brief 注册父子关系和变换提供者
     * 
     * 树形结构：每个子节点只能有一个父节点
     * 
     * @param child 子坐标系
     * @param parent 父坐标系
     * @param childToParent 返回 child→parent 旋转矩阵的Lambda
     */
    void registerTransform(FrameId child, FrameId parent, RotationProvider childToParent) {
        // 检查父节点是否存在
        if (nodes_.find(parent) == nodes_.end()) {
            throw std::runtime_error(
                std::string("Parent frame not registered: ") + frameIdToString(parent));
        }
        
        // 检查子节点是否已有父节点（树形约束）
        auto it = nodes_.find(child);
        if (it != nodes_.end() && it->second.hasParent) {
            throw std::runtime_error(
                std::string("Frame already has a parent: ") + frameIdToString(child));
        }
        
        // 注册子节点
        nodes_[child] = Node{true, parent, std::move(childToParent), {}, -1.0};
    }
    
    /**
     * @brief 检查坐标系是否已注册
     */
    bool hasFrame(FrameId frame) const {
        return nodes_.find(frame) != nodes_.end();
    }
    
    /**
     * @brief 获取旋转矩阵（自动路径查找，带缓存）
     * 
     * @param from 源坐标系
     * @param to 目标坐标系
     * @param time 仿真时间（用于缓存失效判断）
     * @return 从源到目标的旋转矩阵
     * @throws std::runtime_error 找不到路径时
     */
    Matrix3 getRotation(FrameId from, FrameId to, double time = 0.0) const {
        // 相同坐标系返回单位矩阵
        if (from == to) {
            return Matrix3::Identity();
        }
        
        // 使用LCA算法查找路径
        auto [pathUp, lca, pathDown] = findPathViaLCA(from, to);
        
        // 组合变换
        Matrix3 result = Matrix3::Identity();
        
        // from → LCA (使用正向变换)
        for (FrameId frame : pathUp) {
            result = getToParent(frame, time) * result;
        }
        
        // LCA → to (使用逆向变换)
        for (auto it = pathDown.rbegin(); it != pathDown.rend(); ++it) {
            result = getToParent(*it, time).transpose() * result;
        }
        
        return result;
    }
    
    /**
     * @brief 变换向量
     * 
     * @param v 源坐标系中的向量
     * @param from 源坐标系
     * @param to 目标坐标系
     * @param time 仿真时间
     * @return 目标坐标系中的向量
     */
    Vector3 transform(const Vector3& v, FrameId from, FrameId to, double time = 0.0) const {
        return getRotation(from, to, time) * v;
    }
    
    /**
     * @brief 清除所有注册
     */
    void clear() {
        nodes_.clear();
        root_ = FrameId::ECI;
    }
    
    /**
     * @brief 清除所有节点缓存（时间步进时调用）
     */
    void clearCache() const {
        for (auto& [_, node] : nodes_) {
            node.cacheTime = -1.0;
        }
    }
    
    /**
     * @brief 输出拓扑树（调试用）
     */
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
    /// 树节点
    struct Node {
        bool hasParent = false;
        FrameId parent = FrameId::ECI;
        RotationProvider toParent;           ///< child→parent 变换
        mutable Matrix3 cachedRotation;      ///< 缓存的变换矩阵
        mutable double cacheTime = -1.0;     ///< 缓存时间戳
    };
    
    /**
     * @brief 获取节点到父节点的变换（带缓存）
     */
    Matrix3 getToParent(FrameId frame, double time) const {
        auto it = nodes_.find(frame);
        if (it == nodes_.end() || !it->second.hasParent) {
            throw std::runtime_error(
                std::string("Frame has no parent: ") + frameIdToString(frame));
        }
        
        Node& node = const_cast<Node&>(it->second);
        
        // 检查缓存
        if (node.cacheTime == time) {
            return node.cachedRotation;
        }
        
        // 计算并缓存
        node.cachedRotation = node.toParent();
        node.cacheTime = time;
        
        return node.cachedRotation;
    }
    
    /**
     * @brief 获取从节点到根的路径
     */
    std::vector<FrameId> getPathToRoot(FrameId frame) const {
        std::vector<FrameId> path;
        FrameId current = frame;
        
        while (true) {
            auto it = nodes_.find(current);
            if (it == nodes_.end()) {
                throw std::runtime_error(
                    std::string("Frame not in tree: ") + frameIdToString(current));
            }
            
            if (!it->second.hasParent) {
                // 到达根节点
                break;
            }
            
            path.push_back(current);
            current = it->second.parent;
        }
        
        return path;
    }
    
    /**
     * @brief 使用LCA算法查找路径
     * 
     * @return {pathUp, LCA, pathDown}
     *   - pathUp: from 到 LCA 的节点序列（不含LCA）
     *   - LCA: 最近公共祖先
     *   - pathDown: LCA 到 to 的节点序列（不含LCA，to在末尾）
     */
    std::tuple<std::vector<FrameId>, FrameId, std::vector<FrameId>> 
    findPathViaLCA(FrameId from, FrameId to) const {
        // 获取两个节点到根的路径
        auto pathFromRoot = getPathToRoot(from);
        auto pathToRoot = getPathToRoot(to);
        
        // 将路径转为集合加速查找
        std::unordered_set<FrameId> fromAncestors(pathFromRoot.begin(), pathFromRoot.end());
        fromAncestors.insert(from);
        
        // 添加根节点
        fromAncestors.insert(root_);
        
        // 找到LCA：to路径上第一个在from祖先集合中的节点
        FrameId lca = root_;
        std::vector<FrameId> pathDown;
        
        // 检查to本身是否是from的祖先
        if (fromAncestors.count(to)) {
            lca = to;
        } else {
            for (FrameId ancestor : pathToRoot) {
                if (fromAncestors.count(ancestor)) {
                    lca = ancestor;
                    break;
                }
                pathDown.push_back(ancestor);
            }
            // 加入to本身
            pathDown.insert(pathDown.begin(), to);
        }
        
        // pathUp: from到LCA（不含LCA）
        std::vector<FrameId> pathUp;
        if (from != lca) {
            for (FrameId node : pathFromRoot) {
                if (node == lca) break;
                pathUp.push_back(node);
            }
            if (pathUp.empty() || pathUp.back() != from) {
                pathUp.insert(pathUp.begin(), from);
            }
        }
        
        return {pathUp, lca, pathDown};
    }
    
    FrameId root_ = FrameId::ECI;
    mutable std::unordered_map<FrameId, Node> nodes_;
};

} // namespace gnc::services
