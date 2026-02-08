/**
 * @file service_context.hpp
 * @brief 服务上下文容器
 * 
 * 提供服务的依赖注入容器
 * 实现"Pay only for what you use"原则
 */
#pragma once

#include <memory>
#include <typeindex>
#include <unordered_map>
#include <stdexcept>

namespace gnc::core {

/**
 * @brief 服务上下文
 * 
 * 存储服务实例的容器，支持按类型获取服务
 * 组件通过此容器访问所需服务
 */
class ServiceContext {
public:
    /**
     * @brief 注册服务
     * 
     * @tparam T 服务类型
     * @param service 服务实例（shared_ptr）
     */
    template<typename T>
    void registerService(std::shared_ptr<T> service) {
        services_[std::type_index(typeid(T))] = service;
    }
    
    /**
     * @brief 获取服务（可能为空）
     * 
     * @tparam T 服务类型
     * @return 服务指针，不存在返回nullptr
     */
    template<typename T>
    T* get() const {
        auto it = services_.find(std::type_index(typeid(T)));
        if (it == services_.end()) {
            return nullptr;
        }
        return static_cast<T*>(it->second.get());
    }
    
    /**
     * @brief 获取服务（必须存在）
     * 
     * @tparam T 服务类型
     * @return 服务引用
     * @throws std::runtime_error 服务不存在时
     */
    template<typename T>
    T& require() const {
        T* service = get<T>();
        if (!service) {
            throw std::runtime_error("Required service not found");
        }
        return *service;
    }
    
    /**
     * @brief 检查服务是否存在
     */
    template<typename T>
    bool has() const {
        return services_.find(std::type_index(typeid(T))) != services_.end();
    }
    
    /**
     * @brief 清除所有服务
     */
    void clear() {
        services_.clear();
    }

private:
    std::unordered_map<std::type_index, std::shared_ptr<void>> services_;
};

} // namespace gnc::core
