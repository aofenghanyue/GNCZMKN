#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/plugin.hpp"
#include "gnc/core/service_context.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gnc::core {

class ComponentRegistry;

class PluginRegistry {
public:
    using DeferredAction = std::function<void(ComponentRegistry&)>;

    struct ServiceInstallRequest {
        const ConfigNode& config;
        ServiceContext& services;
        const std::string& service_scope_name;
        const std::string& registry_scope;
        std::vector<DeferredAction>& deferred_actions;
    };

    using ServiceInstaller = std::function<void(const ServiceInstallRequest&)>;

    static PluginRegistry& instance() {
        static PluginRegistry registry;
        return registry;
    }

    void registerPlugin(const Plugin& plugin) {
        const std::string plugin_name = plugin.name();
        if (plugin_name.empty()) {
            throw std::runtime_error("Plugin name must not be empty.");
        }
        if (plugin_names_.count(plugin_name) > 0) {
            return;
        }
        plugin.install(*this);
        plugin_names_.insert(plugin_name);
        ordered_plugins_.push_back(plugin_name);
    }

    void registerServiceInstaller(const std::string& service_name,
                                  ServiceInstaller installer) {
        if (service_name.empty()) {
            throw std::runtime_error("Service installer name must not be empty.");
        }
        service_installers_[service_name] = std::move(installer);
    }

    bool hasServiceInstaller(const std::string& service_name) const {
        return service_installers_.count(service_name) > 0;
    }

    void installService(const std::string& service_name,
                        const ServiceInstallRequest& request) const {
        const auto it = service_installers_.find(service_name);
        if (it == service_installers_.end()) {
            throw std::runtime_error("No plugin service installer registered for '" +
                                     service_name + "'.");
        }
        it->second(request);
    }

    std::vector<std::string> getPluginNames() const {
        return ordered_plugins_;
    }

    std::vector<std::string> getServiceNames() const {
        std::vector<std::string> names;
        names.reserve(service_installers_.size());
        for (const auto& [name, _] : service_installers_) {
            names.push_back(name);
        }
        return names;
    }

private:
    PluginRegistry() = default;

    std::unordered_map<std::string, ServiceInstaller> service_installers_;
    std::unordered_set<std::string> plugin_names_;
    std::vector<std::string> ordered_plugins_;
};

template<typename PluginType>
class PluginRegistrar {
public:
    PluginRegistrar() {
        static const PluginType plugin_instance{};
        PluginRegistry::instance().registerPlugin(plugin_instance);
    }
};

#define GNC_REGISTER_PLUGIN(PluginType) \
    namespace { \
        const ::gnc::core::PluginRegistrar<PluginType> \
            PluginType##_plugin_registrar_instance{}; \
    }

} // namespace gnc::core
