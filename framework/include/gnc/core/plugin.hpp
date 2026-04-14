#pragma once

#include <string>
#include <vector>

namespace gnc::core {

class PluginRegistry;

enum class PluginLayer {
    Kernel,
    Subsystem,
    System,
    Application
};

class Plugin {
public:
    virtual ~Plugin() = default;
    virtual const char* name() const = 0;
    virtual PluginLayer layer() const = 0;
    virtual std::vector<std::string> dependencies() const { return {}; }
    virtual void install(PluginRegistry& registry) const = 0;
};

} // namespace gnc::core
