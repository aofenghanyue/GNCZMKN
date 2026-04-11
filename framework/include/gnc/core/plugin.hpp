#pragma once

namespace gnc::core {

class PluginRegistry;

class Plugin {
public:
    virtual ~Plugin() = default;
    virtual const char* name() const = 0;
    virtual void install(PluginRegistry& registry) const = 0;
};

} // namespace gnc::core
