#pragma once

#include <functional>

namespace gnc::core {

class ComponentRegistry;

using DeferredRegistryAction = std::function<void(ComponentRegistry&)>;

} // namespace gnc::core
