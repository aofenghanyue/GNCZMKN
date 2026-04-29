#pragma once

#include <string>

namespace gnc::interfaces {

class ITerminationEvaluator {
public:
    virtual ~ITerminationEvaluator() = default;
    virtual bool shouldTerminate() const = 0;
    virtual std::string reason() const = 0;
};

} // namespace gnc::interfaces
