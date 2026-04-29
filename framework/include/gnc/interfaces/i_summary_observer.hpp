#pragma once

#include <iosfwd>

namespace gnc::interfaces {

class ISummaryObserver {
public:
    virtual ~ISummaryObserver() = default;
    virtual void writeSummary(std::ostream& out) const = 0;
};

} // namespace gnc::interfaces
