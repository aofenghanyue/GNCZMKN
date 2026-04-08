#pragma once

namespace gnc::interfaces {

class IAltitudeProvider {
public:
    virtual ~IAltitudeProvider() = default;
    virtual double getAltitude() const = 0;
};

} // namespace gnc::interfaces
