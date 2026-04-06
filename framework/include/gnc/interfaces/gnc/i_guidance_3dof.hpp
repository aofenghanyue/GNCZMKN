#pragma once

namespace gnc::interfaces {

struct FlightCommand3DOF {
    double alpha = 0.0;
    double sigma = 0.0;
    double timestamp = 0.0;
};

class IGuidance3DOF {
public:
    virtual ~IGuidance3DOF() = default;
    virtual const FlightCommand3DOF& getFlightCommand() const = 0;
    virtual bool isActive() const = 0;
};

} // namespace gnc::interfaces
