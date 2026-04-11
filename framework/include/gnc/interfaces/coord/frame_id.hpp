/**
 * @file frame_id.hpp
 * @brief Public frame identifiers for the framework's built-in coordinate services.
 */
#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace gnc::coord {

enum class FrameId {
    ECI,
    ECEF,
    NUE,
    LAUNCH,
    LAUNCH_INERTIAL,
    BODY,
    TRACK,
    WIND,
};

inline const char* frameIdToString(FrameId id) {
    switch (id) {
        case FrameId::ECI:              return "ECI";
        case FrameId::ECEF:             return "ECEF";
        case FrameId::NUE:              return "NUE";
        case FrameId::LAUNCH:           return "LAUNCH";
        case FrameId::LAUNCH_INERTIAL:  return "LAUNCH_INERTIAL";
        case FrameId::BODY:             return "BODY";
        case FrameId::TRACK:            return "TRACK";
        case FrameId::WIND:             return "WIND";
        default:                        return "UNKNOWN";
    }
}

inline const char* frameIdToSymbol(FrameId id) {
    switch (id) {
        case FrameId::ECI:              return "I";
        case FrameId::ECEF:             return "E";
        case FrameId::NUE:              return "N";
        case FrameId::LAUNCH:           return "L";
        case FrameId::LAUNCH_INERTIAL:  return "LI";
        case FrameId::BODY:             return "B";
        case FrameId::TRACK:            return "K";
        case FrameId::WIND:             return "V";
        default:                        return "?";
    }
}

inline bool tryParseFrameId(std::string token, FrameId& out) {
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });

    if (token == "ECI" || token == "I") {
        out = FrameId::ECI;
        return true;
    }
    if (token == "ECEF" || token == "E") {
        out = FrameId::ECEF;
        return true;
    }
    if (token == "NUE" || token == "N") {
        out = FrameId::NUE;
        return true;
    }
    if (token == "LAUNCH" || token == "L") {
        out = FrameId::LAUNCH;
        return true;
    }
    if (token == "LAUNCH_INERTIAL" || token == "LI") {
        out = FrameId::LAUNCH_INERTIAL;
        return true;
    }
    if (token == "BODY" || token == "B") {
        out = FrameId::BODY;
        return true;
    }
    if (token == "TRACK" || token == "K") {
        out = FrameId::TRACK;
        return true;
    }
    if (token == "WIND" || token == "V") {
        out = FrameId::WIND;
        return true;
    }
    return false;
}

} // namespace gnc::coord
