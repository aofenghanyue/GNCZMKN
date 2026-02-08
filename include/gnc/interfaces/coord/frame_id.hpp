/**
 * @file frame_id.hpp
 * @brief 坐标系标识符枚举
 */
#pragma once

#include <string>

namespace gnc::coord {

/**
 * @brief 坐标系标识枚举
 * 
 * 使用强类型枚举替代字符串，提供编译时类型安全
 */
enum class FrameId {
    ECI,        ///< 地心惯性系 (Earth-Centered Inertial)
    ECEF,       ///< 地心地固系 (Earth-Centered Earth-Fixed)
    NED,        ///< 北东地系 (North-East-Down)
    ENU,        ///< 东北天系 (East-North-Up)
    NSE,        ///< 北天东系 (North-Sky-East，用户自定义)
    BODY,       ///< 载体系
    WIND,       ///< 风轴系
    STABILITY,  ///< 稳定轴系
    LAUNCH,     ///< 发射系
    SENSOR,     ///< 传感器系
    CAMERA,     ///< 相机系
};

/**
 * @brief FrameId 转字符串（用于调试和日志）
 */
inline const char* frameIdToString(FrameId id) {
    switch (id) {
        case FrameId::ECI:       return "ECI";
        case FrameId::ECEF:      return "ECEF";
        case FrameId::NED:       return "NED";
        case FrameId::ENU:       return "ENU";
        case FrameId::NSE:       return "NSE";
        case FrameId::BODY:      return "BODY";
        case FrameId::WIND:      return "WIND";
        case FrameId::STABILITY: return "STABILITY";
        case FrameId::LAUNCH:    return "LAUNCH";
        case FrameId::SENSOR:    return "SENSOR";
        case FrameId::CAMERA:    return "CAMERA";
        default:                 return "UNKNOWN";
    }
}

} // namespace gnc::coord
