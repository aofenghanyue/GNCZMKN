/**
 * @file coord.hpp
 * @brief 坐标旋转公式库
 * 
 * 设计理念：
 * - 只提供坐标系旋转公式
 * - 不包含具体天体参数（如地球椭球）
 * - 具体坐标转换（如 LLA/ECEF）由相关组件负责
 */
#pragma once

#include "rotations.hpp"

namespace gnc::coord {

// 所有旋转公式已在 rotations.hpp 中定义
// 具体坐标转换（如 LLA/ECEF）请使用 IEarthModel 等组件

} // namespace gnc::coord
