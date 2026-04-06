/**
 * @file i_guidance.hpp
 * @brief 制导接口
 */
#pragma once

#include "gnc/interfaces/gnc/i_guidance_6dof.hpp"

namespace gnc::interfaces {

/**
 * @brief 制导算法接口
 */
using GuidanceCommand = GuidanceCommand6DOF;
using IGuidance = IGuidance6DOF;

} // namespace gnc::interfaces
