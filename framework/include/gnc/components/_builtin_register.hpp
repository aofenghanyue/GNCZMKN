/**
 * @file _builtin_register.hpp
 * @brief 仓库随附起步组件自动注册汇总头
 *
 * 本文件通过包含当前仓库随附的 starter components 头文件，触发它们在静态初始化阶段
 * 执行 `GNC_REGISTER_STARTER_COMPONENT(...)` 注册逻辑。
 *
 * 这些组件用于默认任务、最小闭环、示例和冷启动体验。
 * 它们不是 framework core 本身，而是当前仓库附带的可直接运行起步件。
 *
 * 用户通常不需要直接修改此文件；统一入口 `runner.cpp` 会包含它。
 */
#pragma once

// Dynamics and environment starter components
#include "gnc/components/dynamics/dynamics_3dof_spherical.hpp"
#include "gnc/components/dynamics/simple_dynamics.hpp"
#include "gnc/components/environment/spherical_gravity.hpp"
#include "gnc/components/environment/standard_atmosphere.hpp"
#include "gnc/components/environment/wgs84_earth.hpp"

// Navigation / sensor / state starter components
#include "gnc/components/navigation/simple_navigation.hpp"
#include "gnc/components/sensors/ideal_imu.hpp"
#include "gnc/components/state/truth_state.hpp"
