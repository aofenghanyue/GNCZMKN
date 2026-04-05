/**
 * @file _builtin_register.hpp
 * @brief 框架内置组件自动注册汇总头
 *
 * 本文件通过包含所有内置组件头文件，触发它们在静态初始化阶段
 * 执行 `GNC_REGISTER_COMPONENT(...)` 注册逻辑。
 *
 * 用户通常不需要直接修改此文件；统一入口 `runner.cpp` 会包含它。
 */
#pragma once

#include "gnc/components/dynamics/simple_dynamics.hpp"
#include "gnc/components/environment/wgs84_earth.hpp"
#include "gnc/components/navigation/simple_navigation.hpp"
#include "gnc/components/sensors/ideal_imu.hpp"
#include "gnc/components/state/truth_state.hpp"
