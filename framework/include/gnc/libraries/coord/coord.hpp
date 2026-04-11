/**
 * @file coord.hpp
 * @brief Legacy include point for low-level coordinate math helpers.
 *
 * Concrete built-in frame-system definitions are exposed through
 * `gnc/services/coordinate/soviet_coordinate_system.hpp`.
 */
#pragma once

#include "rotations.hpp"

namespace gnc::coord {

// This namespace intentionally stays small. Use the built-in Soviet system
// entry point when you need frame definitions or textbook-specific transforms.

} // namespace gnc::coord
