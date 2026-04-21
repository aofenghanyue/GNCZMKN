#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <string>

namespace gnc::services::soviet_coord {

class ICoordService {
public:
    virtual ~ICoordService() = default;

    virtual gnc::math::Vector3 transform(const gnc::math::Vector3& vector,
                                         const std::string& from_frame,
                                         const std::string& to_frame,
                                         double time) const = 0;
    virtual gnc::math::Matrix3 getRotation(const std::string& from_frame,
                                           const std::string& to_frame,
                                           double time) const = 0;
    virtual bool hasFrame(const std::string& frame_id) const = 0;
    virtual bool hasEdge(const std::string& child_frame,
                         const std::string& parent_frame) const = 0;
};

} // namespace gnc::services::soviet_coord
