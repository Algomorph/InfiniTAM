//  ================================================================
//  Created by Gregory Kramida on 6/5/18.
//  Copyright (c) 2018-2000 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once

#include "../../Utils/Metacoding/Metacoding.h"

namespace ITMLib {

#define AXIS_ENUM_DESCRIPTION Axis, \
    (AXIS_X, "axis_x", "AXIS_X", "Axis_X",  "X", "x"),\
    (AXIS_Y, "axis_y", "AXIS_Y", "Axis_Y",  "Y", "y"),\
    (AXIS_Z, "axis_z", "AXIS_Z", "Axis_Z",  "Z", "z")

DECLARE_SERIALIZABLE_ENUM(AXIS_ENUM_DESCRIPTION);

#define PLANE_ENUM_DESCRIPTION Plane, \
    (PLANE_YZ, "plane_yz", "PLANE_YZ", "Plane_YZ",  "YZ", "yz"),\
    (PLANE_XZ, "plane_xz", "PLANE_XZ", "Plane_XZ",  "XZ", "xz"),\
    (PLANE_XY, "plane_xy", "PLANE_XY", "Plane_XY",  "XY", "xy"),\
    (PLANE_ZY, "plane_zy", "PLANE_ZY", "Plane_ZY",  "ZY", "zy"),\
    (PLANE_ZX, "plane_zx", "PLANE_ZX", "Plane_ZX",  "ZX", "zx"),\
    (PLANE_YX, "plane_yx", "PLANE_YX", "Plane_YX",  "YX", "yx")

DECLARE_SERIALIZABLE_ENUM(PLANE_ENUM_DESCRIPTION);

std::istream& operator>>(std::istream& in, ITMLib::Plane& plane);


}//namespace ITMLib

DEFINE_INLINE_SERIALIZABLE_ENUM(ITMLib::AXIS_ENUM_DESCRIPTION);
DEFINE_INLINE_SERIALIZABLE_ENUM(ITMLib::PLANE_ENUM_DESCRIPTION);

