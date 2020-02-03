//  ================================================================
//  Created by Gregory Kramida on 6/5/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

namespace ITMLib {
enum Axis {
	//NOTA BENE: due to absence of reflection in C++, when modifying this enum, please also modify AxisToString

	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2
};


inline std::string AxisToString(Axis axis) {
	switch (axis) {
		case AXIS_X:
			return "X";
		case AXIS_Y:
			return "Y";
		case AXIS_Z:
			return "Z";
		default:
			return  "Unknown";
	}

}

enum Plane {
	//0-2 ordered to correspond to Axes
	//NOTA BENE: due to absence of reflection in C++, when modifying this enum, please also modify PlaneToString and the
	// >> overload below

	PLANE_YZ = 0,
	PLANE_XZ = 1,
	PLANE_XY = 2,

	PLANE_ZY = 3,
	PLANE_ZX = 4,
	PLANE_YX = 5
};

inline std::string PlaneToString(Plane plane) {
	switch (plane) {
		case PLANE_XY:
			return "XY";
		case PLANE_YZ:
			return "YZ";
		case PLANE_XZ:
			return "XZ";
		case PLANE_ZY:
			return "ZY";
		case PLANE_ZX:
			return "ZX";
		case PLANE_YX:
			return "YX";
		default:
			return "Unknown";
	}
}

std::istream& operator>>(std::istream& in, ITMLib::Plane& plane);


}//namespace ITMLib

