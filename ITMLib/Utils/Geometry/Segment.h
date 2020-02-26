//  ================================================================
//  Created by Gregory Kramida on 11/26/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../Math.h"

namespace ITMLib {

class Segment {
public:
	_CPU_AND_GPU_CODE_
	Segment() :
			origin(0.0f), direction(0.0f),
			inverseDirection(0.0f),
			sign{(inverseDirection.x < 0),(inverseDirection.y < 0),(inverseDirection.z < 0)}
	{}
	_CPU_AND_GPU_CODE_
	Segment(const Vector3f& startPoint, const Vector3f& endPoint) :
			origin(startPoint), direction(endPoint - startPoint),
			inverseDirection(Vector3f(1.0) / direction),
			sign{(inverseDirection.x < 0),(inverseDirection.y < 0),(inverseDirection.z < 0)}
			{}
	_CPU_AND_GPU_CODE_
	float length() const{
		return sqrtf(direction.x * direction.x + direction.y * direction.y +
		             direction.z * direction.z);
	}
	Vector3f origin, direction;
	Vector3f inverseDirection;
	int sign[3];
};

}
