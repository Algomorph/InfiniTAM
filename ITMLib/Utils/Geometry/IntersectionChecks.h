//  ================================================================
//  Created by Gregory Kramida on 11/25/19.
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

#include "../ITMMath.h"
#include "Segment.h"

namespace ITMLib{

/**
 * \brief Determine whether a segment intersects grid-aligned box in 3 dimensions
 * \param segment a segment, with the direction vector from the origin to the end point
 * \param boxMin minimum point of the box
 * \param boxMax maximum point of the box
 * \return true if any part of the segment intersects the box (a segment endpoint on the face of the box counts as an intersection)
 */
_CPU_AND_GPU_CODE_ inline
bool SegmentIntersectsGridAlignedBox3D(Segment segment, Vector3f boxMin, Vector3f boxMax){
	float tmin, tmax, tymin, tymax, tzmin, tzmax;
	Vector3f bounds[] = {boxMin, boxMax};

	tmin = (bounds[segment.sign[0]].x - segment.origin.x) * segment.inverseDirection.x;
	tmax = (bounds[1 - segment.sign[0]].x - segment.origin.x) * segment.inverseDirection.x;
	tymin = (bounds[segment.sign[1]].y - segment.origin.y) * segment.inverseDirection.y;
	tymax = (bounds[1 - segment.sign[1]].y - segment.origin.y) * segment.inverseDirection.y;

	if ((tmin > tymax) || (tymin > tmax)){
		return false;
	}
	if (tymin > tmin) {
		tmin = tymin;
	}
	if (tymax < tmax){
		tmax = tymax;
	}

	tzmin = (bounds[segment.sign[2]].z - segment.origin.z) * segment.inverseDirection.z;
	tzmax = (bounds[1 - segment.sign[2]].z - segment.origin.z) * segment.inverseDirection.z;

	if ((tmin > tzmax) || (tzmin > tmax)){
		return false;
	}
	if (tzmin > tmin){
		tmin = tzmin;
	}
	if (tzmax < tmax){
		tmax = tzmax;
	}
	return !(tmax < 0.0f || tmin > 1.0f);
}

/**
 * \brief Determine whether a segment along a given ray intersects grid-aligned cube in 3 dimensions
 * \param segment a segment, with the direction vector from the origin to the end point
 * \param cubeMin
 * \param cubeSideLength
 * \return true if any part of the segment intersects the cube (a segment endpoint on the face of the cube counts as an intersection)
 */
_CPU_AND_GPU_CODE_ inline
bool SegmentIntersectsGridAlignedCube3D(Segment segment, Vector3f cubeMin, float cubeSideLength){
	Vector3f cubeMax = cubeMin + Vector3f(cubeSideLength);
	return SegmentIntersectsGridAlignedBox3D(segment, cubeMin, cubeMax);
}

} // namespace ITMLib