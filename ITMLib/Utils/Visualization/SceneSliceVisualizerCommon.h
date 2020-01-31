//  ================================================================
//  Created by Gregory Kramida on 7/10/18.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../ITMMath.h"
#include "VisualizationCommon.h"

using namespace ITMLib;

inline Vector6i ComputeBoundsAroundPoint(Vector3i point, int radiusInPlane, int radiusOutOfPlane, const Plane& plane) {
	Vector6i bounds;
	switch (plane) {
		case PLANE_YZ:
			bounds.min_y = point.y - radiusInPlane;
			bounds.max_y = point.y + radiusInPlane + 1;
			bounds.min_z = point.z - radiusInPlane;
			bounds.max_z = point.z + radiusInPlane + 1;
			bounds.min_x = point.x - radiusOutOfPlane;
			bounds.max_x = point.x + radiusOutOfPlane + 1;
			break;
		case PLANE_XZ:
			bounds.min_x = point.x - radiusInPlane;
			bounds.max_x = point.x + radiusInPlane + 1;
			bounds.min_z = point.z - radiusInPlane;
			bounds.max_z = point.z + radiusInPlane + 1;
			bounds.min_y = point.y - radiusOutOfPlane;
			bounds.max_y = point.y + radiusOutOfPlane + 1;
			break;
		case PLANE_XY:
			bounds.min_x = point.x - radiusInPlane;
			bounds.max_x = point.x + radiusInPlane + 1;
			bounds.min_y = point.y - radiusInPlane;
			bounds.max_y = point.y + radiusInPlane + 1;
			bounds.min_z = point.z - radiusOutOfPlane;
			bounds.max_z = point.z + radiusOutOfPlane + 1;
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("PLANE NOT SUPPORTED");
	}
	return bounds;
}

