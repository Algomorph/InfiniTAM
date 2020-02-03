//  ================================================================
//  Created by Gregory Kramida on 2/3/20.
//  Copyright (c) 2020 Gregory Kramida
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

//local
#include "../Math.h"
#include "CardinalAxesAndPlanes.h"

namespace ITMLib {

inline
float pixelsRequiredToExpandFrustumByAngle(const Vector4f& projection_parameters, const Vector2i& resolution,
                                           float extra_angle, bool negative_axis, Axis axis) {
	float leg_along_image = 0.f;
	float focal_distance = 0.f;
	switch (axis) {
		case AXIS_X:
			if (negative_axis) {
				leg_along_image = projection_parameters.cx;
			} else {
				leg_along_image = std::abs(resolution.x - projection_parameters.cx);
			}
			focal_distance = projection_parameters.fx;
			break;
		case AXIS_Y:
			if (negative_axis) {
				leg_along_image = projection_parameters.cy;
			} else {
				leg_along_image = std::abs(resolution.y - projection_parameters.y);
			}
			focal_distance = projection_parameters.fy;
			break;
		case AXIS_Z:
			DIEWITHEXCEPTION_REPORTLOCATION("Image is defined in the x-y plane, passed: AXIS_Z");
			break;
	}
	float tan_alpha = std::tan(extra_angle);
	return (leg_along_image + focal_distance * tan_alpha) / (1 - (leg_along_image / focal_distance) * tan_alpha) -
	       leg_along_image;
}

inline void expandCameraFrustumByAngle(Vector4f& expanded_projection_parameters, Vector2i& expanded_resolution,
                                       const Vector4f& projection_parameters, const Vector2i& resolution, float angle) {
	int margin_negative_x = static_cast<int>(std::ceil(
			pixelsRequiredToExpandFrustumByAngle(projection_parameters, resolution, angle,
			                                     true, AXIS_X)));
	int margin_positive_x = static_cast<int>(std::ceil(
			pixelsRequiredToExpandFrustumByAngle(projection_parameters, resolution, angle,
			                                     false, AXIS_X)));
	int margin_negative_y = static_cast<int>(std::ceil(
			pixelsRequiredToExpandFrustumByAngle(projection_parameters, resolution, angle,
			                                     true, AXIS_Y)));
	int margin_positive_y = static_cast<int>(std::ceil(
			pixelsRequiredToExpandFrustumByAngle(projection_parameters, resolution, angle,
			                                     false, AXIS_Y)));
	
	expanded_resolution.x = resolution.x + margin_negative_x + margin_positive_x;
	expanded_resolution.y = resolution.y + margin_negative_y + margin_positive_y;

	expanded_projection_parameters.fx = projection_parameters.fx;
	expanded_projection_parameters.fy = projection_parameters.fy;
	expanded_projection_parameters.cx = projection_parameters.cx + static_cast<float>(margin_negative_x);
	expanded_projection_parameters.cy = projection_parameters.cy + static_cast<float>(margin_negative_y);
}

} // namespace ITMLib
