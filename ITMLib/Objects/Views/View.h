//  ================================================================
//  Created by Gregory Kramida on 7/3/20.
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

// Credits: Inspired by ITMLib/Objects/Views/View.h in original InfiniTAM (https://github.com/victorprad/InfiniTAM/)

#pragma once

#include "../Camera/CalibIO.h"
#include "../../Utils/ImageTypes.h"

namespace ITMLib {
/**
 * \brief
 * Represents a single frame's worth of data, i.e. RGB and depth images along
 * with all intrinsic and relative calibration information,
 * preprocessed and prepared for consumption by ViewBuilder.
 * */
class View {
public: // instance variables
	/// Intrinsic calibration information for the view.
	RGBD_CalibrationInformation calibration_information;

	ShortImage short_raw_disparity_image;
	FloatImage float_raw_disparity_image;

	/// RGB colour image for the current frame.
	UChar4Image rgb;

	/// Float valued depth image, if available according to @ref inputImageType.
	FloatImage depth;

	// confidence based on distance from center
	FloatImage depth_confidence;

	/// uncertainty (std) in each pixel of depth value based on sensor noise model
	/// allocated when needed
	FloatImage* depth_uncertainty;

	/// RGB colour image for the previous frame.
	/// allocated when needed
	UChar4Image* rgb_prev;

	/// surface normal of depth image
	/// allocated when needed
	Float4Image* depth_normal;

public: // instance functions

	View(const RGBD_CalibrationInformation& calibration_information, Vector2i rgb_image_size, Vector2i depth_image_size, bool use_GPU);
	View(View&& other) noexcept;
	View(const View& other) noexcept;
	View& operator=(View other);
	virtual ~View();
	void Swap(View& other);

public: // friend functions

	friend void swap(View& lhs, View& rhs) { lhs.Swap(rhs); }
};
}
