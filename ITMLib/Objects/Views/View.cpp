//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/3/20.
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

#include "View.h"

using namespace ITMLib;

View::View(const RGBD_CalibrationInformation& calibration_information, Vector2i rgb_image_size, Vector2i depth_image_size, bool use_GPU)
		: calibration_information(calibration_information),
		  short_raw_disparity_image(depth_image_size, true, use_GPU),
		  float_raw_disparity_image(depth_image_size, true, use_GPU),
		  rgb(rgb_image_size, true, use_GPU),
		  depth(depth_image_size, true, use_GPU),
		  depth_confidence(depth_image_size, true, use_GPU) {
	this->rgb_prev = nullptr;
	this->depth_normal = nullptr;
	this->depth_uncertainty = nullptr;
}

View::View(View&& other) noexcept :
		calibration_information(other.calibration_information),
		short_raw_disparity_image(other.short_raw_disparity_image),
		float_raw_disparity_image(other.float_raw_disparity_image),
		rgb(other.rgb),
		depth(other.depth),
		depth_confidence(other.depth_confidence),
		rgb_prev(other.rgb_prev),
		depth_normal(other.depth_normal),
		depth_uncertainty(other.depth_uncertainty){
	other.rgb_prev = nullptr;
	other.depth_normal = nullptr;
	other.depth_uncertainty = nullptr;
}

View::View(const View& other) noexcept :
		calibration_information(other.calibration_information),
		short_raw_disparity_image(other.short_raw_disparity_image),
		float_raw_disparity_image(other.float_raw_disparity_image),
		rgb(other.rgb),
		depth(other.depth),
		depth_confidence(other.depth_confidence),
		rgb_prev(nullptr),
		depth_normal(nullptr),
		depth_uncertainty(nullptr){
	if(other.rgb_prev){
		this->rgb_prev = new UChar4Image(*other.rgb_prev);
	}
	if(other.depth_normal){
		this->depth_normal = new Float4Image(*other.depth_normal);
	}
	if(other.depth_uncertainty){
		this->depth_uncertainty = new FloatImage(*other.depth_uncertainty);
	}
}

View& View::operator=(View other){
	swap(*this, other);
	return *this;
}

View::~View() {
	delete rgb_prev;
	delete depth_normal;
	delete depth_uncertainty;
}

void View::Swap(View& other){
	using std::swap;
	swap(this->calibration_information, other.calibration_information);
	swap(this->short_raw_disparity_image, other.short_raw_disparity_image);
	swap(this->float_raw_disparity_image, other.float_raw_disparity_image);
	swap(this->rgb, other.rgb);
	swap(this->depth, other.depth);
	swap(this->depth_confidence, other.depth_confidence);
	swap(this->rgb_prev,other.rgb_prev);
	swap(this->depth_normal,other.depth_normal);
	swap(this->depth_uncertainty,other.depth_uncertainty);
}
