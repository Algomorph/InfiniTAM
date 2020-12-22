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
#include "../../../ORUtils/MemoryBlockPersistence/MemoryBlockPersistenceOperators.h"
#include "../../../ORUtils/MathTypePersistence/MathTypePersistence.h"


using namespace ITMLib;

View::View(const RGBD_CalibrationInformation& calibration_information, Vector2i rgb_image_size, Vector2i depth_image_size, bool use_GPU)
		: calibration_information(calibration_information),
		  short_raw_disparity_image(depth_image_size, true, use_GPU),
		  float_raw_disparity_image(depth_image_size, true, use_GPU),
		  rgb(rgb_image_size, true, use_GPU),
		  depth(depth_image_size, true, use_GPU),
		  depth_confidence(depth_image_size, true, use_GPU),
		  rgb_prev(nullptr),
		  depth_normal(nullptr),
		  depth_uncertainty(nullptr) {}

View::View(View&& other) noexcept:
		calibration_information(other.calibration_information),
		short_raw_disparity_image(other.short_raw_disparity_image),
		float_raw_disparity_image(other.float_raw_disparity_image),
		rgb(other.rgb),
		depth(other.depth),
		depth_confidence(other.depth_confidence),
		rgb_prev(other.rgb_prev),
		depth_normal(other.depth_normal),
		depth_uncertainty(other.depth_uncertainty) {}

View::View(const View& other) noexcept:
		calibration_information(other.calibration_information),
		short_raw_disparity_image(other.short_raw_disparity_image),
		float_raw_disparity_image(other.float_raw_disparity_image),
		rgb(other.rgb),
		depth(other.depth),
		depth_confidence(other.depth_confidence),
		rgb_prev(nullptr),
		depth_normal(nullptr),
		depth_uncertainty(nullptr) {
	if (other.rgb_prev) {
		this->rgb_prev = new UChar4Image(*other.rgb_prev);
	}
	if (other.depth_normal) {
		this->depth_normal = new Float4Image(*other.depth_normal);
	}
	if (other.depth_uncertainty) {
		this->depth_uncertainty = new FloatImage(*other.depth_uncertainty);
	}
}

View& View::operator=(View other) {
	swap(*this, other);
	return *this;
}

View::~View() {
	delete rgb_prev;
	delete depth_normal;
	delete depth_uncertainty;
}

void View::Swap(View& other) {
	using std::swap;
	swap(this->calibration_information, other.calibration_information);
	swap(this->short_raw_disparity_image, other.short_raw_disparity_image);
	swap(this->float_raw_disparity_image, other.float_raw_disparity_image);
	swap(this->rgb, other.rgb);
	swap(this->depth, other.depth);
	swap(this->depth_confidence, other.depth_confidence);
	swap(this->rgb_prev, other.rgb_prev);
	swap(this->depth_normal, other.depth_normal);
	swap(this->depth_uncertainty, other.depth_uncertainty);
}

namespace ITMLib {
bool operator==(const View& lhs, const View& rhs) {
	return lhs.calibration_information == rhs.calibration_information &&
	       lhs.short_raw_disparity_image == rhs.short_raw_disparity_image &&
	       lhs.rgb == rhs.rgb &&
	       lhs.depth == rhs.depth &&
	       lhs.depth_confidence == rhs.depth_confidence &&
	        !(lhs.depth_uncertainty == nullptr && rhs.depth_uncertainty != nullptr) &&
			!(lhs.depth_uncertainty != nullptr && rhs.depth_uncertainty == nullptr) &&
			((lhs.depth_uncertainty == nullptr && rhs.depth_uncertainty == nullptr) || 
			*lhs.depth_uncertainty == *rhs.depth_uncertainty) &&
			!(lhs.rgb_prev == nullptr && rhs.rgb_prev != nullptr) &&
			!(lhs.rgb_prev != nullptr && rhs.rgb_prev == nullptr) &&
			((lhs.rgb_prev == nullptr && rhs.rgb_prev == nullptr) ||
			 *lhs.rgb_prev == *rhs.rgb_prev) &&
			!(lhs.depth_normal == nullptr && rhs.depth_normal != nullptr) &&
			!(lhs.depth_normal != nullptr && rhs.depth_normal == nullptr) &&
			((lhs.depth_normal == nullptr && rhs.depth_normal == nullptr) ||
			 *lhs.depth_normal == *rhs.depth_normal);
}

ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& source, View& view) {
	source >> view.calibration_information;
	source >> view.short_raw_disparity_image;
	source >> view.rgb;
	source >> view.depth;
	source >> view.depth_confidence;
	bool has_depth_uncertainty, has_rgb_prev, has_depth_normal;
	source.IStream().read(reinterpret_cast<char*>(&has_depth_uncertainty), sizeof(bool));
	source.IStream().read(reinterpret_cast<char*>(&has_rgb_prev), sizeof(bool));
	source.IStream().read(reinterpret_cast<char*>(&has_depth_normal), sizeof(bool));
	if (has_depth_uncertainty) {
		view.depth_uncertainty = new FloatImage(Vector2i(0), true, view.rgb.IsAllocatedForCUDA());
		source >> *view.depth_uncertainty;
	}
	if (has_rgb_prev) {
		view.rgb_prev = new UChar4Image(Vector2i(0), true, view.rgb.IsAllocatedForCUDA());
		source >> *view.rgb_prev;
	}
	if (has_depth_normal) {
		view.depth_normal = new Float4Image(Vector2i(0), true, view.rgb.IsAllocatedForCUDA());
		source >> *view.depth_normal;
	}
	return source;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& destination, const View& view) {
	destination << view.calibration_information;
	destination << view.short_raw_disparity_image;
	destination << view.rgb;
	destination << view.depth;
	destination << view.depth_confidence;
	bool has_depth_uncertainty = view.depth_uncertainty != nullptr;
	bool has_rgb_prev = view.rgb_prev != nullptr;
	bool has_depth_normal = view.depth_normal != nullptr;
	destination.OStream().write(reinterpret_cast<char*>(&has_depth_uncertainty), sizeof(bool));
	destination.OStream().write(reinterpret_cast<char*>(&has_rgb_prev), sizeof(bool));
	destination.OStream().write(reinterpret_cast<char*>(&has_depth_normal), sizeof(bool));
	if (has_depth_uncertainty) {
		destination << *view.depth_uncertainty;
	}
	if (has_rgb_prev) {
		destination << *view.rgb_prev;
	}
	if (has_depth_normal) {
		destination << *view.depth_normal;
	}
	return destination;
}

} // namespace ITMLib