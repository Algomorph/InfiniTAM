//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/17/20.
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
#include "RGBD_CalibrationInformation.h"
#include "CalibIO.h"

namespace ITMLib {
bool operator==(const ITMLib::RGBD_CalibrationInformation& lhs, const ITMLib::RGBD_CalibrationInformation& rhs) {
	return lhs.intrinsics_rgb == rhs.intrinsics_rgb &&
	       lhs.intrinsics_d == rhs.intrinsics_d &&
	       lhs.trafo_rgb_to_depth == rhs.trafo_rgb_to_depth &&
	       lhs.disparity_calibration_coefficients == rhs.disparity_calibration_coefficients;
}

ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, RGBD_CalibrationInformation& dest) {
	src >> dest.intrinsics_rgb;
	src >> dest.intrinsics_d;
	src >> dest.trafo_rgb_to_depth;
	src >> dest.disparity_calibration_coefficients;
	return src;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const RGBD_CalibrationInformation& src) {
	dest << src.intrinsics_rgb;
	dest << src.intrinsics_d;
	dest << src.trafo_rgb_to_depth;
	dest << src.disparity_calibration_coefficients;
	return dest;
}

std::istream& operator>>(std::istream& src, RGBD_CalibrationInformation& dest){
	if(!readRGBDCalib(src, dest)){
		DIEWITHEXCEPTION_REPORTLOCATION("Could not read calibration information.");
	}
	return src;
}
std::ostream& operator<<(std::ostream& dest, const RGBD_CalibrationInformation& src){
	writeRGBDCalib(dest, src);
	return dest;
}

} // namespace ITMLib