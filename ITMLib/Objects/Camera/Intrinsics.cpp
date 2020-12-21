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
#include "Intrinsics.h"
#include "CalibIO.h"
#include "../../../ORUtils/MathTypePersistence/MathTypePersistence.h"

using namespace ITMLib;

void Intrinsics::SetFrom(float fx, float fy, float cx, float cy) {
	projection_params_simple.fx = fx;
	projection_params_simple.fy = fy;
	projection_params_simple.cx = cx;
	projection_params_simple.cy = cy;

	projection_params_simple.all.x = fx;
	projection_params_simple.all.y = fy;
	projection_params_simple.all.z = cx;
	projection_params_simple.all.w = cy;
}

bool Intrinsics::FocalLengthSignsDiffer() const {
	return projection_params_simple.fx * projection_params_simple.fy < 0.f;
}

Intrinsics::Intrinsics() {
	// standard calibration parameters for Kinect RGB camera. Not at all
	// accurate, though...
	SetFrom(580, 580, 320, 240);
}

namespace ITMLib {
bool operator==(const Intrinsics& lhs, const Intrinsics& rhs) {
	return lhs.projection_params_simple.fx == rhs.projection_params_simple.fx &&
	       lhs.projection_params_simple.fy == rhs.projection_params_simple.fy &&
	       lhs.projection_params_simple.cx == rhs.projection_params_simple.cx &&
	       lhs.projection_params_simple.cy == rhs.projection_params_simple.cy &&
	       lhs.projection_params_simple.all == rhs.projection_params_simple.all;
}

ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, Intrinsics& dest) {
	src >> dest.projection_params_simple.all;
	src.IStream().read(reinterpret_cast<char*>(&dest.projection_params_simple.fx), sizeof(float));
	src.IStream().read(reinterpret_cast<char*>(&dest.projection_params_simple.fy), sizeof(float));
	src.IStream().read(reinterpret_cast<char*>(&dest.projection_params_simple.cx), sizeof(float));
	src.IStream().read(reinterpret_cast<char*>(&dest.projection_params_simple.cy), sizeof(float));
	return src;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const Intrinsics& src) {
	dest << src.projection_params_simple.all;
	dest.OStream().write(reinterpret_cast<const char*>(&src.projection_params_simple.fx), sizeof(float));
	dest.OStream().write(reinterpret_cast<const char*>(&src.projection_params_simple.fy), sizeof(float));
	dest.OStream().write(reinterpret_cast<const char*>(&src.projection_params_simple.cx), sizeof(float));
	dest.OStream().write(reinterpret_cast<const char*>(&src.projection_params_simple.cy), sizeof(float));
	return dest;
}

std::istream& operator>>(std::istream& src, Intrinsics& dest){
	if(!readIntrinsics(src, dest)){
		DIEWITHEXCEPTION_REPORTLOCATION("Could not read intrinsics!");
	}
	return src;
}
std::ostream& operator<<(std::ostream& dest, const Intrinsics& src){
	writeIntrinsics(dest, src);
	return dest;
}
} // namespace ITMLib

