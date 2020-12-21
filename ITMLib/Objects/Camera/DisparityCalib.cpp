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

#include "DisparityCalib.h"
#include "CalibIO.h"
#include "../../../ORUtils/MathTypePersistence/MathTypePersistence.h"

using namespace ITMLib;

DisparityCalib::DisparityCalib() {
	SetStandard();
}

const Vector2f& DisparityCalib::GetParams() const {
	return this->params;
}

DisparityCalib::TrafoType DisparityCalib::GetType() const {
	return type;
}

void DisparityCalib::SetFrom(float a, float b, DisparityCalib::TrafoType _type) {
	if (a != 0.0f || b != 0.0f) {
		params.x = a;
		params.y = b;
		type = _type;
	} else SetStandard();
}

void DisparityCalib::SetStandard() {
	// standard calibration parameters - converts mm to metres by dividing by 1000
	SetFrom(1.0f / 1000.0f, 0.0f, TrafoType::TRAFO_AFFINE);
}

namespace ITMLib {
bool operator==(const DisparityCalib& rhs, const DisparityCalib& lhs) {
	return rhs.type == lhs.type && rhs.params == lhs.params;
}

std::istream& operator>>(std::istream& src, DisparityCalib& dest) {
	if(!readDisparityCalib(src, dest)){
		DIEWITHEXCEPTION_REPORTLOCATION("Could not read disparity calibration.");
	}
	return src;
}

std::ostream& operator<<(std::ostream& dest, const DisparityCalib& src) {
	writeDisparityCalib(dest, src);
	return dest;
}

ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, DisparityCalib& dest) {
	src.IStream().read(reinterpret_cast<char*>(&dest.type), sizeof(DisparityCalib::TrafoType));
	src >> dest.params;
	return src;
}
ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const DisparityCalib& src){
	dest.OStream().write(reinterpret_cast<const char*>(&src.type), sizeof (DisparityCalib::TrafoType));
	dest << src.params;
	return dest;
}


} // namespace ITMLib
