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
#include "Extrinsics.h"
#include "CalibIO.h"
#include "../../../ORUtils/MathTypePersistence/MathTypePersistence.h"

using namespace ITMLib;

void Extrinsics::SetFrom(const Matrix4f& src) {
	this->calib = src;
	this->calib_inv.setIdentity();
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) this->calib_inv.values[r + 4 * c] = this->calib.values[c + 4 * r];
	for (int r = 0; r < 3; ++r) {
		float& dest = this->calib_inv.values[r + 4 * 3];
		dest = 0.0f;
		for (int c = 0; c < 3; ++c) dest -= this->calib.values[c + 4 * r] * this->calib.values[c + 4 * 3];
	}
}

Extrinsics::Extrinsics() {
	Matrix4f m;
	m.setZeros();
	m.m00 = m.m11 = m.m22 = m.m33 = 1.0;
	SetFrom(m);
}

namespace ITMLib{
bool operator==(const Extrinsics& rhs, const Extrinsics& lhs) {
	return rhs.calib == lhs.calib && rhs.calib_inv == lhs.calib_inv;
}

ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, Extrinsics& dest) {
	src >> dest.calib;
	src >> dest.calib_inv;
	return src;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const Extrinsics& src) {
	dest << src.calib;
	dest << src.calib_inv;
	return dest;
}

std::istream& operator>>(std::istream& src, Extrinsics& dest){
	if(!readExtrinsics(src, dest)){
		DIEWITHEXCEPTION_REPORTLOCATION("Could not read extrinsics.");
	}
	return src;
}
std::ostream& operator<<(std::ostream& dest, const Extrinsics& src){
	writeExtrinsics(dest, src);
	return dest;
}
} // namespace ITMLib
