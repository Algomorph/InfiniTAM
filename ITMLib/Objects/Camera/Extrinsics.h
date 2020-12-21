// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"
#include "../../../ORUtils/IStreamWrapper.h"
#include "../../../ORUtils/OStreamWrapper.h"

namespace ITMLib {
/** \brief
	Represents the extrinsic calibration between RGB and depth cameras
*/
class Extrinsics {
public: // instance variables
	/**
	 * Matrix of the transformation from RGB to the depth
	 * camera.
	*/
	Matrix4f calib;
	/**
	 * Inverse of the above.
	 * */
	Matrix4f calib_inv;

public: // instance functions
	Extrinsics();

	/** Setup from a given 4x4 matrix, where only the upper
		three rows are used. More specifically, m00...m22
		are expected to contain a rotation and m30...m32
		contain the translation.
	*/
	void SetFrom(const Matrix4f& src);
public: // friend functions
	friend bool operator==(const Extrinsics& rhs, const Extrinsics& lhs);
	friend ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, Extrinsics& dest);
	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const Extrinsics& src);
};
std::istream& operator>>(std::istream& src, Extrinsics& dest);
std::ostream& operator<<(std::ostream& dst, const Extrinsics& src);

} // namespace ITMLib
