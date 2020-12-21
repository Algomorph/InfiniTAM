// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Intrinsics.h"
#include "Extrinsics.h"
#include "DisparityCalib.h"
#include "../../../ORUtils/IStreamWrapper.h"
#include "../../../ORUtils/OStreamWrapper.h"

namespace ITMLib {
/** \brief
	Represents the joint RGBD calibration parameters
*/
class RGBD_CalibrationInformation {
public: // instance variables
	/// Intrinsic parameters of the RGB camera.
	Intrinsics intrinsics_rgb;

	/// Intrinsic parameters of the depth camera.
	Intrinsics intrinsics_d;

	/** @brief
		Extrinsic calibration between RGB and depth
		cameras.

		This transformation takes points from the RGB
		camera coordinate system to the depth camera
		coordinate system.
	*/
	Extrinsics trafo_rgb_to_depth;

	/// Calibration information to compute_allocated depth from disparity images.
	DisparityCalib disparity_calibration_coefficients;
public: // friend functions
	friend bool operator==(const RGBD_CalibrationInformation& lhs, const RGBD_CalibrationInformation& rhs);
	friend ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, RGBD_CalibrationInformation& dest);
	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const RGBD_CalibrationInformation& src);
};

std::istream& operator>>(std::istream& src, RGBD_CalibrationInformation& dest);
std::ostream& operator<<(std::ostream& dest, const RGBD_CalibrationInformation& src);


} // namespace ITMLib
