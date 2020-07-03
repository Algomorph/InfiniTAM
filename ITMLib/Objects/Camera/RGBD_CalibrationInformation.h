// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Intrinsics.h"
#include "Extrinsics.h"
#include "DisparityCalib.h"

namespace ITMLib
{
	/** \brief
	    Represents the joint RGBD calibration parameters
	*/
	class RGBD_CalibrationInformation
	{
	public:
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
		DisparityCalib disparityCalib;
	};
}
