// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"
#include "../../../ORUtils/IStreamWrapper.h"
#include "../../../ORUtils/OStreamWrapper.h"

namespace ITMLib {
/** \brief
	Represents the parameters for projection with a projective
	camera
*/
class Intrinsics {
public: // instance variables
	/** The actual intrinsic calibration parameters. */
	//TODO: why not use a plain Vector4f?
	//TODO: should also contain distortion coefficients
	struct ProjectionParamsSimple {
		Vector4f all;
		float fx, fy, cx, cy;
	} projection_params_simple;
public: // instance functions
	Intrinsics();

	/** Setup all the internal members of this class from
		the given parameters. Everything is in pixel
		coordinates.
		@param fx Focal length in x direction
		@param fy Focal length in y direction
		@param cx Principal point in x direction
		@param cy Principal point in y direction
	*/
	void SetFrom(float fx, float fy, float cx, float cy);

	/**
	 * @brief Returns true if the two focal lengths have a different sign.
	 *
	 * @note  This is used to handle datasets such as ICL_NUIM and other non standard inputs
	 * 	      where one of the two focal lengths is negative: that causes the normals to point
	 * 	      away from the camera. This causes valid points to be ignored during Rendering
	 * 	      and tracking.
	 *
	 * 	      The problem presents itself only when computing normals as cross product of the
	 * 	      difference vectors between raycasted points and is thus solved by flipping
	 * 	      the normal direction.
	 */
	bool FocalLengthSignsDiffer() const;

public: // friend functions
	friend bool operator==(const Intrinsics& lhs, const Intrinsics& rhs);
	friend ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, Intrinsics& dest);
	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const Intrinsics& src);
};

std::istream& operator>>(std::istream& src, Intrinsics& dest);
std::ostream& operator<<(std::ostream& dest, const Intrinsics& src);
} // namespace ITMLib
