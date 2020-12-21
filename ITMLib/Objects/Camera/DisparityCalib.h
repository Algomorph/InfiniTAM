// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"
#include "../../../ORUtils/IStreamWrapper.h"
#include "../../../ORUtils/OStreamWrapper.h"

namespace ITMLib {
/** \brief
	Represents the calibration information to compute_allocated a depth
	image from the disparity image typically received from a
	Kinect.
*/
class DisparityCalib {
public: // inner types
	/** Type of transformation required to get from raw values to depths. */
	enum class TrafoType : int {
		/// Raw values are transformed according to \f$\frac{8c_2f_x}{c_1 - d}\f$
		TRAFO_KINECT = 0,
		/// Raw values are transformed according to \f$c_1 d + c_2\f$
		TRAFO_AFFINE = 1
	};

private: // instance variables
	TrafoType type;

	/** These are the actual parameters. */
	Vector2f params;


public: // instance functions
	DisparityCalib();

	const Vector2f& GetParams() const;

	TrafoType GetType() const;

	/** Setup from given arguments. */
	void SetFrom(float a, float b, TrafoType _type);

	/** Setup from standard arguments. */
	void SetStandard();
public: // friend functions
	friend bool operator==(const DisparityCalib& rhs, const DisparityCalib& lhs);
	friend ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& src, DisparityCalib& dest);
	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& dest, const DisparityCalib& src);
};
std::istream& operator>>(std::istream& src, DisparityCalib& dest);
std::ostream& operator<<(std::ostream& dest, const DisparityCalib& src);


}// namespace ITMLib
