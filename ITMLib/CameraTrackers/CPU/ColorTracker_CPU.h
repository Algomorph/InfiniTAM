// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ColorTracker.h"

namespace ITMLib
{
	class ColorTracker_CPU : public ColorTracker
	{
	public:
		int F_oneLevel(float *f, ORUtils::SE3Pose *pose);
		void G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const;

		ColorTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
		                 const LowLevelEngine *lowLevelEngine);
		~ColorTracker_CPU();
	};
}
