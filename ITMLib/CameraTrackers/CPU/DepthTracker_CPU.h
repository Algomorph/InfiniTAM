// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/DepthTracker.h"

namespace ITMLib
{
	class DepthTracker_CPU : public DepthTracker
	{
	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		DepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
		                 float terminationThreshold, float failureDetectorThreshold, const PreprocessingEngineInterface *lowLevelEngine);
		~DepthTracker_CPU();
	};
}
