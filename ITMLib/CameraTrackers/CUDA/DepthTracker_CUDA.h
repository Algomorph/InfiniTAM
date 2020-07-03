// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/DepthTracker.h"

namespace ITMLib
{
	class DepthTracker_CUDA : public DepthTracker
	{
	public:
		struct AccuCell;

	private:
		AccuCell *accu_host;
		AccuCell *accu_device;

	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		DepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
		                  float terminationThreshold, float failureDetectorThreshold, const ImageProcessingEngineInterface *lowLevelEngine);
		~DepthTracker_CUDA();
	};
}
