// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ExtendedTracker.h"

namespace ITMLib
{
	//N.B. public virtual inheritance here to avoid the deadly-diamond problem in ITMDynamicCameraTracker_CUDA
	class ExtendedTracker_CUDA : public virtual ExtendedTracker
	{
	public:
		struct AccuCell;

	private:
		AccuCell *accu_host;
		AccuCell *accu_device;

	protected:
		int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
		int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
		void ProjectCurrentIntensityFrame(Float4Image *points_out,
		                                  FloatImage *intensity_out,
		                                  const FloatImage *intensity_in,
		                                  const FloatImage *depth_in,
		                                  const Vector4f &intrinsics_depth,
		                                  const Vector4f &intrinsics_rgb,
		                                  const Matrix4f &scenePose);

	public:
		ExtendedTracker_CUDA(Vector2i imgSize_d,
		                     Vector2i imgSize_rgb,
		                     bool useDepth,
		                     bool useColour,
		                     float colourWeight,
		                     TrackerIterationType *trackingRegime,
		                     int noHierarchyLevels,
		                     float terminationThreshold,
		                     float failureDetectorThreshold,
		                     float viewFrustum_min,
		                     float viewFrustum_max,
		                     float minColourGradient,
		                     float tukeyCutOff,
		                     int framesToSkip,
		                     int framesToWeight,
		                     const ImageProcessingEngineInterface *lowLevelEngine);
		~ExtendedTracker_CUDA();
	};
}
