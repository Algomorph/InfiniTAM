// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/LowLevel/Interface/LowLevelEngine.h"
#include "../../Objects/Tracking/ImageHierarchy.h"
#include "../../Objects/Tracking/DepthHierarchyLevel.h"
#include "../../Objects/Tracking/IntensityHierarchyLevel.h"
#include "../../Objects/Tracking/VolumeHierarchyLevel.h"
#include "../../Objects/Tracking/TemplatedHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

namespace ITMLib
{
	/** Base class for engine performing ICP based depth tracking.
	    A typical example would be the original KinectFusion
	    tracking algorithm.
	*/
	class ExtendedTracker : public CameraTracker
	{
	private:
		static const int MIN_VALID_POINTS_DEPTH;
		static const int MIN_VALID_POINTS_RGB;

		const LowLevelEngine *lowLevelEngine;
		ImageHierarchy<VolumeHierarchyLevel> *sceneHierarchy;
		ImageHierarchy<DepthHierarchyLevel> *viewHierarchy_Depth;
		ImageHierarchy<IntensityHierarchyLevel> *viewHierarchy_Intensity;
		ImageHierarchy<TemplatedHierarchyLevel<Float4Image> > *reprojectedPointsHierarchy;
		ImageHierarchy<TemplatedHierarchyLevel<FloatImage> > *projectedIntensityHierarchy;

		CameraTrackingState *trackingState;
		const View *view;

		int *noIterationsPerLevel;

		float terminationThreshold;

		float colourWeight;

		void PrepareForEvaluation();
		void SetEvaluationParams(int levelId);

		void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
		void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
		bool HasConverged(float *step) const;

		void SetEvaluationData(CameraTrackingState *trackingState, const View *view);

		void UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old);

		ORUtils::HomkerMap *map;
		ORUtils::SVMClassifier *svmClassifier;
		Vector4f mu, sigma;
	protected:
		float *spaceThresh;
		float *colourThresh;

		int currentLevelId;
		TrackerIterationType currentIterationType;

		Matrix4f scenePose;
		Matrix4f depthToRGBTransform;
		VolumeHierarchyLevel *sceneHierarchyLevel_Depth;
		DepthHierarchyLevel *viewHierarchyLevel_Depth;
		IntensityHierarchyLevel *viewHierarchyLevel_Intensity;
		TemplatedHierarchyLevel<Float4Image> *reprojectedPointsLevel;
		TemplatedHierarchyLevel<FloatImage > *projectedIntensityLevel;

		bool useColour;
		bool useDepth;

		float minColourGradient;
		float viewFrustum_min, viewFrustum_max;
		float tukeyCutOff;
		int framesToSkip, framesToWeight;
		int framesProcessed;

		virtual int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;
		virtual int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxPose) = 0;
		virtual void ProjectCurrentIntensityFrame(Float4Image *points_out,
		                                          FloatImage *intensity_out,
		                                          const FloatImage *intensity_in,
		                                          const FloatImage *depth_in,
		                                          const Vector4f &intrinsics_depth,
		                                          const Vector4f &intrinsics_rgb,
		                                          const Matrix4f &scenePose) = 0;

	public:
		void TrackCamera(CameraTrackingState *trackingState, const View *view);

		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return true; }
		bool requiresPointCloudRendering() const { return true; }

		void SetupLevels(int numIterCoarse, int numIterFine, float spaceThreshCoarse, float spaceThreshFine, float colourThreshCoarse, float colourThreshFine);

		ExtendedTracker(Vector2i imgSize_d,
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
		                const LowLevelEngine *lowLevelEngine,
		                MemoryDeviceType memoryType
						   );
		virtual ~ExtendedTracker();
	};
}
