// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/ImageProcessing/Interface/ImageProcessingEngineInterface.h"
#include "../../Objects/Tracking/ImageHierarchy.h"
#include "../../Objects/Tracking/TemplatedHierarchyLevel.h"
#include "../../Objects/Tracking/VolumeHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

namespace ITMLib
{
	/** Base class for engine performing ICP based depth tracking.
	    A typical example would be the original KinectFusion
	    tracking algorithm.
	*/
	class DepthTracker : public CameraTracker
	{
	private:
		const ImageProcessingEngineInterface *lowLevelEngine;
		ImageHierarchy<VolumeHierarchyLevel> *sceneHierarchy;
		ImageHierarchy<TemplatedHierarchyLevel<FloatImage> > *viewHierarchy;

		CameraTrackingState *trackingState; const View *view;

		int *noIterationsPerLevel;

		float terminationThreshold;

		void PrepareForEvaluation();
		void SetEvaluationParams(int levelId);

		void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
		void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
		bool HasConverged(float *step) const;

		void SetEvaluationData(CameraTrackingState *trackingState, const View* view);

		void UpdatePoseQuality(int old_valid_points_count, float *hessian_good, float f_old);

		ORUtils::HomkerMap *map;
		ORUtils::SVMClassifier *svmClassifier;
		Vector4f mu, sigma;
	protected:
		float *distThresh;

		int levelId;
		TrackerIterationType iterationType;

		Matrix4f scenePose;
		VolumeHierarchyLevel *sceneHierarchyLevel;
		TemplatedHierarchyLevel<FloatImage> *viewHierarchyLevel;

		virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

	public:
		void TrackCamera(CameraTrackingState *trackingState, const View *view);

		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return true; }

		void SetupLevels(int numIterCoarse, int numIterFine, float distThreshCoarse, float distThreshFine);

		DepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
		             float terminationThreshold, float failureDetectorThreshold,
		             const ImageProcessingEngineInterface *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~DepthTracker();
	};
}
