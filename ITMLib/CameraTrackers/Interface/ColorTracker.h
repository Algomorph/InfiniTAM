// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/LowLevel/Interface/LowLevelEngine.h"
#include "../../Objects/Tracking/ImageHierarchy.h"
#include "../../Objects/Tracking/ViewHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

namespace ITMLib
{
	/** Base class for engines performing point based colour
	    tracking. Implementations would typically project down a
	    point cloud into observed images and try to minimize the
	    reprojection error.
	*/
	class ColorTracker : public CameraTracker
	{
	private:
		const LowLevelEngine *lowLevelEngine;

		void PrepareForEvaluation(const View *view);

	protected: 
		TrackerIterationType iterationType;
		CameraTrackingState *trackingState; const View *view;
		ImageHierarchy<ViewHierarchyLevel> *viewHierarchy;
		int levelId;

		int countedPoints_valid;
	public:
		class EvaluationPoint
		{
		public:
			float f() { return cacheF; }
			const float* nabla_f() { if (cacheNabla == NULL) computeGradients(false); return cacheNabla; }

			const float* hessian_GN() { if (cacheHessian == NULL) computeGradients(true); return cacheHessian; }
			const ORUtils::SE3Pose & getParameter() const { return *mPara; }

			EvaluationPoint(ORUtils::SE3Pose *pos, const ColorTracker *f_parent);
			~EvaluationPoint()
			{
				delete mPara;
				if (cacheNabla != NULL) delete[] cacheNabla;
				if (cacheHessian != NULL) delete[] cacheHessian;
			}

			int getNumValidPoints() const { return mValidPoints; }

		protected:
			void computeGradients(bool requiresHessian);

			ORUtils::SE3Pose *mPara;
			const ColorTracker *mParent;

			float cacheF;
			float *cacheNabla;
			float *cacheHessian;
			int mValidPoints;
		};

		EvaluationPoint* evaluateAt(ORUtils::SE3Pose *para) const
		{
			return new EvaluationPoint(para, this);
		}

		int numParameters() const { return (iterationType == TRACKER_ITERATION_ROTATION) ? 3 : 6; }

		virtual int F_oneLevel(float *f, ORUtils::SE3Pose *pose) = 0;
		virtual void G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const = 0;

		void ApplyDelta(const ORUtils::SE3Pose & para_old, const float *delta, ORUtils::SE3Pose & para_new) const;

		void TrackCamera(CameraTrackingState *trackingState, const View *view);

		bool requiresColourRendering() const { return true; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return true; }

		ColorTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
		             const LowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~ColorTracker();
	};
}
