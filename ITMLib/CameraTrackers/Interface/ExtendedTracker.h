// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/ImageProcessing/ImageProcessingEngineInterface.h"
#include "../../Objects/Tracking/ImageHierarchy.h"
#include "../../Objects/Tracking/DepthHierarchyLevel.h"
#include "../../Objects/Tracking/IntensityHierarchyLevel.h"
#include "../../Objects/Tracking/OrderedPointCloudHierarchyLevel.h"
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
		static const int min_valid_points_depth;
		static const int min_valid_points_color;

		const ImageProcessingEngineInterface *low_level_engine;
		ImageHierarchy<OrderedPointCloudHierarchyLevel> *scene_hierarchy;
		ImageHierarchy<DepthHierarchyLevel> *view_hierarchy_depth;
		ImageHierarchy<IntensityHierarchyLevel> *view_hierarchy_intensity;
		ImageHierarchy<TemplatedHierarchyLevel<Float4Image> > *reprojected_points_hierarchy;
		ImageHierarchy<TemplatedHierarchyLevel<FloatImage> > *projected_intensity_hierarchy;

		CameraTrackingState *tracking_state;
		const View *view;

		int *iteration_per_level_count;

		float termination_threshold;

		float color_weight;

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
		float *level_distance_thresholds;
		float *level_color_thresholds;

		int current_level_id;
		TrackerIterationType current_iteration_type;

		Matrix4f scene_pose;
		Matrix4f depth_to_color_camera_transform;
		OrderedPointCloudHierarchyLevel *point_cloud_hierarchy_level_depth;
		DepthHierarchyLevel *view_hierarchy_level_depth;
		IntensityHierarchyLevel *view_hierarchy_level_intensity;
		TemplatedHierarchyLevel<Float4Image> *reprojected_points_level;
		TemplatedHierarchyLevel<FloatImage > *projected_intensity_level;

		bool use_color;
		bool use_depth;

		float min_color_gradient;
		float near_clipping_distance, far_clipping_distance;
		float tukey_cutoff;
		int frames_to_skip, frames_to_weight;
		int frames_processed;

		virtual int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;
		virtual int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxPose) = 0;
		virtual void ProjectCurrentIntensityFrame(Float4Image *points_out,
		                                          FloatImage *intensity_out,
		                                          const FloatImage *intensity_in,
		                                          const FloatImage *depth_in,
		                                          const Vector4f &intrinsics_depth,
		                                          const Vector4f &intrinsics_rgb,
		                                          const Matrix4f &volume_pose) = 0;

	public:
		void TrackCamera(CameraTrackingState *tracking_state, const View *view);

		bool RequiresColorRendering() const { return false; }
		bool RequiresDepthReliability() const { return true; }
		bool RequiresPointCloudRendering() const { return true; }

		void SetupLevels(int iteration_count_coarse, int iteration_count_fine, float distance_threshold_coarse, float distance_threshold_fine, float color_threshold_coarse, float color_threshold_fine);

		ExtendedTracker(Vector2i depth_image_size,
		                Vector2i color_image_size,
		                bool use_depth,
		                bool use_color,
		                float color_weight,
		                TrackerIterationType *level_optimization_types,
		                int hierarchy_level_count,
		                float termination_threshold,
		                float failure_detection_threshold,
		                float near_clipping_distance,
		                float far_clipping_distance,
		                float min_color_gradient,
		                float tukey_cutoff,
		                int frames_to_skip,
		                int frames_to_weight,
		                const ImageProcessingEngineInterface* image_processing_engine,
		                MemoryDeviceType memory_type);
		virtual ~ExtendedTracker();
	};
}
