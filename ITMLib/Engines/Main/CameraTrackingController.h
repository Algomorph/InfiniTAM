// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "../Raycasting/Interface/SurfelVisualizationEngine.h"
#include "../Raycasting/Interface/RaycastingEngineInterface.h"
#include "../../CameraTrackers/Interface/CameraTracker.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../CameraTrackers/Interface/CameraTracker.h"

namespace ITMLib
{
	/** \brief
	*/
	class CameraTrackingController
	{
	private:
		const configuration::Configuration *settings;
		CameraTracker *tracker;

	public:
		void Track(CameraTrackingState *trackingState, const View *view)
		{
			if (!tracker->RequiresPointCloudRendering() || trackingState->point_cloud_age != -1)
				tracker->TrackCamera(trackingState, view);
		}

		template <typename TSurfel>
		void Prepare(CameraTrackingState *trackingState, const SurfelScene<TSurfel> *scene, const View *view,
		             const SurfelVisualizationEngine<TSurfel> *VisualizationEngine, SurfelRenderState *renderState)
		{
			if (!tracker->RequiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->RequiresColorRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->use_approximate_raycast;

			if(requiresColourRendering)
			{
				// TODO: This should be implemented at some point.
				throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
			}
			else
			{
				const bool useRadii = true;
				VisualizationEngine->FindSurface(scene, trackingState->pose_d, &view->calibration_information.intrinsics_d, useRadii, USR_FAUTEDEMIEUX, renderState);
				trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

				if(requiresFullRendering)
				{
					VisualizationEngine->CreateICPMaps(scene, renderState, trackingState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->point_cloud_age == -1) trackingState->point_cloud_age=-2;
					else trackingState->point_cloud_age = 0;
				}
				else
				{
					trackingState->point_cloud_age++;
				}
			}
		}

		/**
		 * \brief Do whatever the hell this does, great job on the docs and procedure naming and coupling, InfiniTAM authors
		 * \tparam TVoxel
		 * \tparam TIndex
		 * \param tracking_state
		 * \param volume
		 * \param view
		 * \param raycasting_engine
		 * \param render_state
		 */
		template <typename TVoxel, typename TIndex>
		void Prepare(CameraTrackingState *tracking_state, VoxelVolume<TVoxel,TIndex> *volume, const View *view,
		             const RaycastingEngineBase<TVoxel,TIndex> *raycasting_engine, RenderState *render_state)
		{
			if (!tracker->RequiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->RequiresColorRendering();
			bool requiresFullRendering = tracking_state->TrackerFarFromPointCloud() || !settings->use_approximate_raycast;

			if (requiresColourRendering)
			{
				ORUtils::SE3Pose pose_rgb(view->calibration_information.trafo_rgb_to_depth.calib_inv * tracking_state->pose_d->GetM());
				raycasting_engine->CreateExpectedDepths(volume, &pose_rgb, &(view->calibration_information.intrinsics_rgb), render_state);
				raycasting_engine->CreatePointCloud(volume, view, tracking_state, render_state);
				tracking_state->point_cloud_age = 0;
			}
			else
			{
				raycasting_engine->CreateExpectedDepths(volume, tracking_state->pose_d, &(view->calibration_information.intrinsics_d), render_state);

				if (requiresFullRendering)
				{
					raycasting_engine->CreateICPMaps(volume, view, tracking_state, render_state);
					tracking_state->pose_pointCloud->SetFrom(tracking_state->pose_d);
					if (tracking_state->point_cloud_age == -1) tracking_state->point_cloud_age=-2;
					else tracking_state->point_cloud_age = 0;
				}
				else
				{
					raycasting_engine->ForwardRender(volume, view, tracking_state, render_state);
					tracking_state->point_cloud_age++;
				}
			}
		}

		CameraTrackingController(CameraTracker* tracker)
		{
			this->settings = &configuration::Get();
			this->tracker = tracker;
		}

		const Vector2i& GetTrackedImageSize(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d) const
		{
			return tracker->RequiresColorRendering() ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		CameraTrackingController(const CameraTrackingController&);
		CameraTrackingController& operator=(const CameraTrackingController&);
	};
}
