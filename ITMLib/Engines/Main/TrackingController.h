// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "../Visualization/Interface/SurfelVisualizationEngine.h"
#include "../Visualization/Interface/VisualizationEngine.h"
#include "../../CameraTrackers/Interface/ITMCameraTracker.h"
#include "../../Utils/Configuration.h"
#include "../../CameraTrackers/Interface/ITMCameraTracker.h"

namespace ITMLib
{
	/** \brief
	*/
	class TrackingController
	{
	private:
		const configuration::Configuration *settings;
		ITMCameraTracker *tracker;

	public:
		void Track(ITMTrackingState *trackingState, const ITMView *view)
		{
			if (!tracker->requiresPointCloudRendering() || trackingState->age_pointCloud != -1)
				tracker->TrackCamera(trackingState, view);
		}

		template <typename TSurfel>
		void Prepare(ITMTrackingState *trackingState, const ITMSurfelScene<TSurfel> *scene, const ITMView *view,
		             const SurfelVisualizationEngine<TSurfel> *VisualizationEngine, ITMSurfelRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->use_approximate_raycast;

			if(requiresColourRendering)
			{
				// TODO: This should be implemented at some point.
				throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
			}
			else
			{
				const bool useRadii = true;
				VisualizationEngine->FindSurface(scene, trackingState->pose_d, &view->calib.intrinsics_d, useRadii, USR_FAUTEDEMIEUX, renderState);
				trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

				if(requiresFullRendering)
				{
					VisualizationEngine->CreateICPMaps(scene, renderState, trackingState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					trackingState->age_pointCloud++;
				}
			}
		}

		/**
		 * \brief Do whatever the hell this does, great job on the docs and procedure naming / coupling, InfiniTAM team
		 * \tparam TVoxel
		 * \tparam TIndex
		 * \param trackingState
		 * \param scene
		 * \param view
		 * \param VisualizationEngine
		 * \param renderState
		 */
		template <typename TVoxel, typename TIndex>
		void Prepare(ITMTrackingState *trackingState, ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view,
		             const VisualizationEngine<TVoxel,TIndex> *VisualizationEngine, ITMRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->use_approximate_raycast;

			if (requiresColourRendering)
			{
				ORUtils::SE3Pose pose_rgb(view->calib.trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
				VisualizationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib.intrinsics_rgb), renderState);
				VisualizationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skip_points);
				trackingState->age_pointCloud = 0;
			}
			else
			{
				VisualizationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib.intrinsics_d), renderState);

				if (requiresFullRendering)
				{
					VisualizationEngine->CreateICPMaps(scene, view, trackingState, renderState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					VisualizationEngine->ForwardRender(scene, view, trackingState, renderState);
					trackingState->age_pointCloud++;
				}
			}
		}

		TrackingController(ITMCameraTracker* tracker)
		{
			this->settings = &configuration::get();
			this->tracker = tracker;
		}

		const Vector2i& GetTrackedImageSize(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d) const
		{
			return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		TrackingController(const TrackingController&);
		TrackingController& operator=(const TrackingController&);
	};
}
