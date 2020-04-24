//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2000 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
//stdlib
#include <unordered_set>
#include <vector>

//boost
#include <boost/filesystem.hpp>

//log4cplus
#include <log4cplus/loggingmacros.h>

//local
#include "DenseDynamicMapper.h"
#include "../../DepthFusion/DepthFusionEngineFactory.h"
#include "../../Warping/WarpingEngineFactory.h"
#include "../../VolumeFusion/VolumeFusionEngineFactory.h"
#include "../../Swapping/SwappingEngineFactory.h"
#include "../../Indexing/IndexingEngineFactory.h"
#include "../../Analytics/AnalyticsEngineInterface.h"
#include "../../Analytics/AnalyticsEngineFactory.h"
#include "../../Analytics/AnalyticsLogging.h"
#include "../../Analytics/AnalyticsTelemetry.h"
#include "../../../SurfaceTrackers/SurfaceTrackerFactory.h"
#include "../../../Utils/Analytics/BenchmarkUtilities.h"
#include "../../../Utils/Logging/LoggingConfigruation.h"


using namespace ITMLib;

namespace bench = ITMLib::benchmarking;

// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxel, typename TWarp, typename TIndex>
DenseDynamicMapper<TVoxel, TWarp, TIndex>::DenseDynamicMapper(const TIndex& index) :
		indexing_engine(IndexingEngineFactory::Build<TVoxel, TIndex>(configuration::get().device_type)),
		depth_fusion_engine(
				DepthFusionEngineFactory::Build<TVoxel, TWarp, TIndex>
						(configuration::get().device_type)),
		volume_fusion_engine(VolumeFusionEngineFactory::Build<TVoxel, TIndex>(configuration::get().device_type)),
		surface_tracker(SurfaceTrackerFactory::MakeSceneMotionTracker<TVoxel, TWarp, TIndex>()),
		swapping_engine(configuration::get().swapping_mode != configuration::SWAPPINGMODE_DISABLED
		                ? SwappingEngineFactory::Build<TVoxel, TIndex>(
						configuration::get().device_type, index)
		                : nullptr),
		swapping_mode(configuration::get().swapping_mode),

		has_focus_coordinates(configuration::get().verbosity_level >= configuration::VERBOSITY_FOCUS_SPOTS),
		focus_coordinates(configuration::get().telemetry_settings.focus_coordinates),
		verbosity_level(configuration::get().verbosity_level) {
	InitializePerFrameAnalyticsTelemetry(&canonical_volume_memory_usage_file);
}

template<typename TVoxel, typename TWarp, typename TIndex>
DenseDynamicMapper<TVoxel, TWarp, TIndex>::~DenseDynamicMapper() {
	delete depth_fusion_engine;
	delete swapping_engine;
	delete surface_tracker;
	delete canonical_volume_memory_usage_file;
}
//endregion ================================= C/D ======================================================================

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessInitialFrame(
		const View* view, const CameraTrackingState* tracking_state,
		VoxelVolume<TVoxel, TIndex>* canonical_volume, VoxelVolume<TVoxel, TIndex>* live_volume) {
	LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "Generating raw live frame from view..." << reset);
	benchmarking::start_timer("GenerateRawLiveAndCanonicalVolumes");
	live_volume->Reset();
	indexing_engine->AllocateNearSurface(live_volume, view, tracking_state);
	depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(live_volume, view, tracking_state);
	benchmarking::stop_timer("GenerateRawLiveAndCanonicalVolumes");
	//** prepare canonical for new frame
	LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "Fusing data from live frame into canonical frame..." << reset);
	//** fuse the live into canonical directly
	benchmarking::start_timer("FuseOneTsdfVolumeIntoAnother");
	indexing_engine->AllocateFromOtherVolume(canonical_volume, live_volume);
	//FIXME: use proper frame number here instead of 0
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volume, 0u);
	benchmarking::stop_timer("FuseOneTsdfVolumeIntoAnother");
}


template<typename TVoxel, typename TWarp, typename TIndex>
void
DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessFrame(const View* view, const CameraTrackingState* trackingState,
                                                        VoxelVolume<TVoxel, TIndex>* canonical_volume,
                                                        VoxelVolume<TVoxel, TIndex>** live_volume_pair,
                                                        VoxelVolume<TWarp, TIndex>* warp_field) {

	LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "Generating raw live TSDF from view..." << reset);
	benchmarking::start_timer("GenerateRawLiveVolume");
	live_volume_pair[0]->Reset();
	live_volume_pair[1]->Reset();
	warp_field->Reset();

	indexing_engine->AllocateNearAndBetweenTwoSurfaces(live_volume_pair[0], view, trackingState);
	indexing_engine->AllocateFromOtherVolume(live_volume_pair[1], live_volume_pair[0]);
	indexing_engine->AllocateFromOtherVolume(canonical_volume, live_volume_pair[0]);
	indexing_engine->AllocateWarpVolumeFromOtherVolume(warp_field, live_volume_pair[0]);
	depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(live_volume_pair[0], view, trackingState);


	LogTSDFVolumeStatistics(live_volume_pair[0], "[[live TSDF before tracking]]");
	benchmarking::stop_timer("GenerateRawLiveVolume");

	benchmarking::start_timer("TrackMotion");
	LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "*** Optimizing warp based on difference between canonical and live SDF. ***" << reset);
	VoxelVolume<TVoxel, TIndex>* target_warped_live_volume =
			surface_tracker->TrackFrameMotion(canonical_volume, live_volume_pair,warp_field);
	LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "*** Warping optimization finished for current frame. ***" << reset);
	benchmarking::stop_timer("TrackMotion");
	LogTSDFVolumeStatistics(target_warped_live_volume, "[[live TSDF after tracking]]");


	//fuse warped live into canonical
	benchmarking::start_timer("FuseOneTsdfVolumeIntoAnother");
	//FIXME: use proper frame number here
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, target_warped_live_volume, 0);
	benchmarking::stop_timer("FuseOneTsdfVolumeIntoAnother");

	LogTSDFVolumeStatistics(canonical_volume, "[[canonical TSDF after fusion]]");
	RecordVolumeMemoryUsageInfo(*canonical_volume_memory_usage_file, *canonical_volume);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::UpdateVisibleList(
		const View* view,
		const CameraTrackingState* trackingState,
		VoxelVolume<TVoxel, TIndex>* scene, RenderState* renderState,
		bool resetVisibleList) {
	depth_fusion_engine->UpdateVisibleList(scene, view, trackingState, renderState, resetVisibleList);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessSwapping(
		VoxelVolume<TVoxel, TIndex>* canonicalScene, RenderState* renderState) {
	if (swapping_engine != nullptr) {
		// swapping: CPU -> CUDA
		if (swapping_mode == configuration::SWAPPINGMODE_ENABLED)
			swapping_engine->IntegrateGlobalIntoLocal(canonicalScene, renderState);

		// swapping: CUDA -> CPU
		switch (swapping_mode) {
			case configuration::SWAPPINGMODE_ENABLED:
				swapping_engine->SaveToGlobalMemory(canonicalScene, renderState);
				break;
			case configuration::SWAPPINGMODE_DELETE:
				swapping_engine->CleanLocalMemory(canonicalScene, renderState);
				break;
			case configuration::SWAPPINGMODE_DISABLED:
				break;
		}
	}
}


