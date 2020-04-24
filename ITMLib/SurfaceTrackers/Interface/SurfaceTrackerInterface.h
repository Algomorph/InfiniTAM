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
#pragma once

//local
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.h"
#include "../../Utils/Visualization/SceneSliceVisualizer2D.h"
#include "../../Utils/Telemetry/VolumeSequenceRecorder.h"
#include "../../Utils/CPPPrintHelpers.h"
#include "../../Utils/VoxelFlags.h"
#include "../Shared/SurfaceTrackerOptimizationParameters.h"
#include "SlavchevaSufraceTracker.h"

namespace ITMLib {
/**
 * \brief Class responsible for tracking motion of rigid or dynamic surfaces within the scene
 * \tparam TVoxel TSDF voxel type
 * \tparam TWarp Warping vector voxel type
 * \tparam TIndex Indexing structure type used for voxel volumes
 */
template<typename TVoxel, typename TWarp, typename TIndex>
class SurfaceTrackerInterface {

public:

	virtual ~SurfaceTrackerInterface() = default;
	virtual VoxelVolume<TVoxel, TIndex>* TrackFrameMotion(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>** live_volume_pair,
			VoxelVolume<TWarp, TIndex>* warp_field) = 0;

	virtual void
	CalculateWarpGradient(VoxelVolume<TWarp, TIndex>* warpField,
	                      VoxelVolume<TVoxel, TIndex>* canonicalScene,
	                      VoxelVolume<TVoxel, TIndex>* liveScene) = 0;
	virtual void SmoothWarpGradient(
			VoxelVolume<TWarp, TIndex>* warpField,
			VoxelVolume<TVoxel, TIndex>* canonicalScene,
			VoxelVolume<TVoxel, TIndex>* liveScene) = 0;
	virtual float UpdateWarps(VoxelVolume<TWarp, TIndex>* warpField,
	                          VoxelVolume<TVoxel, TIndex>* canonicalScene,
	                          VoxelVolume<TVoxel, TIndex>* liveScene) = 0;
	virtual void ClearOutFramewiseWarps(VoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual void ClearOutCumulativeWarps(VoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual void ClearOutWarpUpdates(VoxelVolume<TWarp, TIndex>* warpField) = 0;
	virtual void AddFramewiseWarpToWarp(
			VoxelVolume<TWarp, TIndex>* warpField, bool clearFramewiseWarp) = 0;



};


}//namespace ITMLib



