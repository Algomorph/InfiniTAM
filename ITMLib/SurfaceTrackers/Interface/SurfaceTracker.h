//  ================================================================
//  Created by Gregory Kramida on 11/18/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "SurfaceTrackerInterface.h"
#include "../WarpGradientFunctors/WarpGradientFunctor.h"
#include "../../Utils/Configuration.h"

namespace ITMLib {


template<typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
class SurfaceTracker :
		public SurfaceTrackerInterface<TTSDFVoxel, TWarpVoxel, TIndex>, public SlavchevaSurfaceTracker {
public:
	using SlavchevaSurfaceTracker::SlavchevaSurfaceTracker;
	virtual ~SurfaceTracker() = default;

#ifndef __CUDACC__
	bool const histograms_enabled = configuration::get().verbosity_level >= configuration::VERBOSITY_PER_ITERATION;
#else
	bool const histograms_enabled = false;
#endif

	void ClearOutFramewiseWarp(VoxelVolume <TWarpVoxel, TIndex>* warp_field) override;
	void AddFramewiseWarpToWarp(
			VoxelVolume <TWarpVoxel, TIndex>* warp_field, bool clear_framewise_warps) override;
	void CalculateWarpGradient(VoxelVolume <TWarpVoxel, TIndex>* warp_field, VoxelVolume <TTSDFVoxel, TIndex>* canonical_volume,
	                           VoxelVolume <TTSDFVoxel, TIndex>* live_volume) override;
	void SmoothWarpGradient(VoxelVolume <TWarpVoxel, TIndex>* warp_field,
	                        VoxelVolume <TTSDFVoxel, TIndex>* canonical_volume,
	                        VoxelVolume <TTSDFVoxel, TIndex>* live_volume) override;
	float UpdateWarps(VoxelVolume <TWarpVoxel, TIndex>* warp_field,
	                  VoxelVolume <TTSDFVoxel, TIndex>* canonical_volume,
	                  VoxelVolume <TTSDFVoxel, TIndex>* live_volume) override;
	void ResetWarps(VoxelVolume <TWarpVoxel, TIndex>* warp_field) override;
};


} //namespace ITMLib

