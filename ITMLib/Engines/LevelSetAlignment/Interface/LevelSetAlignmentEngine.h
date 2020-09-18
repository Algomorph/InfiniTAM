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

//temporary
#include "LevelSetAlignmentParameters.h"
#include "LevelSetAlignmentEngineInterface.h"
#include "../WarpGradientFunctors/WarpGradientFunctor.h"
#include "../../Warping/WarpingEngine.h"
#include "../../../Utils/Configuration/Configuration.h"
#include "../../../Utils/Enums/WarpType.h"


namespace ITMLib {


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
class LevelSetAlignmentEngine : public LevelSetAlignmentEngineInterface<TVoxel, TWarp, TIndex> {

private: // instance variables
	//TODO: for auto-completion in Clion, remove when CLion is fixed and this is no longer necessary
	const LevelSetAlignmentWeights& weights;
	const LevelSetAlignmentSwitches& switches;
	const LevelSetAlignmentTerminationConditions& termination;
	int iteration;

	WarpingEngineInterface<TVoxel, TWarp, TIndex>* warping_engine;
	// needs to be declared after "parameters", derives value from it during initialization
	const float mean_vector_update_threshold_in_voxels;
	const bool log_settings = false;

public: // instance functions

	LevelSetAlignmentEngine();
	LevelSetAlignmentEngine(const LevelSetAlignmentSwitches& switches);
	LevelSetAlignmentEngine(const LevelSetAlignmentSwitches& switches, const LevelSetAlignmentTerminationConditions& termination_conditions);
	virtual ~LevelSetAlignmentEngine();


	VoxelVolume<TVoxel, TIndex>* Align(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>** live_volume_pair,
			VoxelVolume<TWarp, TIndex>* warp_field) override;
protected:
	void ClearOutFramewiseWarps(VoxelVolume<TWarp, TIndex>* warp_field) override;
	void ClearOutCumulativeWarps(VoxelVolume<TWarp, TIndex>* warp_field) override;
	void ClearOutWarpUpdates(VoxelVolume<TWarp, TIndex>* warp_field) override;

	void AddFramewiseWarpToWarp(
			VoxelVolume<TWarp, TIndex>* warp_field, bool clear_framewise_warps) override;
	void CalculateWarpGradient(VoxelVolume<TWarp, TIndex>* warp_field, VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                           VoxelVolume<TVoxel, TIndex>* live_volume) override;
	void SmoothWarpGradient(VoxelVolume<TWarp, TIndex>* warp_field,
	                        VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                        VoxelVolume<TVoxel, TIndex>* live_volume) override;
	float UpdateWarps(VoxelVolume<TWarp, TIndex>* warp_field,
	                  VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                  VoxelVolume<TVoxel, TIndex>* live_volume) override;
	void PerformSingleOptimizationStep(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>* source_live_volume,
			VoxelVolume<TVoxel, TIndex>* target_live_volume,
			VoxelVolume<TWarp, TIndex>* warp_field,
			float& average_update_vector_length);

private: // instance functions
	void PerformSingleOptimizationStep_Diagnostic(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>* source_live_volume,
			VoxelVolume<TVoxel, TIndex>* target_live_volume,
			VoxelVolume<TWarp, TIndex>* warp_field,
			float& average_update_vector_length);

	void PerformSingleOptimizationStep_Optimized(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>* source_live_volume,
			VoxelVolume<TVoxel, TIndex>* target_live_volume,
			VoxelVolume<TWarp, TIndex>* warp_field,
			float& average_update_vector_length);

	template<WarpType TWarpType>
	void ClearOutWarps(VoxelVolume<TWarp, TIndex>* warp_field);
	void LogSettings();


};


} //namespace ITMLib

