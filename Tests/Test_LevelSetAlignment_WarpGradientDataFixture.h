//  ================================================================
//  Created by Gregory Kramida on 10/21/19.
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

//ITMLib
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"

using namespace ITMLib;


template<MemoryDeviceType TMemoryDeviceType, typename TIndex>
struct WarpGradientDataFixture {
private:
	struct WarpOutputs {
		std::shared_ptr<VoxelVolume<WarpVoxel, TIndex>> volume;
		unsigned int update_count;
		float average_update_warp_length;
	};
	std::unordered_map<int, WarpOutputs> iteration_0_outputs;
	std::unordered_map<int, WarpOutputs> iteration_1_outputs;
	LevelSetAlignmentSwitches iteration_0_complete_Sobolev_switches;
public:
	WarpGradientDataFixture();

	VoxelVolume<WarpVoxel, TIndex>* GetIteration1StartingWarpField();

	VoxelVolume<WarpVoxel, TIndex>* GetWarpField(int iteration, const LevelSetAlignmentSwitches& switches);

	unsigned int GetUpdateCount(int iteration, const LevelSetAlignmentSwitches& switches);

	float GetAverageUpdateLength(int iteration, const LevelSetAlignmentSwitches& switches);

	configuration::Configuration* settings;
	float update_0_average_length;


	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;

	const std::string path_to_data;

	const typename TIndex::InitializationParameters index_parameters;
	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>& indexing_engine;
};



