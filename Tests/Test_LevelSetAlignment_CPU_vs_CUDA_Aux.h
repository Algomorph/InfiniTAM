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

//stdlib
#include <unordered_map>
#include <boost/test/test_tools.hpp>

//ITMLib
#include "../ORUtils/MemoryDeviceType.h"
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/EditAndCopy/Interface/EditAndCopyEngineInterface.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
//(CPU)
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif

//test_utils
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/LevelSetAlignment/TestCaseOrganizationBySwitches.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


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
	WarpGradientDataFixture() :
			settings(nullptr),
			canonical_volume(nullptr), live_volume(nullptr),
			path_to_data(GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/"),
			index_parameters(snoopy::InitializationParameters_Fr16andFr17<TIndex>()),
			indexing_engine(IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()),
			iteration_0_outputs(),
			iteration_1_outputs() {
		configuration::LoadDefault();
		settings = &configuration::Get();

		auto load_sdf_volume = [&](VoxelVolume<TSDFVoxel, TIndex>** volume, const std::string& path_suffix) {
			*volume = new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
			                                             index_parameters);
			PrepareVoxelVolumeForLoading(*volume);
			(*volume)->LoadFromDisk(path_to_data + path_suffix);
		};
		auto load_warp_volume = [&](const std::string& path_suffix) {
			auto volume = std::make_shared<VoxelVolume<WarpVoxel, TIndex>>(TMemoryDeviceType,index_parameters);
			PrepareVoxelVolumeForLoading(volume);
			volume->LoadFromDisk(path_to_data + path_suffix);
			return volume;
		};

		ORUtils::IStreamWrapper warp_stats_file(
				std::string(test_utilities::GeneratedArraysDirectory) + "warp_gradient_stats_" + IndexString<TIndex>() + ".dat", false);

		auto load_warp_outputs = [&](const LevelSetAlignmentSwitches& switches, int iteration) {
			std::string file_suffix = "warp_field_" + std::to_string(iteration) + "_" + SwitchesToPrefix(switches) + ".dat";
			auto warp_field = load_warp_volume(file_suffix);
			unsigned int update_count;
			warp_stats_file.IStream().read(reinterpret_cast<char*>(&update_count), sizeof(unsigned int));
			float average_update_warp_length;
			warp_stats_file.IStream().read(reinterpret_cast<char*>(&average_update_warp_length), sizeof(float));
			WarpOutputs warp_outputs{warp_field, update_count, average_update_warp_length};
			return warp_outputs;
		};

		load_sdf_volume(&live_volume, "snoopy_partial_frame_17.dat");
		load_sdf_volume(&canonical_volume, "snoopy_partial_frame_16.dat");
		AllocateUsingOtherVolume(canonical_volume, live_volume, TMemoryDeviceType);

		std::array<LevelSetAlignmentSwitches, 3> switches_iteration_0 = {
				LevelSetAlignmentSwitches(true, false, false, false, false),
				LevelSetAlignmentSwitches(true, false, false, false, true),
				LevelSetAlignmentSwitches(true, false, true, false, true)
		};

		std::array<LevelSetAlignmentSwitches, 4> switches_iteration_1 = {
				LevelSetAlignmentSwitches(false, false, true, false, false),
				LevelSetAlignmentSwitches(true, false, true, false, false),
				LevelSetAlignmentSwitches(true, false, true, true, false),
				LevelSetAlignmentSwitches(true, true, false, false, false)
		};

		iteration_0_complete_Sobolev_switches = switches_iteration_0[switches_iteration_0.size() - 1];

		for (auto& switches : switches_iteration_0) {
			iteration_0_outputs[SwitchesToIntCode(switches)] = load_warp_outputs(switches, 0);
		}

		for (auto& switches : switches_iteration_1) {
			iteration_1_outputs[SwitchesToIntCode(switches)] = load_warp_outputs(switches, 1);
		}
	}

	VoxelVolume<WarpVoxel, TIndex>* GetIteration1StartingWarpField() {
		return iteration_0_outputs[SwitchesToIntCode(iteration_0_complete_Sobolev_switches)].volume.get();
	}

	VoxelVolume<WarpVoxel, TIndex>* GetWarpField(int iteration, const LevelSetAlignmentSwitches& switches) {
		switch (iteration) {
			case 0:
				return iteration_0_outputs[SwitchesToIntCode(switches)].volume.get();
			case 1:
				return iteration_1_outputs[SwitchesToIntCode(switches)].volume.get();
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Iteration can only be 0 or 1 here.");
		}
	}

	unsigned int GetUpdateCount(int iteration, const LevelSetAlignmentSwitches& switches) {
		switch (iteration) {
			case 0:
				return iteration_0_outputs[SwitchesToIntCode(switches)].update_count;
			case 1:
				return iteration_1_outputs[SwitchesToIntCode(switches)].update_count;
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Iteration can only be 0 or 1 here.");
		}
	}

	float GetAverageUpdateLength(int iteration, const LevelSetAlignmentSwitches& switches) {
		switch (iteration) {
			case 0:
				return iteration_0_outputs[SwitchesToIntCode(switches)].average_update_warp_length;
			case 1:
				return iteration_1_outputs[SwitchesToIntCode(switches)].average_update_warp_length;
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Iteration can only be 0 or 1 here.");
		}
	}

	configuration::Configuration* settings;
	float update_0_average_length;


	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;

	const std::string path_to_data;

	const typename TIndex::InitializationParameters index_parameters;
	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>& indexing_engine;
};


