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

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


template<MemoryDeviceType TMemoryDeviceType, typename TIndex>
struct WarpGradientDataFixture {
	WarpGradientDataFixture() :
			settings(nullptr),
			warp_field_data_term(nullptr), canonical_volume(nullptr), live_volume(nullptr),
			path_to_data(GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/"),
			index_parameters(snoopy::InitializationParameters_Fr16andFr17<TIndex>()),
			indexing_engine(IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()){
		configuration::load_default();
		settings = &configuration::get();

		BOOST_TEST_MESSAGE("setup fixture");
		auto loadSdfVolume = [&](VoxelVolume<TSDFVoxel, TIndex>** volume, const std::string& pathSuffix) {
			*volume = new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
			                                             index_parameters);
			PrepareVoxelVolumeForLoading(*volume);
			(*volume)->LoadFromDisk(path_to_data + pathSuffix);
		};
		auto loadWarpVolume = [&](VoxelVolume<WarpVoxel, TIndex>** volume, const std::string& pathSuffix) {
			*volume = new VoxelVolume<WarpVoxel, TIndex>(TMemoryDeviceType,
			                                             index_parameters);
			PrepareVoxelVolumeForLoading(*volume);
			(*volume)->LoadFromDisk(path_to_data + pathSuffix);
		};
		loadSdfVolume(&live_volume, "snoopy_partial_frame_17.dat");
		loadSdfVolume(&canonical_volume, "snoopy_partial_frame_16.dat");
		AllocateUsingOtherVolume(canonical_volume, live_volume, TMemoryDeviceType);
		loadWarpVolume(&warp_field_data_term, "warp_field_0_data.dat");
		loadWarpVolume(&warp_field_iter0, "warp_field_0_data_framewise_warps.dat");
		loadWarpVolume(&warp_field_data_term_smoothed, "warp_field_0_smoothed.dat");
		loadWarpVolume(&warp_field_tikhonov_term, "warp_field_1_tikhonov.dat");
		loadWarpVolume(&warp_field_data_and_tikhonov_term, "warp_field_1_data_and_tikhonov.dat");
		loadWarpVolume(&warp_field_data_and_killing_term, "warp_field_1_data_and_killing.dat");
		loadWarpVolume(&warp_field_data_and_level_set_term, "warp_field_1_data_and_level_set.dat");
	}

	~WarpGradientDataFixture() {
		BOOST_TEST_MESSAGE("teardown fixture");
		delete live_volume;
		delete canonical_volume;
		delete warp_field_data_term;
		delete warp_field_iter0;
		delete warp_field_data_term_smoothed;
		delete warp_field_tikhonov_term;
		delete warp_field_data_and_tikhonov_term;
		delete warp_field_data_and_killing_term;
		delete warp_field_data_and_level_set_term;
	}

	configuration::Configuration* settings;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_data_term;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_data_term_smoothed;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_iter0;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_tikhonov_term;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_data_and_tikhonov_term;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_data_and_killing_term;
	VoxelVolume<WarpVoxel, TIndex>* warp_field_data_and_level_set_term;
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	const std::string path_to_data;
	const typename TIndex::InitializationParameters index_parameters;
	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>& indexing_engine;
};
