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
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/EditAndCopy/Interface/EditAndCopyEngineInterface.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#endif

//test_utils
#include "TestUtilsForSnoopyFrames16And17.h"
#include "TestUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy16and17utilities;

template<typename TIndex>
std::string getIndexSuffix();

template<>
std::string getIndexSuffix<VoxelBlockHash>() {
	return "VBH";
}

template<>
std::string getIndexSuffix<PlainVoxelArray>() {
	return "PVA";
}


template<MemoryDeviceType TMemoryType, typename TIndex>
struct WarpGradientDataFixture {
	WarpGradientDataFixture() :
			settings(nullptr),
			warp_field_data_term(nullptr), canonical_volume(nullptr), live_volume(nullptr),
			pathToData("TestData/snoopy_result_fr16-17_partial_" + getIndexSuffix<TIndex>() + "/"),
			indexParameters(snoopy::InitializationParameters<TIndex>()),
			indexing_engine(IndexingEngine<TSDFVoxel, TIndex, TMemoryType>::Instance()){
		configuration::load_default();
		settings = &configuration::get();

		BOOST_TEST_MESSAGE("setup fixture");
		auto loadSdfVolume = [&](VoxelVolume<TSDFVoxel, TIndex>** scene, const std::string& pathSuffix) {
			*scene = new VoxelVolume<TSDFVoxel, TIndex>(TMemoryType,
			                                            indexParameters);
			PrepareVoxelVolumeForLoading(*scene);
			(*scene)->LoadFromDisk(pathToData + pathSuffix);
		};
		auto loadWarpVolume = [&](VoxelVolume<WarpVoxel, TIndex>** scene, const std::string& pathSuffix) {
			*scene = new VoxelVolume<WarpVoxel, TIndex>(TMemoryType,
			                                            indexParameters);
			PrepareVoxelVolumeForLoading(*scene);
			(*scene)->LoadFromDisk(pathToData + pathSuffix);
		};
		loadSdfVolume(&live_volume, "snoopy_partial_frame_17_");
		loadSdfVolume(&canonical_volume, "snoopy_partial_frame_16_");
		IndexingEngine<TSDFVoxel,TIndex,TMemoryType>::Instance().AllocateUsingOtherVolume(canonical_volume, live_volume);
		loadWarpVolume(&warp_field_data_term, "warp_field_0_data_");
		loadWarpVolume(&warp_field_iter0, "warp_field_0_data_framewise_warps_");
		loadWarpVolume(&warp_field_data_term_smoothed, "warp_field_0_smoothed_");
		loadWarpVolume(&warp_field_tikhonov_term, "warp_field_1_tikhonov_");
		loadWarpVolume(&warp_field_data_and_tikhonov_term, "warp_field_1_data_and_tikhonov_");
		loadWarpVolume(&warp_field_data_and_killing_term, "warp_field_1_data_and_killing_");
		loadWarpVolume(&warp_field_data_and_level_set_term, "warp_field_1_data_and_level_set_");
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
	const std::string pathToData;
	const typename TIndex::InitializationParameters indexParameters;
	IndexingEngine<TSDFVoxel, TIndex, TMemoryType>& indexing_engine;
};


template<typename TIndex>
std::string GetIndexFolderSuffix();

template<>
std::string GetIndexFolderSuffix<VoxelBlockHash>() { return "VBH"; }

template<>
std::string GetIndexFolderSuffix<PlainVoxelArray>() { return "PVA"; }

/// Warning: has to be used when executable is within and two directories deep from the root source folder!! e.g. <root>/build/Tests
template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateTestData() {
	//TODO: use CMake-configured header instead to get source folder
	std::string sourceFolder = "../../";
	std::string output_directory =
			sourceFolder + "Tests/TestData/snoopy_result_fr16-17_partial_" + GetIndexFolderSuffix<TIndex>() + "/";

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, output_directory + "snoopy_partial_frame_17_", TMemoryDeviceType,
	           snoopy::InitializationParameters<TIndex>());
	LoadVolume(&canonical_volume, output_directory + "snoopy_partial_frame_16_", TMemoryDeviceType,
	           snoopy::InitializationParameters<TIndex>());

	SlavchevaSurfaceTracker::Switches data_only_switches(true, false, false, false, false);
	std::string data_only_filename = "warp_field_0_data_";
	SlavchevaSurfaceTracker::Switches data_smoothed_switches(false, false, false, false, true);
	std::string data_smoothed_filename = "warp_field_0_smoothed_";
	std::string framewise_warps_filename = "warp_field_0_data_framewise_warps_";
	SlavchevaSurfaceTracker::Switches warp_complete_switches(true, false, true, false, true);
	std::string warp_complete_filename = "warp_field_0_complete_";

	std::vector<std::tuple<std::string, SlavchevaSurfaceTracker::Switches>> configurationPairs = {
			std::make_tuple(std::string("warp_field_1_tikhonov_"),
			                SlavchevaSurfaceTracker::Switches(false, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_tikhonov_"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_killing_"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, true, false)),
			std::make_tuple(std::string("warp_field_1_data_and_level_set_"),
			                SlavchevaSurfaceTracker::Switches(true, true, false, false, false))
	};


	VoxelVolume<WarpVoxel, TIndex> warp_field(&configuration::get().general_voxel_volume_parameters,
	                                           configuration::get().swapping_mode ==
	                                           configuration::SWAPPINGMODE_ENABLED,
	                                          TMemoryDeviceType, snoopy::InitializationParameters<TIndex>());
	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field, live_volume);
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateFromOtherVolume(canonical_volume, live_volume);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataOnlyMotionTracker(
			data_only_switches);

	dataOnlyMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_only_filename);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataSmoothedMotionTracker(
			data_smoothed_switches);
	dataSmoothedMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_smoothed_filename);

	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field, live_volume);
	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> completeMotionTracker(
			warp_complete_switches);
	completeMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + warp_complete_filename);

	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field, live_volume);
	warp_field.LoadFromDisk(output_directory + data_only_filename);

	dataOnlyMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + framewise_warps_filename);


	for (auto& pair : configurationPairs) {
		EditAndCopyEngineFactory::Instance<WarpVoxel, TIndex, TMemoryDeviceType>().ResetVolume(&warp_field);
		warp_field.LoadFromDisk(output_directory + framewise_warps_filename);
		std::string filename = std::get<0>(pair);
		SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> tracker(
				std::get<1>(pair));
		tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
		warp_field.SaveToDisk(output_directory + filename);
	}

	delete canonical_volume;
	delete live_volume;
}