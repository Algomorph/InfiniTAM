//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/20/20.
//  Copyright (c) $YEAR-2020 Gregory Kramida
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

//ITMLib
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
//(CPU)
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

#endif

//test_utilities
#include "TestUtilities.h"
#include "SnoopyTestUtilities.h"

//local
#include <GenerateDerivedTestData_Config.h>
#include <log4cplus/loggingmacros.h>

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

void ConstructSnoopyUnmaskedVolumes00() {
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_00;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_00;

	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_00,
	                                              snoopy::Frame00DepthPath(),
	                                              snoopy::Frame00ColorPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr00<PlainVoxelArray>());
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_00,
	                                              snoopy::Frame00DepthPath(),
	                                              snoopy::Frame00ColorPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr00<VoxelBlockHash>());
	volume_PVA_00->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume00Path<PlainVoxelArray>());
	volume_VBH_00->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume00Path<VoxelBlockHash>());

	delete volume_PVA_00;
	delete volume_VBH_00;
}

void ConstructSnoopyMaskedVolumes16and17() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked partial volumes 16 & 17...");

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_16;
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_17;
	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_16;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_17;
	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked partial volumes 16 & 17...");

	volume_PVA_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume16Path<PlainVoxelArray>());
	volume_VBH_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume16Path<VoxelBlockHash>());
	volume_PVA_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume17Path<PlainVoxelArray>());
	volume_VBH_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume17Path<VoxelBlockHash>());

	volume_PVA_16->Reset();
	volume_VBH_16->Reset();
	volume_PVA_17->Reset();
	volume_VBH_17->Reset();

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 PVA...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 VBH...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 PVA...");
	volume_PVA_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume16Path<PlainVoxelArray>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 VBH...");
	volume_VBH_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume16Path<VoxelBlockHash>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 PVA...");
	volume_PVA_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume17Path<PlainVoxelArray>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 VBH...");
	volume_VBH_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume17Path<VoxelBlockHash>());

	delete volume_PVA_16;
	delete volume_VBH_16;
	delete volume_PVA_17;
	delete volume_VBH_17;
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateWarpGradientTestData() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating warp field data from snoopy masked partial volumes 16 & 17 "
	                << IndexString<TIndex>() << "...");

	std::string output_directory = GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/";

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, output_directory + "snoopy_partial_frame_17.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	LoadVolume(&canonical_volume, output_directory + "snoopy_partial_frame_16.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	SlavchevaSurfaceTracker::Switches data_only_switches(true, false, false, false, false);
	std::string data_only_filename = "warp_field_0_data.dat";
	SlavchevaSurfaceTracker::Switches data_smoothed_switches(false, false, false, false, true);
	std::string data_smoothed_filename = "warp_field_0_smoothed.dat";
	std::string framewise_warps_filename = "warp_field_0_data_framewise_warps.dat";
	SlavchevaSurfaceTracker::Switches warp_complete_switches(true, false, true, false, true);
	std::string warp_complete_filename = "warp_field_0_complete.dat";

	std::vector<std::tuple<std::string, SlavchevaSurfaceTracker::Switches>> configuration_pairs = {
			std::make_tuple(std::string("warp_field_1_tikhonov.dat"),
			                SlavchevaSurfaceTracker::Switches(false, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_tikhonov.dat"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_killing.dat"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, true, false)),
			std::make_tuple(std::string("warp_field_1_data_and_level_set.dat"),
			                SlavchevaSurfaceTracker::Switches(true, true, false, false, false))
	};

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType,
	                                          snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field,
	                                                                                                  live_volume);
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateFromOtherVolume(canonical_volume,
	                                                                                        live_volume);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataOnlyMotionTracker(
			data_only_switches);

	dataOnlyMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_only_filename);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataSmoothedMotionTracker(
			data_smoothed_switches);
	dataSmoothedMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_smoothed_filename);

	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field,
	                                                                                                  live_volume);
	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> completeMotionTracker(
			warp_complete_switches);
	completeMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + warp_complete_filename);

	warp_field.Reset();
	IndexingEngine<TSDFVoxel, TIndex, MEMORYDEVICE_CPU>::Instance().AllocateWarpVolumeFromOtherVolume(&warp_field,
	                                                                                                  live_volume);
	warp_field.LoadFromDisk(output_directory + data_only_filename);

	dataOnlyMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + framewise_warps_filename);

	for (auto& pair : configuration_pairs) {
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


int main(int argc, char* argv[]) {
	log4cplus::initialize();
	log4cplus::Logger::getRoot().setLogLevel(log4cplus::DEBUG_LOG_LEVEL);
	ConstructSnoopyUnmaskedVolumes00();
	ConstructSnoopyMaskedVolumes16and17();
	GenerateWarpGradientTestData<PlainVoxelArray, MEMORYDEVICE_CPU>();
	GenerateWarpGradientTestData<VoxelBlockHash, MEMORYDEVICE_CPU>();
	return 0;
}