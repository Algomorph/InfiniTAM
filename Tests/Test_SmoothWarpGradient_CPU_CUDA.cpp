//  ================================================================
//  Created by Gregory Kramida on 12/8/19.
//  Copyright (c)  2019 Gregory Kramida
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

#define BOOST_TEST_MODULE SmoothWarpGradient_CPU_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif


//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "TestUtils.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "TestUtilsForSnoopyFrames16And17.h"


using namespace ITMLib;


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_PVA, Frame16And17Fixture) {
	const int iteration = 0;

	std::string path_warps = "TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration) + "_";
	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_field_CPU;
	loadVolume(&warp_field_CPU, path_warps, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_CPU;
	loadVolume(&canonical_volume_CPU, path_frame_17_PVA, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* live_volume_CPU;
	loadVolume(&live_volume_CPU, path_frame_16_PVA, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_field_CUDA;
	loadVolume(&warp_field_CUDA, path_warps, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_CUDA;
	loadVolume(&canonical_volume_CUDA, path_frame_17_PVA, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* live_volume_CUDA;
	loadVolume(&live_volume_CUDA, path_frame_16_PVA, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());


	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true));
	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true)
	);

	motionTracker_PVA_CPU->SmoothWarpGradient(warp_field_CPU, canonical_volume_CPU, live_volume_CPU);
	motionTracker_PVA_CUDA->SmoothWarpGradient(warp_field_CUDA, canonical_volume_CUDA, live_volume_CUDA);

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA_copy(*warp_field_CUDA, MEMORYDEVICE_CPU);

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CUDA_copy, warp_field_CPU, tolerance));

	delete motionTracker_PVA_CPU;
	delete motionTracker_PVA_CUDA;

	delete warp_field_CPU;
	delete warp_field_CUDA;
	delete canonical_volume_CPU;
	delete canonical_volume_CUDA;
	delete live_volume_CPU;
	delete live_volume_CUDA;
}