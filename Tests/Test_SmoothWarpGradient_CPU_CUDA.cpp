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

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


BOOST_AUTO_TEST_CASE(testSmoothWarpGradient_PVA){
	const int iteration = 0;

	SlavchevaSurfaceTracker::Switches data_only_switches(true, false, false, false, false);
	std::string path_warps = GetWarpsPath<PlainVoxelArray>(SwitchesToPrefix(data_only_switches), iteration);
	std::string path_frame_16_PVA = snoopy::PartialVolume16Path<PlainVoxelArray>();
	std::string path_frame_17_PVA = snoopy::PartialVolume17Path<PlainVoxelArray>();

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_field_CPU;
	LoadVolume(&warp_field_CPU, path_warps, MEMORYDEVICE_CPU,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_CPU;
	LoadVolume(&canonical_volume_CPU, path_frame_17_PVA, MEMORYDEVICE_CPU,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* live_volume_CPU;
	LoadVolume(&live_volume_CPU, path_frame_16_PVA, MEMORYDEVICE_CPU,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_field_CUDA;
	LoadVolume(&warp_field_CUDA, path_warps, MEMORYDEVICE_CUDA,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_CUDA;
	LoadVolume(&canonical_volume_CUDA, path_frame_17_PVA, MEMORYDEVICE_CUDA,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* live_volume_CUDA;
	LoadVolume(&live_volume_CUDA, path_frame_16_PVA, MEMORYDEVICE_CUDA,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());


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