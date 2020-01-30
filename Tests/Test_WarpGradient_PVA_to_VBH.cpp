//  ================================================================
//  Created by Gregory Kramida on 11/7/19.
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

#define BOOST_TEST_MODULE WarpGradient_VBH_to_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_map>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"
#include "WarpAdvancedTestingUtilities.h"

#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

//local CPU
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
//local CUDA
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngineFactory.h"
#endif


using namespace ITMLib;

///CAUTION: SAVE modes require the build directory to be immediately inside the root source directory.
template<MemoryDeviceType TMemoryDeviceType>
void
GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit = 10,
                GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS, float absoluteTolerance = 1e-7) {

	std::string prefix = switches_to_prefix(switches);
	GenericWarpConsistencySubtest<PlainVoxelArray, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                     absoluteTolerance);
	GenericWarpConsistencySubtest<VoxelBlockHash, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                    absoluteTolerance);

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> volume_PVA(&configuration::get().general_voxel_volume_parameters,
	                                                        configuration::get().swapping_mode ==
	                                                        configuration::SWAPPINGMODE_ENABLED,
	                                                        TMemoryDeviceType,
	                                                        Frame16And17Fixture::InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_VBH(&configuration::get().general_voxel_volume_parameters,
	                                                       configuration::get().swapping_mode ==
	                                                       configuration::SWAPPINGMODE_ENABLED,
	                                                       TMemoryDeviceType,
	                                                       Frame16And17Fixture::InitParams<VoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_PVA(&configuration::get().general_voxel_volume_parameters,
			                                                           configuration::get().swapping_mode ==
			                                                           configuration::SWAPPINGMODE_ENABLED,
			                                                           TMemoryDeviceType,
			                                                           Frame16And17Fixture::InitParams<PlainVoxelArray>());
			ITMVoxelVolume<ITMWarp, VoxelBlockHash> warp_field_VBH(&configuration::get().general_voxel_volume_parameters,
			                                                          configuration::get().swapping_mode ==
			                                                          configuration::SWAPPINGMODE_ENABLED,
			                                                          TMemoryDeviceType,
			                                                          Frame16And17Fixture::InitParams<VoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_PVA.LoadFromDirectory(get_path_warps(prefix, iteration));
				EditAndCopyEngineFactory::Instance<ITMWarp, VoxelBlockHash, TMemoryDeviceType>().ResetScene(
						&warp_field_VBH);
				warp_field_VBH.LoadFromDirectory(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(&warp_field_PVA, &warp_field_VBH,
				                                                  absoluteTolerance, TMemoryDeviceType));
				EditAndCopyEngineFactory::Instance<ITMVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetScene(
						&volume_VBH);
				volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration));
				BOOST_REQUIRE(contentForFlagsAlmostEqual_Verbose(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                                 absoluteTolerance, TMemoryDeviceType));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			volume_PVA.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			EditAndCopyEngineFactory::Instance<ITMVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetScene(
					&volume_VBH);
			volume_VBH.LoadFromDirectory(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
			volume_PVA.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			EditAndCopyEngineFactory::Instance<ITMVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetScene(
					&volume_VBH);
			volume_VBH.LoadFromDirectory(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
		}
			break;
		default:
			break;
	}
}

//#define GENERATE_TEST_DATA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, TEST_SUCCESSIVE_ITERATIONS);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CPU) {
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                  5,GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                  5,GenericWarpTestMode::TEST_FINAL_ITERATION_AND_FUSION);
#endif
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 10, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CUDA) {
	GenericWarpTest<MEMORYDEVICE_CUDA>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                   5, GenericWarpTestMode::TEST_FINAL_ITERATION_AND_FUSION, 1e-5);
}
#endif

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only_basic) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches, false);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only_expanded) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches, true);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_and_tikhonov_expanded) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches, true);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_only_expanded) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(0, switches, true);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_and_tikhonov_expanded) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(0, switches, true);
}
#endif