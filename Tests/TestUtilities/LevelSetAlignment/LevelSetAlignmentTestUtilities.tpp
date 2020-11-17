//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
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

//boost
#include <boost/test/test_tools.hpp>

//ITMLib
#include "../../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../../../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../../../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../../../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../../../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "../../../ITMLib/Utils/Geometry/SpatialIndexConversions.h"
//(CPU)
#include "../../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif


//test_utilities
#include "../TestUtilities.h"
#include "../SnoopyTestUtilities.h"
#include "GenericWarpConsistencySubtest.h"
#include "LevelSetAlignmentTestUtilities.h"
#include "TestCaseOrganizationBySwitches.h"
#include "SingleIterationTestConditions.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

namespace test_utilities {


template<MemoryDeviceType TMemoryDeviceType>
void PVA_to_VBH_WarpComparisonSubtest(int iteration, LevelSetAlignmentSwitches tracker_switches, float absolute_tolerance) {

	if (iteration < 0) {
		DIEWITHEXCEPTION_REPORTLOCATION("Expecting iteration >= 0, got less than that, aborting.");
	}

	std::string prefix = SwitchesToPrefix(tracker_switches);

	// *** initialize/load warps
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps_PVA = GetWarpsPath<PlainVoxelArray>(prefix, iteration - 1);
		LoadVolume(&warps_PVA, path_warps_PVA, TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
		std::string path_warps_VBH = GetWarpsPath<VoxelBlockHash>(prefix, iteration - 1);
		LoadVolume(&warps_VBH, path_warps_VBH, TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		BOOST_REQUIRE(AllocatedContentAlmostEqual(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));
	} else {
		InitializeVolume(&warps_PVA, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>(),
		                 TMemoryDeviceType);
		InitializeVolume(&warps_VBH, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>(), TMemoryDeviceType);

		BOOST_REQUIRE(AllocatedContentAlmostEqual(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));
	}

	// *** load warped live scene
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_PVA;
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_pair_PVA[2];
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_VBH;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_pair_VBH[2];

	std::string path_live_VBH, path_live_PVA;
	if (iteration > 0) {
		path_live_PVA = GetWarpedLivePath<PlainVoxelArray>(prefix, iteration - 1);
		path_live_VBH = GetWarpedLivePath<VoxelBlockHash>(prefix, iteration - 1);
	} else {
		path_live_PVA = snoopy::PartialVolume17Path<PlainVoxelArray>();
		path_live_VBH =  snoopy::PartialVolume17Path<VoxelBlockHash>();
	}
	LoadVolume(&warped_live_PVA, path_live_PVA, TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	LoadVolume(&warped_live_VBH, path_live_VBH, TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	if (iteration == 0) {
		AllocateUsingOtherVolume(warps_VBH, warped_live_VBH, TMemoryDeviceType);
	}

	warped_pair_PVA[0] = warped_live_PVA;
	warped_pair_PVA[1] = nullptr;
	warped_pair_VBH[0] = warped_live_VBH;
	warped_pair_VBH[1] = nullptr;

	// *** load canonical volume as the two different data structures
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_16_PVA;
	LoadVolume(&volume_16_PVA, snoopy::PartialVolume16Path<PlainVoxelArray>(), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	AllocateUsingOtherVolume(volume_16_PVA, warped_live_PVA, TMemoryDeviceType);
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_16_VBH;
	LoadVolume(&volume_16_VBH, snoopy::PartialVolume16Path<VoxelBlockHash>(), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	AllocateUsingOtherVolume(volume_16_VBH, warped_live_VBH, TMemoryDeviceType);

	// *** perform the warp gradient computation and warp updates
	LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, TMemoryDeviceType, DIAGNOSTIC>
			level_set_aligner_PVA(tracker_switches, SingleIterationTerminationConditions());


	BOOST_TEST_MESSAGE("==== CALCULATE PVA WARPS === ");
	configuration::Get().logging_settings.verbosity_level = VerbosityLevel::VERBOSITY_FOCUS_SPOTS;
	auto focus_spot = Vector3i(-17, 47, 223);
	configuration::Get().focus_voxel = focus_spot;
	level_set_aligner_PVA.Align(warps_PVA, warped_pair_PVA, volume_16_PVA);

	LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, TMemoryDeviceType, DIAGNOSTIC>
			level_set_aligner_VBH(tracker_switches, SingleIterationTerminationConditions());

	BOOST_TEST_MESSAGE("==== CALCULATE VBH WARPS === ");
	level_set_aligner_VBH.Align(warps_VBH, warped_pair_VBH, volume_16_VBH);

	BOOST_REQUIRE(AllocatedContentAlmostEqual_Verbose(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));

	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	VoxelVolume<WarpVoxel, PlainVoxelArray>* loaded_warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* loaded_warps_VBH;

	LoadVolume(&loaded_warps_PVA, GetWarpsPath<PlainVoxelArray>(prefix, iteration), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	LoadVolume(&loaded_warps_VBH, GetWarpsPath<VoxelBlockHash>(prefix, iteration), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());


	BOOST_REQUIRE(ContentAlmostEqual_Verbose(warps_PVA, loaded_warps_PVA, absolute_tolerance, TMemoryDeviceType));
	BOOST_REQUIRE(ContentAlmostEqual_Verbose(warps_VBH, loaded_warps_VBH, absolute_tolerance, TMemoryDeviceType));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericWarpTest(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                     LevelSetAlignmentTestMode mode, float absolute_tolerance) {

	std::string prefix = SwitchesToPrefix(switches);
	GenericWarpConsistencySubtest<PlainVoxelArray, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                  absolute_tolerance);
	GenericWarpConsistencySubtest<VoxelBlockHash, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                 absolute_tolerance);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA(TMemoryDeviceType,
	                                                   snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH(TMemoryDeviceType,
	                                                  snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_PVA(TMemoryDeviceType,
			                                                       snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
			VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_VBH(TMemoryDeviceType,
			                                                      snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_PVA.LoadFromDisk(GetWarpsPath<PlainVoxelArray>(prefix, iteration));
				warp_field_VBH.Reset();
				warp_field_VBH.LoadFromDisk(GetWarpsPath<VoxelBlockHash>(prefix, iteration));
				BOOST_REQUIRE(AllocatedContentAlmostEqual_Verbose(&warp_field_PVA, &warp_field_VBH,
				                                                  absolute_tolerance, TMemoryDeviceType));
				volume_PVA.LoadFromDisk(GetWarpedLivePath<PlainVoxelArray>(prefix, iteration));
				volume_VBH.Reset();
				volume_VBH.LoadFromDisk(GetWarpedLivePath<VoxelBlockHash>(prefix, iteration));
				BOOST_REQUIRE(ContentForFlagsAlmostEqual_Verbose(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                                 absolute_tolerance, TMemoryDeviceType));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			volume_PVA.LoadFromDisk(GetWarpedLivePath<PlainVoxelArray>(prefix, iteration_limit - 1));
			volume_VBH.Reset();
			volume_VBH.LoadFromDisk(GetWarpedLivePath<VoxelBlockHash>(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					ContentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absolute_tolerance,
					                           TMemoryDeviceType));
			volume_PVA.LoadFromDisk(GetFusedPath<PlainVoxelArray>(prefix, iteration_limit - 1));
			volume_VBH.Reset();
			volume_VBH.LoadFromDisk(GetFusedPath<VoxelBlockHash>(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					ContentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absolute_tolerance,
					                           TMemoryDeviceType));
		}
			break;
		default:
			break;
	}
}


} // namespace test_utilities