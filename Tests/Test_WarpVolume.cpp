//  ================================================================
//  Created by Gregory Kramida on 10/31/19.
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

#define BOOST_TEST_MODULE WarpVolume
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "../ITMLib/Engines/Warping/WarpingEngine.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
//(CPU)
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

#endif


//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericWarpVolumeTest() {
	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps, "TestData/volumes/" + IndexString<TIndex>() + "/warp_field_0_complete.dat",
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, snoopy::PartialVolume17Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto warped_live_volume = new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
	                                                             snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warped_live_volume->Reset();

	auto warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);

	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance().AllocateFromOtherVolume(warped_live_volume,
	                                                                                        live_volume);
	warping_engine->WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);

	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume_gt;
	LoadVolume(&warped_live_volume_gt, "TestData/volumes/" + IndexString<TIndex>() + "/warped_live.dat",
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	float absolute_tolerance = 1e-6;
	BOOST_REQUIRE(!contentAlmostEqual(warped_live_volume, live_volume, absolute_tolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentAlmostEqual_Verbose(warped_live_volume, warped_live_volume_gt, absolute_tolerance, TMemoryDeviceType));

	delete warping_engine;
	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericWarpVolume_VBH_to_PVA_Test(const int iteration = 0) {
	// *** load warps
	SlavchevaSurfaceTracker::Switches data_only_switches(true, false, false, false, false);
	std::string path_warps_PVA = GetWarpsPath<PlainVoxelArray>(SwitchesToPrefix(data_only_switches), iteration);
	std::string path_warps_VBH = GetWarpsPath<VoxelBlockHash>(SwitchesToPrefix(data_only_switches), iteration);

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	LoadVolume(&warps_PVA, path_warps_PVA, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	LoadVolume(&warps_VBH, path_warps_VBH, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	std::string path_frame_17_PVA = snoopy::PartialVolume17Path<PlainVoxelArray>();
	std::string path_frame_17_VBH = snoopy::PartialVolume17Path<VoxelBlockHash>();

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA = GetWarpedLivePath<PlainVoxelArray>(SwitchesToPrefix(data_only_switches), iteration - 1);
		source_path_VBH = GetWarpedLivePath<VoxelBlockHash>(SwitchesToPrefix(data_only_switches), iteration - 1);
	}

	// *** load same frame scene as the two different data structures
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* source_volume_PVA;
	LoadVolume(&source_volume_PVA, source_path_PVA, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* source_volume_VBH;
	LoadVolume(&source_volume_VBH, source_path_VBH, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	// *** initialize target scenes
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* target_PVA;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* target_VBH;
	initializeVolume(&target_PVA, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>(), TMemoryDeviceType);
	initializeVolume(&target_VBH, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>(), TMemoryDeviceType);

	// *** perform the warping
	auto warping_engine_PVA =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, PlainVoxelArray>(TMemoryDeviceType);
	auto warping_engine_VBH =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(TMemoryDeviceType);

	IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>& indexer =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>::Instance();
	indexer.AllocateFromOtherVolume(target_VBH, source_volume_VBH);
	warping_engine_PVA->WarpVolume_WarpUpdates(warps_PVA, source_volume_PVA, target_PVA);
	warping_engine_VBH->WarpVolume_WarpUpdates(warps_VBH, source_volume_VBH, target_VBH);

	// *** test content
	float absolute_tolerance = 1e-7;
	BOOST_REQUIRE(
			contentForFlagsAlmostEqual_Verbose(target_PVA, target_VBH, VOXEL_NONTRUNCATED, absolute_tolerance,
			                                   TMemoryDeviceType));


	delete warps_PVA;
	delete warps_VBH;
	delete source_volume_PVA;
	delete source_volume_VBH;
	delete target_PVA;
	delete target_VBH;
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_PVA) {
	GenericWarpVolumeTest<PlainVoxelArray, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_VBH) {
	GenericWarpVolumeTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_VBH_to_PVA) {
	GenericWarpVolume_VBH_to_PVA_Test<MEMORYDEVICE_CPU>(0);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_PVA) {
	GenericWarpVolumeTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_VBH) {
	GenericWarpVolumeTest<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_VBH_to_PVA) {
	GenericWarpVolume_VBH_to_PVA_Test<MEMORYDEVICE_CUDA>(1);
}

#endif