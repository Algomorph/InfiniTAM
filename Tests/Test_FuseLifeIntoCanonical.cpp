//  ================================================================
//  Created by Gregory Kramida on 11/4/19.
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
#define BOOST_TEST_MODULE FuseLiveIntoCanonical
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngine.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
//(CPU)
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
void GenericFusionTest(const int iteration = 4) {
	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume;
	SlavchevaSurfaceTracker::Switches data_tikhonov_sobolev_switches(true, false, true, false, true);
	LoadVolume(&warped_live_volume,
	           GetWarpedLivePath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	IndexingEngine<TSDFVoxel,TIndex,MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(canonical_volume, warped_live_volume);

	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine = VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(
			TMemoryDeviceType);
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume, 0);

	VoxelVolume<TSDFVoxel, TIndex>* fused_canonical_volume_gt;
	LoadVolume(&fused_canonical_volume_gt,
	           GetFusedPath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	float absoluteTolerance = 1e-7;

	BOOST_REQUIRE(
			contentAlmostEqual(fused_canonical_volume_gt, canonical_volume, absoluteTolerance, TMemoryDeviceType));
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_PVA) {
	GenericFusionTest<PlainVoxelArray, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CPU_VBH) {
	GenericFusionTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_PVA) {
	GenericFusionTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_CUDA_VBH) {
	GenericFusionTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}


template<MemoryDeviceType TMemoryDeviceType>
void GenericFusion_PVA_to_VBH_Test(const int iteration = 4) {
	SlavchevaSurfaceTracker::Switches data_tikhonov_sobolev_switches(true, false, true, false, true);

	// *** load PVA stuff
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_volume_PVA;
	LoadVolume(&warped_live_volume_PVA, GetWarpedLivePath<PlainVoxelArray>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume_PVA;
	LoadVolume(&canonical_volume_PVA, snoopy::PartialVolume16Path<PlainVoxelArray>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	// *** load VBH stuff
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_volume_VBH;
	LoadVolume(&warped_live_volume_VBH, GetWarpedLivePath<VoxelBlockHash>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume_VBH;
	LoadVolume(&canonical_volume_VBH, snoopy::PartialVolume16Path<VoxelBlockHash>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>::Instance().AllocateFromOtherVolume(
			canonical_volume_VBH, warped_live_volume_VBH);

	VolumeFusionEngineInterface<TSDFVoxel, PlainVoxelArray>* volume_fusion_engine_PVA =
			VolumeFusionEngineFactory::Build<TSDFVoxel, PlainVoxelArray>(TMemoryDeviceType);
	VolumeFusionEngineInterface<TSDFVoxel, VoxelBlockHash>* volume_fusion_engine_VBH =
			VolumeFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(TMemoryDeviceType);

	volume_fusion_engine_PVA->FuseOneTsdfVolumeIntoAnother(canonical_volume_PVA, warped_live_volume_PVA, 0);
	volume_fusion_engine_VBH->FuseOneTsdfVolumeIntoAnother(canonical_volume_VBH, warped_live_volume_VBH, 0);
	float absoluteTolerance = 1e-7;
	//(uncomment for a less rigorous test)
	//BOOST_REQUIRE( allocatedContentAlmostEqual_Verbose(canonical_volume_PVA, canonical_volume_VBH, absoluteTolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentForFlagsAlmostEqual(canonical_volume_PVA, canonical_volume_VBH, VOXEL_NONTRUNCATED,
	                                         absoluteTolerance, TMemoryDeviceType));
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_PVA_to_VBH_CPU) {
	GenericFusion_PVA_to_VBH_Test<MEMORYDEVICE_CPU>(4);
}

BOOST_AUTO_TEST_CASE(Test_FuseLifeIntoCanonical_PVA_to_VBH_CUDA) {
	GenericFusion_PVA_to_VBH_Test<MEMORYDEVICE_CUDA>(4);
}


#endif