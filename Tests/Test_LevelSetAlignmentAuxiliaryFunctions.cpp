//  ================================================================
//  Created by Gregory Kramida on 10/30/19.
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
#define BOOST_TEST_MODULE LevelSetAlignmentAuxiliaryFunctions
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/TestDataUtilities.h"

using namespace ITMLib;
using namespace test;

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericClearOutWarpUpdateTest(){
	VoxelVolume<WarpVoxel, TIndex>* warp_field;
	LoadVolume(&warp_field, std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/warp_field_1_data_tikhonov.dat",
	           TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	float relative_tolerance = 0.1f;//percent
	auto& analytics_engine = AnalyticsEngine<WarpVoxel,TIndex,TMemoryDeviceType>::Instance();
	BOOST_REQUIRE_CLOSE(analytics_engine.ComputeWarpUpdateMax(warp_field), 0.49605023860931396f, relative_tolerance);
	BOOST_REQUIRE_CLOSE(analytics_engine.ComputeWarpUpdateMin(warp_field), 0.0f, relative_tolerance);

	auto alignment_engine = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC>();

	alignment_engine->ClearOutWarpUpdates(warp_field);
	BOOST_REQUIRE_CLOSE(analytics_engine.ComputeWarpUpdateMax(warp_field), 0.0f, relative_tolerance);
	BOOST_REQUIRE_CLOSE(analytics_engine.ComputeWarpUpdateMin(warp_field), 0.0f, relative_tolerance);

	delete warp_field;
}

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CPU_PVA){
	GenericClearOutWarpUpdateTest<PlainVoxelArray, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CPU_VBH){
	GenericClearOutWarpUpdateTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CUDA_PVA){
	GenericClearOutWarpUpdateTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CUDA_VBH){
	GenericClearOutWarpUpdateTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}


#endif