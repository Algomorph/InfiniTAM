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
#define BOOST_TEST_MODULE SurfaceTrackingAuxiliaryFunctions
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#endif

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

BOOST_AUTO_TEST_CASE(Test_ClearOutFramewiseWarp_CPU_PVA){
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	LoadVolume(&warps_PVA, GENERATED_TEST_DATA_PREFIX "TestData/volumes/PVA/warp_field_0_data_framewise_warps.dat",
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Warp::Instance().ComputeWarpUpdateMax(warps_PVA), 0.12124350666999817f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Warp::Instance().ComputeWarpUpdateMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_PVA_CPU->ClearOutWarpUpdates(warps_PVA);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Warp::Instance().ComputeWarpUpdateMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Warp::Instance().ComputeWarpUpdateMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CPU_VBH){
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	LoadVolume(&warps_VBH, GENERATED_TEST_DATA_PREFIX "TestData/volumes/VBH/warp_field_0_data_framewise_warps.dat",
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(Analytics_CPU_VBH_Warp::Instance().ComputeWarpUpdateMax(warps_VBH), 0.12124350666999817f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_VBH_Warp::Instance().ComputeWarpUpdateMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_VBH_CPU->ClearOutWarpUpdates(warps_VBH);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_VBH_Warp::Instance().ComputeWarpUpdateMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_VBH_Warp::Instance().ComputeWarpUpdateMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CUDA_PVA){
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	LoadVolume(&warps_PVA, GENERATED_TEST_DATA_PREFIX "TestData/volumes/PVA/warp_field_0_data_framewise_warps.dat",
	           MEMORYDEVICE_CUDA, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_PVA_Warp::Instance().ComputeWarpUpdateMax(warps_PVA), 0.12124350666999817, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_PVA_Warp::Instance().ComputeWarpUpdateMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_PVA_CUDA->ClearOutWarpUpdates(warps_PVA);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_PVA_Warp::Instance().ComputeWarpUpdateMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_PVA_Warp::Instance().ComputeWarpUpdateMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_AUTO_TEST_CASE(Test_ClearOutWarpUpdate_CUDA_VBH){
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	LoadVolume(&warps_VBH, GENERATED_TEST_DATA_PREFIX "TestData/volumes/VBH/warp_field_0_data_framewise_warps.dat",
	           MEMORYDEVICE_CUDA, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_VBH_Warp::Instance().ComputeWarpUpdateMax(warps_VBH), 0.12124350666999817f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_VBH_Warp::Instance().ComputeWarpUpdateMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_VBH_CUDA->ClearOutWarpUpdates(warps_VBH);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_VBH_Warp::Instance().ComputeWarpUpdateMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(Analytics_CUDA_VBH_Warp::Instance().ComputeWarpUpdateMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}


#endif