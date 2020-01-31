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

//local
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "TestUtils.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/SceneStatisticsCalculator_CPU.h"
#include "TestUtilsForSnoopyFrames16And17.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#endif

using namespace ITMLib;

BOOST_FIXTURE_TEST_CASE(Test_ClearOutFramewiseWarp_CPU_PVA, Frame16And17Fixture){
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	loadVolume(&warps_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_framewise_warps_", MEMORYDEVICE_CPU,
			InitParams<PlainVoxelArray>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFramewiseWarpMax(warps_PVA), 0.18186526000499725f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFramewiseWarpMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_PVA_CPU->ClearOutFramewiseWarp(warps_PVA);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFramewiseWarpMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Warp::Instance().ComputeFramewiseWarpMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_FIXTURE_TEST_CASE(Test_ClearOutFramewiseWarp_CPU_VBH, Frame16And17Fixture){
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_data_framewise_warps_",
			MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFramewiseWarpMax(warps_VBH), 0.18186526000499725f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFramewiseWarpMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_VBH_CPU->ClearOutFramewiseWarp(warps_VBH);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFramewiseWarpMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_VBH_Warp::Instance().ComputeFramewiseWarpMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_FIXTURE_TEST_CASE(Test_ClearOutFramewiseWarp_CUDA_PVA, Frame16And17Fixture){
	ITMVoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	loadVolume(&warps_PVA, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_framewise_warps_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFramewiseWarpMax(warps_PVA), 0.18186526000499725f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFramewiseWarpMin(warps_PVA), 0.0f, relativeTolerance);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_PVA_CUDA->ClearOutFramewiseWarp(warps_PVA);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFramewiseWarpMax(warps_PVA), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_PVA_Warp::Instance().ComputeFramewiseWarpMin(warps_PVA), 0.0f, relativeTolerance);

	delete warps_PVA;
}

BOOST_FIXTURE_TEST_CASE(Test_ClearOutFramewiseWarp_CUDA_VBH, Frame16And17Fixture){
	ITMVoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_data_framewise_warps_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	float relativeTolerance = 0.1f;//percent
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFramewiseWarpMax(warps_VBH), 0.18186526000499725f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFramewiseWarpMin(warps_VBH), 0.0f, relativeTolerance);

	auto motionTracker_VBH_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>();

	motionTracker_VBH_CUDA->ClearOutFramewiseWarp(warps_VBH);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFramewiseWarpMax(warps_VBH), 0.0f, relativeTolerance);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CUDA_VBH_Warp::Instance().ComputeFramewiseWarpMin(warps_VBH), 0.0f, relativeTolerance);

	delete warps_VBH;
}


#endif