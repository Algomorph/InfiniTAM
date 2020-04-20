//  ================================================================
//  Created by Gregory Kramida on 10/17/19.
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
#define BOOST_TEST_MODULE VoxelVolumeSlicing
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>


//ITMLib
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#endif

//test_utilities
#include "TestUtilities.h"
#include "TestUtilsForSnoopyFrames16And17.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy16and17utilities;

//#define GET_SCENE_BOUNDS
#ifdef GET_SCENE_BOUNDS
BOOST_AUTO_TEST_CASE(GetSceneBounds){
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_vbh_CPU(MEMORYDEVICE_CPU);
	volume_vbh_CPU.Reset();
	volume_vbh_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");
	std::cout << "BOUNDS(16/VBH):  " << Analytics_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(&volume_vbh_CPU) << std::endl;
	volume_vbh_CPU.Reset();
	volume_vbh_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_17_");
	std::cout << "BOUNDS(17/VBH):  " << Analytics_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(&volume_vbh_CPU) << std::endl;
	#ifndef COMPILE_WITHOUT_CUDA
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_pva_CUDA(MEMORYDEVICE_CUDA);
	volume_pva_CUDA.Reset();
	volume_pva_CUDA.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");
	std::cout << "ALTERED BOUNDS(16/PVA): " << Analytics_CUDA_PVA_Voxel::Instance().ComputeAlteredVoxelBounds(&volume_pva_CUDA) << std::endl;
	#else
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_pva_CPU(MEMORYDEVICE_CPU);
	volume_pva_CPU.Reset();
	volume_pva_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");
	std::cout << "ALTERED BOUNDS(16/PVA): " << Analytics_CPU_PVA_Voxel::Instance().ComputeAlteredVoxelBounds(&volume_pva_CPU) << std::endl;
	#endif
}
#endif

BOOST_AUTO_TEST_CASE(testPVASceneSlice_CPU) {
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_CPU(MEMORYDEVICE_CPU);
	volume_CPU.Reset();

	const int expected_non_truncated_voxel_count = 58368;
	volume_CPU.LoadFromDisk("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");

	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(&volume_CPU),
	                    expected_non_truncated_voxel_count);

	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(&volume_CPU),
	                    28887.87700, 0.001);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_slice_same_dimensions_CPU(
			MEMORYDEVICE_CPU);
	volume_slice_same_dimensions_CPU.Reset();

	Vector6i bounds(-66, -24, 159, 15, 72, 311);
	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&volume_slice_same_dimensions_CPU,
	                                                         &volume_CPU, bounds);

	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(
			&volume_slice_same_dimensions_CPU), expected_non_truncated_voxel_count);

	float tolerance = 1e-8;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(
			&volume_CPU, &volume_slice_same_dimensions_CPU, tolerance));

	bounds = Vector6i(-72, -24, 160, 16, 80, 320);
	Vector3i offsetSlice(bounds.min_x, bounds.min_y, bounds.min_z);
	Vector3i sizeSlice(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y, bounds.max_z - bounds.min_z);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_slice_different_dimensions_CPU(
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&volume_slice_different_dimensions_CPU);

	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&volume_slice_different_dimensions_CPU,
	                                                         &volume_CPU, bounds);
	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(
			&volume_slice_different_dimensions_CPU), expected_non_truncated_voxel_count);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(
			&volume_slice_different_dimensions_CPU), 28887.877, 0.001);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(
			&volume_CPU), 28887.877, 0.001);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_CPU,
	                                                      &volume_slice_different_dimensions_CPU, tolerance));

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_slice_from_disk_CPU(
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	volume_slice_from_disk_CPU.Reset();

	volume_slice_from_disk_CPU.LoadFromDisk("TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_");

	BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_slice_different_dimensions_CPU,
	                                     &volume_slice_from_disk_CPU, tolerance));


}