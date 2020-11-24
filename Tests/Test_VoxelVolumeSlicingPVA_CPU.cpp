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
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
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

BOOST_AUTO_TEST_CASE(testPVASceneSlice_CPU) {
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_CPU(MEMORYDEVICE_CPU);
	volume_CPU.Reset();

	const int expected_non_truncated_voxel_count = 58785;
	volume_CPU.LoadFromDisk(snoopy::FullVolume16Path<PlainVoxelArray>());

	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(&volume_CPU),
	                    expected_non_truncated_voxel_count);


	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(&volume_CPU),
	                    29304.876607656479, 0.001);

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

	auto partial_volume_info = snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>();

	bounds = Vector6i(
			partial_volume_info.offset.x,
			partial_volume_info.offset.y,
			partial_volume_info.offset.z,
			partial_volume_info.offset.x + partial_volume_info.size.x,
			partial_volume_info.offset.y + partial_volume_info.size.y,
			partial_volume_info.offset.z + partial_volume_info.size.z
	);


	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_slice_different_dimensions_CPU(
			MEMORYDEVICE_CPU, partial_volume_info);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&volume_slice_different_dimensions_CPU);

	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&volume_slice_different_dimensions_CPU,
	                                                         &volume_CPU, bounds);
	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(
			&volume_slice_different_dimensions_CPU), expected_non_truncated_voxel_count);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(
			&volume_slice_different_dimensions_CPU), 29304.876607656479, 0.001);
	BOOST_REQUIRE_CLOSE(Analytics_CPU_PVA_Voxel::Instance().SumNonTruncatedVoxelAbsSdf(
			&volume_CPU), 29304.876607656479, 0.001);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_CPU,
	                                                      &volume_slice_different_dimensions_CPU, tolerance));

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_slice_from_disk_CPU(MEMORYDEVICE_CPU, partial_volume_info);
	volume_slice_from_disk_CPU.Reset();

	volume_slice_from_disk_CPU.LoadFromDisk(snoopy::PartialVolume16Path<PlainVoxelArray>());

	BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_slice_different_dimensions_CPU,
	                                     &volume_slice_from_disk_CPU, tolerance));


}