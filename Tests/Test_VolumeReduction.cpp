//  ================================================================
//  Created by Gregory Kramida on 2/24/20.
//  Copyright (c) 2020 Gregory Kramida
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
#define BOOST_TEST_MODULE VolumeReduction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


template<typename TIndex>
typename TIndex::InitializationParameters GetTestSpecificInitializationParameters();

template<>
VoxelBlockHash::InitializationParameters GetTestSpecificInitializationParameters<VoxelBlockHash>() {
	return {65536, 32768};
}


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericVolumeReductionCountWeightRangeTest1() {
	VoxelVolume<TSDFVoxel, TIndex> volume(TMemoryDeviceType, GetTestSpecificInitializationParameters<TIndex>());
	volume.Reset();

	Extent3Di filled_voxel_bounds(0, 0, 0, 48, 48, 48);
	Extent2Di general_range(0, 50);
	GenerateRandomDepthWeightSubVolume<TMemoryDeviceType>(&volume, filled_voxel_bounds, general_range);

	const unsigned int voxel_count = filled_voxel_bounds.max_x * filled_voxel_bounds.max_y * filled_voxel_bounds.max_z;

	unsigned int voxels_in_range;
	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, general_range);
	BOOST_REQUIRE_EQUAL(voxel_count, voxels_in_range);

	Extent2Di different_range1(50, 56);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	Extent2Di different_range2(56, 100);
	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	TSDFVoxel alternative_weight_voxel;
	alternative_weight_voxel.w_depth = 55;
	Vector3i insert_position(12, 12, 12);
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {17, 0, 14};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {45, 16, 34};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);

	BOOST_REQUIRE_EQUAL(3, voxels_in_range);

	Extent3Di different_weight_sub_volume_bounds(42, 42, 42, 48, 48, 48);
	Vector3i sub_volume_size = different_weight_sub_volume_bounds.max() - different_weight_sub_volume_bounds.min();
	const unsigned int sub_volume_voxel_count = sub_volume_size.x * sub_volume_size.y * sub_volume_size.z;
	GenerateRandomDepthWeightSubVolume<TMemoryDeviceType>(&volume, different_weight_sub_volume_bounds,
	                                                      different_range2);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);

	BOOST_REQUIRE_EQUAL(sub_volume_voxel_count, voxels_in_range);
}


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericVolumeReductionCountWeightRangeTest2() {
	VoxelVolume<TSDFVoxel, TIndex> volume(TMemoryDeviceType, GetTestSpecificInitializationParameters<TIndex>());
	volume.Reset();

	Extent3Di filled_voxel_bounds(0, 0, 0, 256, 256, 256);
	Extent2Di general_range(0, 50);
	GenerateRandomDepthWeightSubVolume<TMemoryDeviceType>(&volume, filled_voxel_bounds, general_range);

	const unsigned int voxel_count = filled_voxel_bounds.max_x * filled_voxel_bounds.max_y * filled_voxel_bounds.max_z;

	unsigned int voxels_in_range;
	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, general_range);

	BOOST_REQUIRE_EQUAL(voxel_count, voxels_in_range);

	Extent2Di different_range1(50, 56);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	Extent2Di different_range2(56, 67);
	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	TSDFVoxel alternative_weight_voxel;
	alternative_weight_voxel.w_depth = 55;
	Vector3i insert_position(12, 12, 12);
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {17, 0, 14};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {123, 53, 234};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);

	BOOST_REQUIRE_EQUAL(3, voxels_in_range);

	Extent3Di different_weight_sub_volume_bounds(128, 128, 128, 256, 256, 256);
	Vector3i sub_volume_size = different_weight_sub_volume_bounds.max() - different_weight_sub_volume_bounds.min();
	const unsigned int sub_volume_voxel_count = sub_volume_size.x * sub_volume_size.y * sub_volume_size.z;
	Vector3i sub_volume_block_size = sub_volume_size / VOXEL_BLOCK_SIZE;
	const unsigned int sub_volume_hash_block_count =
			sub_volume_block_size.x * sub_volume_block_size.y * sub_volume_block_size.z;
	GenerateRandomDepthWeightSubVolume<TMemoryDeviceType>(&volume, different_weight_sub_volume_bounds,
	                                                      different_range2);

	voxels_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);
	BOOST_REQUIRE_EQUAL(sub_volume_voxel_count, voxels_in_range);

	unsigned int blocks_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountHashBlocksWithDepthWeightInRange(&volume, different_range2);

	BOOST_REQUIRE_EQUAL(sub_volume_hash_block_count, blocks_in_range);

	alternative_weight_voxel.w_depth = 100;
	insert_position = {128, 128, 128};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);

	insert_position = {167, 184, 135};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {189, 200, 234};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>()
			.SetVoxel(&volume, insert_position, alternative_weight_voxel);

	Extent2Di different_range3(100, 101);
	blocks_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountHashBlocksWithDepthWeightInRange(&volume, different_range3);

	BOOST_REQUIRE_EQUAL(0u, blocks_in_range);

	blocks_in_range = AnalyticsEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.CountHashBlocksWithDepthWeightInRange(&volume, different_range2);

	BOOST_REQUIRE_EQUAL(sub_volume_hash_block_count - 3u, blocks_in_range);
}


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericTestVolumeReductionMaxWarpUpdate() {
	const int iteration = 1;

	// *** load warps
	LevelSetAlignmentSwitches data_tikhonov_sobolev_switches(true, false, true, false, true);
	std::string path_warps = GetWarpsPath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration);

	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps, path_warps, TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	float value_gt = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMax(warps);

	float max_value;
	Vector3i position;
	AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMaxAndPosition(max_value, position, warps);

	BOOST_REQUIRE_EQUAL(value_gt, max_value);

	WarpVoxel voxel = warps->GetValueAt(position);

	BOOST_REQUIRE_EQUAL(ORUtils::length(voxel.warp_update), max_value);

// #define TEST_PERFORMANCE
#ifdef TEST_PERFORMANCE
	TimeIt(
		[&]() {
			AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMax(warps);
		}, "Warp Update Max (Atomics)", 100
	);

	TimeIt(
		[&]() {
			AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMaxAndPosition(max_value, position, warps);
		}, "Warp Update Max (Reduciton)", 100
	);
#endif

	delete warps;
}



BOOST_AUTO_TEST_CASE(Test_VolumeReduction_CountWeightRange1_VBH_CPU) {
	GenericVolumeReductionCountWeightRangeTest1<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_CountWeightRange2_VBH_CPU) {
	GenericVolumeReductionCountWeightRangeTest2<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_MaxWarpUpdate_VBH_CPU) {
	GenericTestVolumeReductionMaxWarpUpdate<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_CountWeightRange1_VBH_CUDA) {
	GenericVolumeReductionCountWeightRangeTest1<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_CountWeightRange2_VBH_CUDA) {
	GenericVolumeReductionCountWeightRangeTest2<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_MaxWarpUpdate_VBH_CUDA) {
	GenericTestVolumeReductionMaxWarpUpdate<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

#endif

template<typename TIndex>
typename TIndex::InitializationParameters GetTestLargeVolumeInitializationParameters();

template<>
VoxelBlockHash::InitializationParameters GetTestLargeVolumeInitializationParameters<VoxelBlockHash>() {
	return {0x40000, 0x20000};
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericTestVolumeReductionPerformance() {
	VoxelVolume<WarpVoxel, TIndex> volume(TMemoryDeviceType, GetTestLargeVolumeInitializationParameters<TIndex>());
	volume.Reset();
	Extent3Di bounds{-256,-256,0,256,256,512};
	IndexingEngineFactory::GetDefault<WarpVoxel, TIndex>(TMemoryDeviceType).AllocateGridAlignedBox(&volume, bounds);
	WarpVoxel voxel;
	float max_value_gt = 100.0f;
	voxel.warp_update = Vector3f(100.0f, 0, 0.0);
	Vector3i position_gt{0, 0, 256};
	volume.SetValueAt(position_gt, voxel);

	Vector3i position;
	float max_value;

	TimeIt(
			[&]() {
				AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMaxAndPosition(max_value, position, &volume);
			}, "Warp Update Max (Atomics)", 10
	);
	BOOST_REQUIRE_EQUAL(max_value_gt, max_value);
	BOOST_REQUIRE_EQUAL(position, position_gt);

}

BOOST_AUTO_TEST_CASE(Test_VolumeReduction_Performance_VBH_CUDA) {
	GenericTestVolumeReductionPerformance<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}