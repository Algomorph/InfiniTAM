//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 10/8/20.
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
#pragma once

//boost
#include <boost/test/test_tools.hpp>

#include "GenericWarpConsistencySubtest.h"
#include "SingleIterationTestConditions.h"
#include "TestCaseOrganizationBySwitches.h"
#include "../TestDataUtilities.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../../../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../../../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../../../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#include "../../../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../../../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#endif

using namespace ITMLib;

namespace test{


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericMultiIterationAlignmentSubtest(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                           LevelSetAlignmentTestMode mode, float absolute_tolerance) {


	std::string volume_filename_prefix = SwitchesToPrefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType,test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* raw_live_volume;
	LoadVolume(&canonical_volume,  test::snoopy::PartialVolume16Path<TIndex>(), TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	LoadVolume(&raw_live_volume, test::snoopy::PartialVolume17Path<TIndex>(), TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	const int source_warped_field_ix = 0;
	const int target_warped_field_ix = 1;

	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[2] = {
			raw_live_volume,
			new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,test::snoopy::InitializationParameters_Fr16andFr17<TIndex>())
	};
	live_volumes[target_warped_field_ix]->Reset();

	AllocateUsingOtherVolume(live_volumes[target_warped_field_ix], live_volumes[source_warped_field_ix], TMemoryDeviceType);
	AllocateUsingOtherVolume(&warp_field, live_volumes[source_warped_field_ix], TMemoryDeviceType);
	AllocateUsingOtherVolume(canonical_volume, live_volumes[source_warped_field_ix], TMemoryDeviceType);

	LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC>
			level_set_alignment_engine(switches, SingleIterationTerminationConditions());

	VoxelVolume<WarpVoxel, TIndex> ground_truth_warp_field(TMemoryDeviceType,
	                                                       test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex> ground_truth_sdf_volume(TMemoryDeviceType,
	                                                       test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	ground_truth_warp_field.Reset();

	DepthFusionEngineInterface<TSDFVoxel, TIndex>* depth_fusion_engine =
			DepthFusionEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);
	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);

	for (int iteration = 0; iteration < iteration_limit; iteration++) {

		std::cout << "Subtest " << IndexString<TIndex>() << " iteration " << std::to_string(iteration) << std::endl;

		level_set_alignment_engine.Align(&warp_field, live_volumes, canonical_volume);

		std::string path = GetWarpsPath<TIndex>(volume_filename_prefix, iteration);
		std::string path_warped_live = GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration);

		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				live_volumes[target_warped_field_ix]->SaveToDisk(path_warped_live);
				warp_field.SaveToDisk(path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				ground_truth_warp_field.Reset();
				ground_truth_warp_field.LoadFromDisk(path);

				BOOST_REQUIRE(ContentAlmostEqual_Verbose(&warp_field, &ground_truth_warp_field, absolute_tolerance,
				                                         TMemoryDeviceType));
				ground_truth_sdf_volume.Reset();
				ground_truth_sdf_volume.LoadFromDisk(path_warped_live);
				BOOST_REQUIRE(ContentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
				                                         absolute_tolerance, TMemoryDeviceType));
				break;
			default:
				break;
		}

		if (iteration < iteration_limit - 1) {
			// prepare for next iteration by swapping source & target (live) TSDF fields
			std::swap(live_volumes[source_warped_field_ix], live_volumes[target_warped_field_ix]);
		}
	}
	std::cout << IndexString<TIndex>() << " fusion test" << std::endl;
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field.SaveToDisk(GetWarpsPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			live_volumes[target_warped_field_ix]->SaveToDisk(GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix], 0);
			canonical_volume->SaveToDisk(GetFusedPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			ground_truth_warp_field.Reset();
			ground_truth_warp_field.LoadFromDisk(GetWarpsPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					ContentAlmostEqual(&warp_field, &ground_truth_warp_field, absolute_tolerance, TMemoryDeviceType));
			ground_truth_sdf_volume.LoadFromDisk(
					GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(ContentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
			                                         absolute_tolerance, TMemoryDeviceType));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix], 0);
			ground_truth_sdf_volume.LoadFromDisk(GetFusedPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(ContentAlmostEqual(canonical_volume, &ground_truth_sdf_volume, absolute_tolerance,
			                                 TMemoryDeviceType));
			break;
		default:
			break;
	}

	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete depth_fusion_engine;
	delete volume_fusion_engine;
}

} // namespace test_utilities