//  ================================================================
//  Created by Gregory Kramida on 12/8/19.
//  Copyright (c)  2019 Gregory Kramida
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

#define BOOST_TEST_MODULE SmoothEnergyGradient
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif


//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


template <typename TIndex>
void GenericTestCompareSmoothEnergyGradient_CPU_to_CUDA() {
		const int iteration = 0;

		LevelSetAlignmentSwitches data_only_switches(true, false, false, false, false);
		std::string path_warps = GetWarpsPath<TIndex>(SwitchesToPrefix(data_only_switches), iteration);
		std::string path_frame_16 = snoopy::PartialVolume16Path<TIndex>();
		std::string path_frame_17 = snoopy::PartialVolume17Path<TIndex>();

		VoxelVolume<WarpVoxel, TIndex>* warp_field_CPU;
		LoadVolume(&warp_field_CPU, path_warps, MEMORYDEVICE_CPU,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());
		VoxelVolume<TSDFVoxel, TIndex>* canonical_volume_CPU;
		LoadVolume(&canonical_volume_CPU, path_frame_17, MEMORYDEVICE_CPU,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());
		VoxelVolume<TSDFVoxel, TIndex>* live_volume_CPU;
		LoadVolume(&live_volume_CPU, path_frame_16, MEMORYDEVICE_CPU,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());

		VoxelVolume<WarpVoxel, TIndex>* warp_field_CUDA;
		LoadVolume(&warp_field_CUDA, path_warps, MEMORYDEVICE_CUDA,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());
		VoxelVolume<TSDFVoxel, TIndex>* canonical_volume_CUDA;
		LoadVolume(&canonical_volume_CUDA, path_frame_17, MEMORYDEVICE_CUDA,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());
		VoxelVolume<TSDFVoxel, TIndex>* live_volume_CUDA;
		LoadVolume(&live_volume_CUDA, path_frame_16, MEMORYDEVICE_CUDA,
		snoopy::InitializationParameters_Fr16andFr17<TIndex>());


		auto motion_tracker_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, MEMORYDEVICE_CPU, DIAGNOSTIC>(
		LevelSetAlignmentSwitches(false, false, false, false, true));
		auto motion_tracker_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
		LevelSetAlignmentSwitches(false, false, false, false, true)
		);

		VoxelVolume<TSDFVoxel, TIndex>* live_volumes_CPU[2] = {live_volume_CPU, nullptr};
		VoxelVolume<TSDFVoxel, TIndex>* live_volumes_CUDA[2] = {live_volume_CUDA, nullptr};

		motion_tracker_CPU->Align(warp_field_CPU, live_volumes_CPU, canonical_volume_CPU);
		motion_tracker_CUDA->Align(warp_field_CUDA, live_volumes_CUDA, canonical_volume_CUDA);

		// generate a CPU version of the CUDA output for comparison with the CPU output
		VoxelVolume<WarpVoxel, TIndex> warp_field_CUDA_copy(*warp_field_CUDA, MEMORYDEVICE_CPU);

		float tolerance = 1e-6;
		BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CUDA_copy, warp_field_CPU, tolerance));

		delete motion_tracker_CPU;
		delete motion_tracker_CUDA;

		delete warp_field_CPU;
		delete warp_field_CUDA;
		delete canonical_volume_CPU;
		delete canonical_volume_CUDA;
		delete live_volume_CPU;
		delete live_volume_CUDA;
};

BOOST_AUTO_TEST_CASE(CompareSmoothEnergyGradient_CPU_to_CUDA_PVA){
	GenericTestCompareSmoothEnergyGradient_CPU_to_CUDA<PlainVoxelArray>();
}

BOOST_AUTO_TEST_CASE(CompareSmoothEnergyGradient_CPU_to_CUDA_VBH){
	GenericTestCompareSmoothEnergyGradient_CPU_to_CUDA<VoxelBlockHash>();
}