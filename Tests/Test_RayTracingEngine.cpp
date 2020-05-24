//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/20/20.
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
#define BOOST_TEST_MODULE RaytracingEngine
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"

//ORUtils
#include "../ORUtils/IStreamWrapper.h"

//ITMLib
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
#include "../ITMLib/Utils/Analytics/RawMemoryArrayComparison.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;


template<MemoryDeviceType TMemoryDeviceType>
void GenericFindAndCountVisibleBlocksTest() {
	VisualizationEngine <TSDFVoxel, VoxelBlockHash>* visualization_engine = VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(
			TMemoryDeviceType);

	RGBDCalib calibration_data;
	readRGBDCalib(snoopy::SnoopyCalibrationPath().c_str(), calibration_data);
	RenderState* render_state = new RenderState(Vector2i(640, 480),
	                                            configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                                            configuration::get().general_voxel_volume_parameters.far_clipping_distance,
	                                            TMemoryDeviceType);

	Vector3f target(-0.09150545, 0.07265271, 0.7908916);
	Vector3f original_viewpoint(0.f);
	const int degree_increment = 30;
	std::vector<ORUtils::SE3Pose> camera_poses =
			GenerateCameraTrajectoryAroundPoint(original_viewpoint, target, degree_increment);

	ORUtils::IStreamWrapper visible_blocks_file("TestData/arrays/visible_blocks.dat", true);

	for (auto& pose : camera_poses) {
		VoxelVolume <TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		visualization_engine->FindVisibleBlocks(volume, &pose, &calibration_data.intrinsics_d, render_state);
		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results

		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();
		ORUtils::MemoryBlock<int> visible_codes_ground_truth(visible_block_count, MEMORYDEVICE_CPU);
		ORUtils::MemoryBlockPersistence::LoadMemoryBlock(visible_blocks_file, visible_codes_ground_truth,
		                                                 MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysEqual(visible_codes_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
				visible_codes_device, TMemoryDeviceType, visible_block_count));

		delete volume;
	}

	delete render_state;
	delete visualization_engine;
}


BOOST_AUTO_TEST_CASE(Test_FindVisibleBlocks_CPU) {
	GenericFindAndCountVisibleBlocksTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CountVisibleBlocks) {

}

BOOST_AUTO_TEST_CASE(Test_CreateExpectedDepths) {

}

BOOST_AUTO_TEST_CASE(Test_RenderImage) {

}

BOOST_AUTO_TEST_CASE(Test_FindSurface) {

}

BOOST_AUTO_TEST_CASE(Test_CreatePointCloud) {

}

BOOST_AUTO_TEST_CASE(Test_CreateICPMaps) {

}

BOOST_AUTO_TEST_CASE(Test_ForwardRender) {

}