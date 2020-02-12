//  ================================================================
//  Created by Gregory Kramida on 2/11/20.
//  Copyright (c)  2020 Gregory Kramida
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

#define BOOST_TEST_MODULE HashAllocationThreadSafety
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_set>

//boost
#include <boost/test/unit_test.hpp>
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "TestUtils.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CPU/VolumeStatisticsCalculator_CPU.h"
//(CUDA)
#ifndef COMPILE_WITH_CUDA
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CUDA/VolumeStatisticsCalculator_CUDA.h"
#endif

using namespace ITMLib;

namespace std {
template<>
struct hash<Vector3s> {
	size_t operator()(const Vector3s& vector) const {
		return std::hash<size_t>()(static_cast<size_t>(vector.x) << 16u | static_cast<size_t>(vector.y) << 8u |
		                           static_cast<size_t>(vector.z));
	}
};
} // namespace std

template<MemoryDeviceType TMemoryDeviceType>
struct TestData {
	TestData() {
		view_builder = ViewBuilderFactory::MakeViewBuilder(calibration_path, TMemoryDeviceType);
		auto* rgb = new ITMUChar4Image(true, false);
		auto* depth = new ITMShortImage(true, false);
		ReadImageFromFile(rgb, square_1_color_path.c_str());
		ReadImageFromFile(depth, square_1_depth_path.c_str());
		view_builder->UpdateView(&view_square_1, rgb, depth, false, false, false, true);
		ReadImageFromFile(rgb, square_2_color_path.c_str());
		ReadImageFromFile(depth, square_2_depth_path.c_str());
		view_builder->UpdateView(&view_square_2, rgb, depth, false, false, false, true);
		tracking_state = new CameraTrackingState(depth->noDims, TMemoryDeviceType);
		render_state = new RenderState(depth->noDims,
		                               configuration::get().general_voxel_volume_parameters.near_clipping_distance,
		                               configuration::get().general_voxel_volume_parameters.far_clipping_distance,
		                               TMemoryDeviceType);

		delete rgb;
		delete depth;
	}

	~TestData() {
		delete view_builder;
		delete view_square_1;
		delete view_square_2;
		delete tracking_state;
		delete render_state;
	}

	const std::string calibration_path = "TestData/snoopy_calib.txt";
	const std::string square_1_depth_path = "TestData/square1_depth.png";
	const std::string square_2_depth_path = "TestData/square2_depth.png";
	const std::string square_1_color_path = "TestData/square1_color.png";
	const std::string square_2_color_path = "TestData/square2_color.png";
	ViewBuilder* view_builder;
	ITMView* view_square_1 = nullptr;
	ITMView* view_square_2 = nullptr;
	CameraTrackingState* tracking_state;
	RenderState* render_state;

};

typedef TestData<MEMORYDEVICE_CPU> TestData_CPU;

BOOST_FIXTURE_TEST_CASE(Test_TwoSurfaceAllocation_CPU, TestData_CPU) {


	VoxelVolume<TSDFVoxel, VoxelBlockHash> square_1_volume(MEMORYDEVICE_CPU, {0x8000, 0x20000});
	square_1_volume.Reset();
	DepthFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> depth_fusion_engine;

	depth_fusion_engine.GenerateTsdfVolumeFromView(&square_1_volume, view_square_1, tracking_state);

	std::cout << StatCalc_CPU_VBH_Voxel::Instance().ComputeAllocatedHashBlockCount(&square_1_volume) << std::endl;

	VisualizationEngine<TSDFVoxel, VoxelBlockHash>* visualization_engine = VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(
			MEMORYDEVICE_CPU);

	// builds the point cloud
	visualization_engine->CreateICPMaps(&square_1_volume, view_square_1, tracking_state, render_state);

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	VoxelVolume<TSDFVoxel, VoxelBlockHash> span_volume(MEMORYDEVICE_CPU, {0x8000, 0x20000});
	span_volume.Reset();
	indexer.AllocateNearAndBetweenTwoSurfaces(&span_volume, tracking_state, view_square_2);

	const float distance_to_first_square = 2.0f;
	const float distance_to_second_square = 2.08f;

	int voxel_block_size = span_volume.sceneParams->voxel_size * VOXEL_BLOCK_SIZE;
	short first_line_of_blocks_z = static_cast<short>(std::floor(distance_to_first_square / voxel_block_size));

	std::vector<Vector3s> hash_block_positions_square1 = StatCalc_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(&square_1_volume);
	std::unordered_set<Vector3s> hash_block_positions_square1_set(hash_block_positions_square1.begin(), hash_block_positions_square1.end());
	std::vector<Vector3s> hash_block_positions_span = StatCalc_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(&span_volume);
	std::unordered_set<Vector3s> hash_block_positions_span_set(hash_block_positions_span.begin(), hash_block_positions_span.end());

	bool all_present = true;
	for(auto position : hash_block_positions_square1){
		if(hash_block_positions_span_set.find(position) == hash_block_positions_span_set.end()){
			all_present = false;
		}
	}
	std::cout << (all_present ? "all there" : "not all there") << std::endl;

	std::cout << StatCalc_CPU_VBH_Voxel::Instance().ComputeAllocatedHashBlockCount(&span_volume) << std::endl;

	delete visualization_engine;
}