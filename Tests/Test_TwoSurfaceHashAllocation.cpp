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

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "TestUtilities/TestUtilities.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#endif

//test_utilities
#include "TestUtilities/SnoopyTestUtilities.h"

namespace snoopy = snoopy_test_utilities;

using namespace ITMLib;

template<MemoryDeviceType TMemoryDeviceType>
struct TestData {
	TestData() {
		view_builder = ViewBuilderFactory::Build(calibration_path, TMemoryDeviceType);
		UChar4Image rgb(true, false);
		ShortImage depth(true, false);
		ReadImageFromFile(rgb, square_1_color_path.c_str());
		ReadImageFromFile(depth, square_1_depth_path.c_str());
		view_builder->UpdateView(&view_square_1, &rgb, &depth, false, false, false, true);
		ReadImageFromFile(rgb, square_2_color_path.c_str());
		ReadImageFromFile(depth, square_2_depth_path.c_str());
		view_builder->UpdateView(&view_square_2, &rgb, &depth, false, false, false, true);
		tracking_state = new CameraTrackingState(depth.dimensions, TMemoryDeviceType);
		render_state = new RenderState(depth.dimensions,
		                               configuration::get().general_voxel_volume_parameters.near_clipping_distance,
		                               configuration::get().general_voxel_volume_parameters.far_clipping_distance,
		                               TMemoryDeviceType);

		GenerateGroundTruthPositions();
	}

	~TestData() {
		delete view_builder;
		delete view_square_1;
		delete view_square_2;
		delete tracking_state;
		delete render_state;
	}

	const std::string calibration_path = snoopy::SnoopyCalibrationPath();
	const std::string square_1_depth_path = "TestData/frames/square1_depth.png";
	const std::string square_2_depth_path = "TestData/frames/square2_depth.png";
	const std::string square_1_color_path = "TestData/frames/square1_color.png";
	const std::string square_2_color_path = "TestData/frames/square2_color.png";
	ViewBuilder* view_builder;
	View* view_square_1 = nullptr;
	View* view_square_2 = nullptr;
	CameraTrackingState* tracking_state;
	RenderState* render_state;
	std::unordered_set<Vector3s> ground_truth_block_positions;

private:
	void GenerateGroundTruthPositions(
			const VoxelVolumeParameters& volume_parameters = configuration::get().general_voxel_volume_parameters,
			const float distance_to_first_square = 2.0f, const float distance_to_second_square = 2.096f,
			const float square_size_px = 40) {


		auto voxel_block_size = static_cast<float>(volume_parameters.voxel_size * VOXEL_BLOCK_SIZE);
		auto first_line_of_blocks_z = static_cast<int>(std::ceil((distance_to_first_square -
		                                                          volume_parameters.truncation_distance *
		                                                          volume_parameters.block_allocation_band_factor) /
		                                                         voxel_block_size));
		auto last_line_of_blocks_z = static_cast<int>(std::ceil((distance_to_second_square +
		                                                          volume_parameters.truncation_distance *
		                                                          volume_parameters.block_allocation_band_factor) /
		                                                         voxel_block_size));
		auto horizontal_block_span = static_cast<int>(std::ceil(((square_size_px / 2.0f) * distance_to_first_square /
		                                                         view_square_1->calib.intrinsics_d.projectionParamsSimple.fx) /
		                                                        voxel_block_size)) * 2;
		auto vertical_block_span = static_cast<int>(std::ceil(((square_size_px / 2.0f) * distance_to_first_square /
		                                                       view_square_1->calib.intrinsics_d.projectionParamsSimple.fy) /
		                                                      voxel_block_size)) * 2;
		auto depth_block_span = last_line_of_blocks_z + 1 - first_line_of_blocks_z;


		int start_x = -horizontal_block_span / 2;
		int end_x = start_x + horizontal_block_span;
		int start_y = -vertical_block_span / 2;
		int end_y = start_y + vertical_block_span;
		int end_z = first_line_of_blocks_z + depth_block_span;


		for (int x = start_x; x < end_x; x++) {
			for (int y = start_y; y < end_y; y++) {
				for (int z = first_line_of_blocks_z; z < end_z; z++) {
					Vector3s position(x, y, z);
					ground_truth_block_positions.insert(position);
				}
			}
		}
	}

};

typedef TestData<MEMORYDEVICE_CPU> TestData_CPU;

static void check_positions(const std::unordered_set<Vector3s>& ground_truth_block_positions,
                            const std::unordered_set<Vector3s>& hash_block_positions_span_set,
                            const std::vector<Vector3s> hash_block_positions_span) {

	bool bad_block_detected = false;
	Vector3s bad_block(-1);
	for (auto block_position : hash_block_positions_span) {
		if (ground_truth_block_positions.find(block_position) == ground_truth_block_positions.end()) {
			bad_block = block_position;
			bad_block_detected = true;
		}
	}
	BOOST_REQUIRE_MESSAGE(!bad_block_detected, "Detected incorrect block allocated at " << bad_block << ".");
	for (auto block_position : ground_truth_block_positions) {
		if (hash_block_positions_span_set.find(block_position) == hash_block_positions_span_set.end()) {
			bad_block = block_position;
			bad_block_detected = true;
		}
	}
	BOOST_REQUIRE_MESSAGE(!bad_block_detected, "Missing block at " << bad_block << ".");
}

BOOST_FIXTURE_TEST_CASE(Test_TwoSurfaceAllocation_CPU, TestData_CPU) {


	VoxelVolume<TSDFVoxel, VoxelBlockHash> square_volume(MEMORYDEVICE_CPU, {0x8000, 0x20000});
	square_volume.Reset();
	DepthFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> depth_fusion_engine;

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer.AllocateNearSurface(&square_volume, view_square_1, tracking_state);
	depth_fusion_engine.IntegrateDepthImageIntoTsdfVolume(&square_volume, view_square_1, tracking_state);

	RenderingEngineBase<TSDFVoxel, VoxelBlockHash>* visualization_engine = RenderingEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(
			MEMORYDEVICE_CPU);

	// builds the point cloud
	visualization_engine->CreateICPMaps(&square_volume, view_square_1, tracking_state, render_state);

	VoxelVolume<TSDFVoxel, VoxelBlockHash> span_volume(MEMORYDEVICE_CPU, {0x8000, 0x20000});
	span_volume.Reset();
	indexer.AllocateNearAndBetweenTwoSurfaces(&span_volume, view_square_2, tracking_state);

	std::vector<Vector3s> hash_block_positions_span = Analytics_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&span_volume);
	std::unordered_set<Vector3s> hash_block_positions_span_set(hash_block_positions_span.begin(),
	                                                           hash_block_positions_span.end());
	int test_volume_block_count = Analytics_CPU_VBH_Voxel::Instance().CountAllocatedHashBlocks(&span_volume);

	BOOST_REQUIRE_EQUAL(test_volume_block_count, ground_truth_block_positions.size());

	check_positions(ground_truth_block_positions, hash_block_positions_span_set, hash_block_positions_span);

	//reverse square order
	square_volume.Reset();
	span_volume.Reset();
	tracking_state->Reset();


	indexer.AllocateNearAndBetweenTwoSurfaces(&square_volume, view_square_2, tracking_state);
	depth_fusion_engine.IntegrateDepthImageIntoTsdfVolume(&square_volume, view_square_2, tracking_state);
	visualization_engine->CreateICPMaps(&square_volume, view_square_2, tracking_state, render_state);
	indexer.AllocateNearAndBetweenTwoSurfaces(&span_volume, view_square_1, tracking_state);

	hash_block_positions_span = Analytics_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&span_volume);
	hash_block_positions_span_set = std::unordered_set<Vector3s>(hash_block_positions_span.begin(),
	                                                             hash_block_positions_span.end());
	test_volume_block_count = Analytics_CPU_VBH_Voxel::Instance().CountAllocatedHashBlocks(&span_volume);

	BOOST_REQUIRE_EQUAL(test_volume_block_count, ground_truth_block_positions.size());

	check_positions(ground_truth_block_positions, hash_block_positions_span_set, hash_block_positions_span);

	delete visualization_engine;
}

#ifndef COMPILE_WITHOUT_CUDA
typedef TestData<MEMORYDEVICE_CUDA> TestData_CUDA;

BOOST_FIXTURE_TEST_CASE(Test_TwoSurfaceAllocation_CUDA, TestData_CUDA) {


	VoxelVolume<TSDFVoxel, VoxelBlockHash> square_volume(MEMORYDEVICE_CUDA, {0x8000, 0x20000});
	square_volume.Reset();
	DepthFusionEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> depth_fusion_engine;

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	indexer.AllocateNearSurface(&square_volume, view_square_1, tracking_state);
	depth_fusion_engine.IntegrateDepthImageIntoTsdfVolume(&square_volume, view_square_1, tracking_state);

	RenderingEngineBase<TSDFVoxel, VoxelBlockHash>* visualization_engine = RenderingEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(
			MEMORYDEVICE_CUDA);

	// builds the point cloud
	visualization_engine->CreateICPMaps(&square_volume, view_square_1, tracking_state, render_state);

	VoxelVolume<TSDFVoxel, VoxelBlockHash> span_volume(MEMORYDEVICE_CUDA, {0x8000, 0x20000});
	span_volume.Reset();
	indexer.AllocateNearAndBetweenTwoSurfaces(&span_volume, view_square_2, tracking_state);

	std::vector<Vector3s> hash_block_positions_span = Analytics_CUDA_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&span_volume);
	std::unordered_set<Vector3s> hash_block_positions_span_set(hash_block_positions_span.begin(),
	                                                           hash_block_positions_span.end());

	int test_volume_block_count = Analytics_CUDA_VBH_Voxel::Instance().CountAllocatedHashBlocks(&span_volume);

	BOOST_REQUIRE_EQUAL(test_volume_block_count, ground_truth_block_positions.size());

	check_positions(ground_truth_block_positions, hash_block_positions_span_set, hash_block_positions_span);

	//reverse square order
	square_volume.Reset();
	span_volume.Reset();
	tracking_state->Reset();

	indexer.AllocateNearAndBetweenTwoSurfaces(&square_volume, view_square_2, tracking_state);
	depth_fusion_engine.IntegrateDepthImageIntoTsdfVolume(&square_volume, view_square_2, tracking_state);
	visualization_engine->CreateICPMaps(&square_volume, view_square_2, tracking_state, render_state);
	indexer.AllocateNearAndBetweenTwoSurfaces(&span_volume, view_square_1, tracking_state);

	hash_block_positions_span = Analytics_CUDA_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&span_volume);
	hash_block_positions_span_set = std::unordered_set<Vector3s>(hash_block_positions_span.begin(),
	                                                             hash_block_positions_span.end());
	test_volume_block_count = Analytics_CUDA_VBH_Voxel::Instance().CountAllocatedHashBlocks(&span_volume);

	BOOST_REQUIRE_EQUAL(test_volume_block_count, ground_truth_block_positions.size());

	check_positions(ground_truth_block_positions, hash_block_positions_span_set, hash_block_positions_span);

	delete visualization_engine;
}

#endif