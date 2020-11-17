//  ================================================================
//  Created by Gregory Kramida on 11/7/19.
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

#define BOOST_TEST_MODULE LevelSetAlignment_VBH_vs_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_map>
#include <unordered_set>

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
//(CPU)
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
//(CUDA)
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#endif


//test_utils
#include "TestUtilities/LevelSetAlignment/LevelSetAlignmentTestUtilities.h"
#include "Test_LevelSetAlignment_CPU_vs_CUDA_Aux.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CPU) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, TEST_SUCCESSIVE_ITERATIONS);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CPU) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 25e-6);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CPU) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 25e-6);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CPU) {
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(LevelSetAlignmentSwitches(true, false, true, false, true),
									  5,LevelSetAlignmentTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(LevelSetAlignmentSwitches(true, false, true, false, true),
	                                  5, LevelSetAlignmentTestMode::TEST_FINAL_ITERATION_AND_FUSION);
#endif
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CUDA) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 10, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CUDA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CUDA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 3, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CUDA) {
	GenericWarpTest<MEMORYDEVICE_CUDA>(LevelSetAlignmentSwitches(true, false, true, false, true),
	                                   5, LevelSetAlignmentTestMode::TEST_FINAL_ITERATION_AND_FUSION, 1e-5);
}

#endif

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CPU>(1, switches);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_and_tikhonov) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CPU>(1, switches);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_and_tikhonov_and_sobolev_smoothing) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CPU>(1, switches, 1e-6);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_only) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CUDA>(1, switches);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_and_tikhonov) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CUDA>(1, switches);
}

#endif


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintVolumeStatistics(
		VoxelVolume<TVoxel, TIndex>* volume,
		std::string description) {
	AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>& calculator =
			AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.CountAllocatedVoxels(volume) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.CountNonTruncatedVoxels(volume) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.CountVoxelsWithSpecificSdfValue(volume, 1.0f) << std::endl;
	std::vector<int> allocated_hash_codes = calculator.GetAllocatedHashCodes(volume);
	std::cout << "    Allocated hash count: " << allocated_hash_codes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.SumNonTruncatedVoxelAbsSdf(volume) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.SumTruncatedVoxelAbsSdf(volume) << std::endl;
};

#ifdef TEST_PERFORMANCE
BOOST_AUTO_TEST_CASE(Test_Warp_Performance_CPU) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(&configuration::get().general_voxel_volume_parameters,
													  configuration::get().swapping_mode ==
													  configuration::SWAPPINGMODE_ENABLED,
													  MEMORYDEVICE_CPU,
													  snoopy::InitializationParameters<VoxelBlockHash>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CPU,
													   snoopy::InitializationParameters<VoxelBlockHash>()),
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CPU,
													   snoopy::InitializationParameters<VoxelBlockHash>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();


	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&canonical_volume, &view,
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_depth_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_color_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_omask_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_calib.txt",
												  MEMORYDEVICE_CPU,
												  snoopy::InitializationParameters<VoxelBlockHash>());

	Vector2i image_size = view->depth.dimensions;

	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CPU);

	RenderingEngineBase <TSDFVoxel, VoxelBlockHash>* visualization_engine_legacy =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
							 configuration::get().general_voxel_volume_parameters.far_clipping_distance,
							 MEMORYDEVICE_CPU);

	visualization_engine_legacy->CreateICPMaps(canonical_volume, view, &tracking_state, &render_state);

	updateView(&view, GENERATED_TEST_DATA_PREFIX "TestData/snoopy_depth_000017.png",
			   GENERATED_TEST_DATA_PREFIX "TestData/snoopy_color_000017.png", GENERATED_TEST_DATA_PREFIX "TestData/snoopy_omask_000017.png",
			   GENERATED_TEST_DATA_PREFIX "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	const int live_index_to_start_from = 0;
	reconstructionEngine->GenerateTsdfVolumeFromTwoSurfaces(live_volumes[live_index_to_start_from], view,
															&tracking_state);

	VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& tsdf_calculator =
			VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[live_index_to_start_from], "[initial source live]");

	std::cout << "Utilized (initial source) hash count: " << live_volumes[live_index_to_start_from]->index.GetUtilizedBlockCount() << std::endl;

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(live_volumes[(live_index_to_start_from + 1) % 2],
																									  live_volumes[live_index_to_start_from]);



	std::vector<int> hash_codes_1 = tsdf_calculator.GetAllocatedHashCodes(live_volumes[0]);
	std::vector<int> hash_codes_2 = tsdf_calculator.GetAllocatedHashCodes(live_volumes[1]);
	std::unordered_set<int> hash_codes_1_set(hash_codes_1.begin(), hash_codes_1.end());
	std::unordered_set<int> hash_codes_2_set(hash_codes_2.begin(), hash_codes_2.end());
	for(auto code : hash_codes_1){
		if(hash_codes_2_set.find(code) == hash_codes_2_set.end()){
			std::cout << "Missing block at hash code " << code << " from the second volume." << std::endl;
		}
	}

	int* utilized_codes1_data = live_volumes[0]->index.GetUtilizedBlockHashCodes();
	const int utilized_codes1_count = live_volumes[0]->index.GetUtilizedBlockCount();
	std::unordered_set<int> utilized_codes1_set;
	for(int i_code = 0; i_code < utilized_codes1_count; i_code++){
		utilized_codes1_set.insert(utilized_codes1_data[i_code]);
	}
	for(auto code : hash_codes_1){
		if(utilized_codes1_set.find(code) == utilized_codes1_set.end()){
			std::cout << "Missing block at hash code " << code << " from utilized code set of the first volume." << std::endl;
		}
	}

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(canonical_volume,
																									  live_volumes[live_index_to_start_from]);
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(&warp_field,
																									  live_volumes[live_index_to_start_from]);



	LevelSetAlignmentSwitches switches(true, false, true, false, true);

	LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_OPTIMIZED> motion_tracker(switches);
	WarpingEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* warping_engine =
			WarpingEngineFactory::MakeWarpingEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
	VolumeFusionEngineInterface<TSDFVoxel, VoxelBlockHash>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& warp_calculator =
			VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	const int iteration_limit = 300;
	int source_warped_field_ix = (live_index_to_start_from + 1) % 2;
	int target_warped_field_ix = live_index_to_start_from;
	for (int iteration = 0; iteration < iteration_limit; iteration++) {
		std::swap(source_warped_field_ix, target_warped_field_ix);
		bench::StartTimer("1_CalculateEnergyGradient");
		motion_tracker.CalculateEnergyGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		std::cout << "Warp allocated hash count: " << warp_calculator.ComputeAllocatedHashBlockCount(&warp_field) << std::endl;
		bench::StopTimer("1_CalculateEnergyGradient");
		bench::StartTimer("2_SmoothEnergyGradient");
		motion_tracker.SmoothEnergyGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("2_SmoothEnergyGradient");
		bench::StartTimer("3_UpdateDeformationFieldUsingGradient");
		motion_tracker.UpdateDeformationFieldUsingGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("3_UpdateDeformationFieldUsingGradient");
		bench::StartTimer("4_WarpVolume");
		motion_tracker.CalculateEnergyGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		std::cout << "Warp allocated hash count (2): " << warp_calculator.ComputeAllocatedHashBlockCount(&warp_field) << std::endl;
		PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[target_warped_field_ix], "[target live before warp]");
		std::cout << "Utilized (target) hash count: " << live_volumes[target_warped_field_ix]->index.GetUtilizedBlockCount() << std::endl;
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
											   live_volumes[target_warped_field_ix]);
		PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[target_warped_field_ix], "[target live after warp]");
		std::cout << "Utilized (target) hash count: " << live_volumes[target_warped_field_ix]->index.GetUtilizedBlockCount() << std::endl;
		bench::StopTimer("4_WarpVolume");
	}

	bench::PrintAllCumulativeTimes();

	delete visualization_engine_legacy;
	delete view;
	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete warping_engine;
	delete volume_fusion_engine;
}
#endif

#if !defined(COMPILE_WITHOUT_CUDA) && defined(TEST_PERFORMANCE)
BOOST_AUTO_TEST_CASE(Test_Warp_Performance_CUDA) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(&configuration::get().general_voxel_volume_parameters,
													  configuration::get().swapping_mode ==
													  configuration::SWAPPINGMODE_ENABLED,
													  MEMORYDEVICE_CUDA,
													  snoopy::InitializationParameters<VoxelBlockHash>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CUDA,
													   snoopy::InitializationParameters<VoxelBlockHash>()),
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CUDA,
													   snoopy::InitializationParameters<VoxelBlockHash>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();


	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&canonical_volume, &view,
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_depth_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_color_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_omask_000016.png",
												  GENERATED_TEST_DATA_PREFIX "TestData/snoopy_calib.txt",
												  MEMORYDEVICE_CUDA,
												  snoopy::InitializationParameters<VoxelBlockHash>());

	Vector2i image_size = view->depth.dimensions;

	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CUDA);

	RenderingEngineBase <TSDFVoxel, VoxelBlockHash>* visualization_engine_legacy =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
							 configuration::get().general_voxel_volume_parameters.far_clipping_distance,
							 MEMORYDEVICE_CUDA);

	visualization_engine_legacy->CreateICPMaps(canonical_volume, view, &tracking_state, &render_state);

	updateView(&view, GENERATED_TEST_DATA_PREFIX "TestData/snoopy_depth_000017.png",
			   GENERATED_TEST_DATA_PREFIX "TestData/snoopy_color_000017.png", GENERATED_TEST_DATA_PREFIX "TestData/snoopy_omask_000017.png",
			   GENERATED_TEST_DATA_PREFIX "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	const int live_index_to_start_from = 0;
	reconstructionEngine->GenerateTsdfVolumeFromTwoSurfaces(live_volumes[live_index_to_start_from], view,
															&tracking_state);

	VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& tsdf_calculator =
			VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().AllocateUsingOtherVolume(live_volumes[(live_index_to_start_from + 1) % 2],
																						   live_volumes[live_index_to_start_from]);

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().AllocateUsingOtherVolume(canonical_volume,
																									  live_volumes[live_index_to_start_from]);
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().AllocateUsingOtherVolume(&warp_field,
																									  live_volumes[live_index_to_start_from]);

	LevelSetAlignmentSwitches switches(true, false, true, false, true);

	LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_OPTIMIZED> motion_tracker(switches);
	WarpingEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* warping_engine =
			WarpingEngineFactory::MakeWarpingEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);
	VolumeFusionEngineInterface<TSDFVoxel, VoxelBlockHash>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& warp_calculator =
			VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();

	const int iteration_limit = 300;
	int source_warped_field_ix = (live_index_to_start_from + 1) % 2;
	int target_warped_field_ix = live_index_to_start_from;

	for (int iteration = 0; iteration < iteration_limit; iteration++) {
		std::swap(source_warped_field_ix, target_warped_field_ix);
		bench::StartTimer("1_CalculateEnergyGradient");
		motion_tracker.CalculateEnergyGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("1_CalculateEnergyGradient");
		bench::StartTimer("2_SmoothEnergyGradient");
		motion_tracker.SmoothEnergyGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("2_SmoothEnergyGradient");
		bench::StartTimer("3_UpdateDeformationFieldUsingGradient");
		motion_tracker.UpdateDeformationFieldUsingGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("3_UpdateDeformationFieldUsingGradient");
		bench::StartTimer("4_WarpVolume");
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
											   live_volumes[target_warped_field_ix]);
		bench::StopTimer("4_WarpVolume");
	}

	bench::PrintAllCumulativeTimes();

	delete visualization_engine_legacy;
	delete view;
	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete warping_engine;
	delete volume_fusion_engine;
}

#endif