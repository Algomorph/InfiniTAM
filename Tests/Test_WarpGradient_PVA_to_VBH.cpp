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

#define BOOST_TEST_MODULE WarpGradient_VBH_to_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_map>
#include <unordered_set>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"
#include "WarpAdvancedTestingUtilities.h"

#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"

//local CPU
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"

#ifndef COMPILE_WITHOUT_CUDA
//local CUDA
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

#endif


using namespace ITMLib;

///CAUTION: SAVE modes require the build directory to be immediately inside the root source directory.
template<MemoryDeviceType TMemoryDeviceType>
void
GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit = 10,
                GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS, float absoluteTolerance = 1e-7) {

	std::string prefix = switches_to_prefix(switches);
	GenericWarpConsistencySubtest<PlainVoxelArray, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                  absoluteTolerance);
	GenericWarpConsistencySubtest<VoxelBlockHash, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                 absoluteTolerance);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA(TMemoryDeviceType,
	                                                   Frame16And17Fixture::InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH(TMemoryDeviceType,
	                                                  Frame16And17Fixture::InitParams<VoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_PVA(TMemoryDeviceType,
			                                                       Frame16And17Fixture::InitParams<PlainVoxelArray>());
			VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_VBH(TMemoryDeviceType,
			                                                      Frame16And17Fixture::InitParams<VoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_PVA.LoadFromDisk(get_path_warps(prefix, iteration));
				EditAndCopyEngineFactory::Instance<WarpVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetVolume(
						&warp_field_VBH);
				warp_field_VBH.LoadFromDisk(get_path_warps(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(&warp_field_PVA, &warp_field_VBH,
				                                                  absoluteTolerance, TMemoryDeviceType));
				EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetVolume(
						&volume_VBH);
				volume_PVA.LoadFromDisk(get_path_warped_live(prefix, iteration));
				volume_VBH.LoadFromDisk(get_path_warped_live(prefix, iteration));
				BOOST_REQUIRE(contentForFlagsAlmostEqual_Verbose(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                                 absoluteTolerance, TMemoryDeviceType));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			volume_PVA.LoadFromDisk(get_path_warped_live(prefix, iteration_limit - 1));
			EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetVolume(
					&volume_VBH);
			volume_VBH.LoadFromDisk(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
			volume_PVA.LoadFromDisk(get_path_fused(prefix, iteration_limit - 1));
			EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>().ResetVolume(
					&volume_VBH);
			volume_VBH.LoadFromDisk(get_path_fused(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absoluteTolerance,
					                           TMemoryDeviceType));
		}
			break;
		default:
			break;
	}
}

//#define GENERATE_TEST_DATA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 10, TEST_SUCCESSIVE_ITERATIONS);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CPU) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, SAVE_SUCCESSIVE_ITERATIONS);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-7);
#endif
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CPU) {
#ifdef GENERATE_TEST_DATA
	GenericWarpTest<MEMORYDEVICE_CPU>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
									  5,GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
#else
	GenericWarpTest<MEMORYDEVICE_CPU>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                  5, GenericWarpTestMode::TEST_FINAL_ITERATION_AND_FUSION);
#endif
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataTermOnly_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 10, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonov_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 5, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_CUDA) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CUDA>(switches, 3, TEST_SUCCESSIVE_ITERATIONS, 1e-5);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_DataAndTikhonovAndSobolevSmoothing_Fusion_CUDA) {
	GenericWarpTest<MEMORYDEVICE_CUDA>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                   5, GenericWarpTestMode::TEST_FINAL_ITERATION_AND_FUSION, 1e-5);
}

#endif

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_only) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(0, switches);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CPU_data_and_tikhonov) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(1, switches);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_only) {
	SlavchevaSurfaceTracker::Switches switches(true, false, false, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(0, switches);
}

BOOST_AUTO_TEST_CASE(Test_Warp_PVA_VBH_simple_CUDA_data_and_tikhonov) {
	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, false);
	Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(0, switches);
}

#endif


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintVolumeStatistics(
		VoxelVolume<TVoxel, TIndex>* volume,
		std::string description) {
	VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>& calculator =
			VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::Instance();
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
													  Frame16And17Fixture::InitParams<VoxelBlockHash>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CPU,
													   Frame16And17Fixture::InitParams<VoxelBlockHash>()),
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CPU,
													   Frame16And17Fixture::InitParams<VoxelBlockHash>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();


	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&canonical_volume, &view,
												  "TestData/snoopy_depth_000016.png",
												  "TestData/snoopy_color_000016.png",
												  "TestData/snoopy_omask_000016.png",
												  "TestData/snoopy_calib.txt",
												  MEMORYDEVICE_CPU,
												  Frame16And17Fixture::InitParams<VoxelBlockHash>());

	Vector2i image_size = view->depth->dimensions;

	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CPU);

	VisualizationEngine <TSDFVoxel, VoxelBlockHash>* visualization_engine =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
							 configuration::get().general_voxel_volume_parameters.far_clipping_distance,
							 MEMORYDEVICE_CPU);

	visualization_engine->CreateICPMaps(canonical_volume, view, &tracking_state, &render_state);

	updateView(&view, "TestData/snoopy_depth_000017.png",
			   "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
			   "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	const int live_index_to_start_from = 0;
	reconstructionEngine->GenerateTsdfVolumeFromTwoSurfaces(live_volumes[live_index_to_start_from], view,
															&tracking_state);

	VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& tsdf_calculator =
			VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[live_index_to_start_from], "[initial source live]");

	std::cout << "Utilized (initial source) hash count: " << live_volumes[live_index_to_start_from]->index.GetUtilizedHashBlockCount() << std::endl;

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
	const int utilized_codes1_count = live_volumes[0]->index.GetUtilizedHashBlockCount();
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



	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);

	SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_OPTIMIZED> motion_tracker(switches);
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
		bench::StartTimer("1_CalculateWarpGradient");
		motion_tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		std::cout << "Warp allocated hash count: " << warp_calculator.ComputeAllocatedHashBlockCount(&warp_field) << std::endl;
		bench::StopTimer("1_CalculateWarpGradient");
		bench::StartTimer("2_SmoothWarpGradient");
		motion_tracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("2_SmoothWarpGradient");
		bench::StartTimer("3_UpdateWarps");
		motion_tracker.UpdateWarps(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("3_UpdateWarps");
		bench::StartTimer("4_WarpVolume");
		motion_tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		std::cout << "Warp allocated hash count (2): " << warp_calculator.ComputeAllocatedHashBlockCount(&warp_field) << std::endl;
		PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[target_warped_field_ix], "[target live before warp]");
		std::cout << "Utilized (target) hash count: " << live_volumes[target_warped_field_ix]->index.GetUtilizedHashBlockCount() << std::endl;
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
											   live_volumes[target_warped_field_ix]);
		PrintVolumeStatistics<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>(live_volumes[target_warped_field_ix], "[target live after warp]");
		std::cout << "Utilized (target) hash count: " << live_volumes[target_warped_field_ix]->index.GetUtilizedHashBlockCount() << std::endl;
		bench::StopTimer("4_WarpVolume");
	}

	bench::PrintAllCumulativeTimes();

	delete visualization_engine;
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
													  Frame16And17Fixture::InitParams<VoxelBlockHash>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CUDA,
													   Frame16And17Fixture::InitParams<VoxelBlockHash>()),
			new VoxelVolume<TSDFVoxel, VoxelBlockHash>(&configuration::get().general_voxel_volume_parameters,
													   configuration::get().swapping_mode ==
													   configuration::SWAPPINGMODE_ENABLED,
													   MEMORYDEVICE_CUDA,
													   Frame16And17Fixture::InitParams<VoxelBlockHash>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();


	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&canonical_volume, &view,
												  "TestData/snoopy_depth_000016.png",
												  "TestData/snoopy_color_000016.png",
												  "TestData/snoopy_omask_000016.png",
												  "TestData/snoopy_calib.txt",
												  MEMORYDEVICE_CUDA,
												  Frame16And17Fixture::InitParams<VoxelBlockHash>());

	Vector2i image_size = view->depth->dimensions;

	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CUDA);

	VisualizationEngine <TSDFVoxel, VoxelBlockHash>* visualization_engine =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
							 configuration::get().general_voxel_volume_parameters.far_clipping_distance,
							 MEMORYDEVICE_CUDA);

	visualization_engine->CreateICPMaps(canonical_volume, view, &tracking_state, &render_state);

	updateView(&view, "TestData/snoopy_depth_000017.png",
			   "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
			   "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA);

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

	SlavchevaSurfaceTracker::Switches switches(true, false, true, false, true);

	SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_OPTIMIZED> motion_tracker(switches);
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
		bench::StartTimer("1_CalculateWarpGradient");
		motion_tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("1_CalculateWarpGradient");
		bench::StartTimer("2_SmoothWarpGradient");
		motion_tracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("2_SmoothWarpGradient");
		bench::StartTimer("3_UpdateWarps");
		motion_tracker.UpdateWarps(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		bench::StopTimer("3_UpdateWarps");
		bench::StartTimer("4_WarpVolume");
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
											   live_volumes[target_warped_field_ix]);
		bench::StopTimer("4_WarpVolume");
	}

	bench::PrintAllCumulativeTimes();

	delete visualization_engine;
	delete view;
	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete warping_engine;
	delete volume_fusion_engine;
}

#endif