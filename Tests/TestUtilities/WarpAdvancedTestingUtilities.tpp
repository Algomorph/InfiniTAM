//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
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
#pragma once

//boost
#include <boost/test/test_tools.hpp>

//ITMLib
#include "../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../ITMLib/Engines/Rendering/VisualizationEngineFactory.h"
#include "../../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
//(CPU)
#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif


//test_utilities
#include "TestUtilities.h"
#include "SnoopyTestUtilities.h"

#include "WarpAdvancedTestingUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

namespace test_utilities {

template<typename TIndex>
std::string GetPathBase(std::string prefix, int iteration) {
	return "TestData/volumes/" + IndexString<TIndex>() + "/" + prefix + "_iter_" + std::to_string(iteration) + "_";
}

template<typename TIndex>
std::string GetWarpsPath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "warps.dat";
}

template<typename TIndex>
std::string GetWarpedLivePath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "warped_live.dat";
}

template<typename TIndex>
std::string GetFusedPath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "fused.dat";
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateRawLiveAndCanonicalVolumes(VoxelVolume<TSDFVoxel, TIndex>** canonical_volume,
                                        VoxelVolume<TSDFVoxel, TIndex>** live_volume) {
	View* view = nullptr;
	BuildSdfVolumeFromImage_NearSurfaceAllocation(canonical_volume,
	                                              &view,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              TMemoryDeviceType,
	                                              snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	Vector2i image_size = view->depth->dimensions;

	CameraTrackingState tracking_state(image_size, TMemoryDeviceType);

	VisualizationEngine<TSDFVoxel, TIndex>* visualization_engine =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, TIndex>(TMemoryDeviceType);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                         configuration::get().general_voxel_volume_parameters.far_clipping_distance,
	                         TMemoryDeviceType);

	visualization_engine->CreateICPMaps(*canonical_volume, view, &tracking_state, &render_state);

	UpdateView(&view,
	           snoopy::Frame17DepthPath(),
	           snoopy::Frame17ColorPath(),
	           snoopy::Frame17MaskPath(),
	           snoopy::SnoopyCalibrationPath(),
	           TMemoryDeviceType);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, TIndex>* depth_fusion_engine =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);

	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateNearAndBetweenTwoSurfaces(*live_volume, view, &tracking_state);
	depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(*live_volume, view);


	delete visualization_engine;
	delete view;
}


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericWarpConsistencySubtest(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit,
                                   GenericWarpTestMode mode, float absolute_tolerance) {


	std::string volume_filename_prefix = SwitchesToPrefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType,
	                                          snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
			                                   snoopy::InitializationParameters_Fr16andFr17<TIndex>()),
			new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
			                                   snoopy::InitializationParameters_Fr16andFr17<TIndex>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();

	const int live_index_to_start_from = 0;
	GenerateRawLiveAndCanonicalVolumes<TIndex, TMemoryDeviceType>(&canonical_volume,
	                                                              &live_volumes[live_index_to_start_from]);
	AllocateUsingOtherVolume(live_volumes[(live_index_to_start_from + 1) % 2], live_volumes[live_index_to_start_from],
	                         TMemoryDeviceType);
	AllocateUsingOtherVolume(&warp_field, live_volumes[live_index_to_start_from], TMemoryDeviceType);
	AllocateUsingOtherVolume(canonical_volume, live_volumes[live_index_to_start_from], TMemoryDeviceType);


	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motion_tracker(switches);

	VoxelVolume<WarpVoxel, TIndex> ground_truth_warp_field(TMemoryDeviceType,
	                                                       snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex> ground_truth_sdf_volume(TMemoryDeviceType,
	                                                       snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	ground_truth_warp_field.Reset();

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, TIndex>* reconstruction_engine =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);
	WarpingEngineInterface<TSDFVoxel, WarpVoxel, TIndex>* warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);
	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);

	//note: will be swapped before first iteration

	int source_warped_field_ix = (live_index_to_start_from + 1) % 2;
	int target_warped_field_ix = live_index_to_start_from;
	for (int iteration = 0; iteration < iteration_limit; iteration++) {
		std::swap(source_warped_field_ix, target_warped_field_ix);
		std::cout << "Subtest " << IndexString<TIndex>() << " iteration " << std::to_string(iteration) << std::endl;
		motion_tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		motion_tracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		motion_tracker.UpdateWarps(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
		                                       live_volumes[target_warped_field_ix]);
		std::string path = GetWarpsPath<TIndex>(volume_filename_prefix, iteration);
		std::string path_warped_live = GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				live_volumes[target_warped_field_ix]->SaveToDisk(
						std::string(GENERATED_TEST_DATA_PREFIX) + path_warped_live);
				warp_field.SaveToDisk(std::string(GENERATED_TEST_DATA_PREFIX) + path);
				break;
			case TEST_SUCCESSIVE_ITERATIONS:
				EditAndCopyEngineFactory::Instance<WarpVoxel, TIndex, TMemoryDeviceType>().ResetVolume(
						&ground_truth_warp_field);
				ground_truth_warp_field.LoadFromDisk(path);

				BOOST_REQUIRE(contentAlmostEqual_Verbose(&warp_field, &ground_truth_warp_field, absolute_tolerance,
				                                         TMemoryDeviceType));
				EditAndCopyEngineFactory::Instance<TSDFVoxel, TIndex, TMemoryDeviceType>().ResetVolume(
						&ground_truth_sdf_volume);
				ground_truth_sdf_volume.LoadFromDisk(path_warped_live);
				BOOST_REQUIRE(contentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
				                                         absolute_tolerance, TMemoryDeviceType));
				break;
			default:
				break;
		}
	}
	std::cout << IndexString<TIndex>() << " fusion test" << std::endl;
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field.SaveToDisk(std::string(GENERATED_TEST_DATA_PREFIX) +
			                      GetWarpsPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			live_volumes[target_warped_field_ix]->SaveToDisk(
					std::string(GENERATED_TEST_DATA_PREFIX) +
					GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix],
			                                                   0);
			canonical_volume->SaveToDisk(
					std::string(GENERATED_TEST_DATA_PREFIX) +
					GetFusedPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			EditAndCopyEngineFactory::Instance<WarpVoxel, TIndex, TMemoryDeviceType>().ResetVolume(
					&ground_truth_warp_field);
			ground_truth_warp_field.LoadFromDisk(GetWarpsPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentAlmostEqual(&warp_field, &ground_truth_warp_field, absolute_tolerance, TMemoryDeviceType));
			ground_truth_sdf_volume.LoadFromDisk(
					GetWarpedLivePath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
			                                         absolute_tolerance, TMemoryDeviceType));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix],
			                                                   0);
			ground_truth_sdf_volume.LoadFromDisk(GetFusedPath<TIndex>(volume_filename_prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual(canonical_volume, &ground_truth_sdf_volume, absolute_tolerance,
			                                 TMemoryDeviceType));
			break;
		default:
			break;
	}

	delete canonical_volume;
	delete live_volumes[0];
	delete live_volumes[1];
	delete reconstruction_engine;
	delete warping_engine;
	delete volume_fusion_engine;
}


template<MemoryDeviceType TMemoryDeviceType>
void Warp_PVA_VBH_simple_subtest(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches) {

	if (iteration < 0) {
		DIEWITHEXCEPTION_REPORTLOCATION("Expecting iteration >= 0, got less than that, aborting.");
	}
	std::string path_frame_16_PVA = snoopy::PartialVolume16Path<PlainVoxelArray>();
	std::string path_frame_16_VBH = snoopy::PartialVolume16Path<VoxelBlockHash>();
	std::string path_frame_17_PVA = snoopy::PartialVolume17Path<PlainVoxelArray>();
	std::string path_frame_17_VBH = snoopy::PartialVolume17Path<VoxelBlockHash>();

	std::string prefix = SwitchesToPrefix(trackerSwitches);
	float absolute_tolerance = 1e-7;

	// *** initialize/load warps
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps_PVA = GetWarpsPath<PlainVoxelArray>(prefix, iteration - 1);
		LoadVolume(&warps_PVA, path_warps_PVA, TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
		std::string path_warps_VBH = GetWarpsPath<VoxelBlockHash>(prefix, iteration - 1);
		LoadVolume(&warps_VBH, path_warps_VBH, TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));
	} else {
		initializeVolume(&warps_PVA, snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>(),
		                 TMemoryDeviceType);
		initializeVolume(&warps_VBH, snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>(), TMemoryDeviceType);

		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));
	}

	// *** load warped live scene
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_PVA;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_VBH;

	std::string path_live_VBH, path_live_PVA;
	if (iteration > 0) {
		path_live_PVA = GetWarpedLivePath<PlainVoxelArray>(prefix, iteration - 1);
		path_live_VBH = GetWarpedLivePath<VoxelBlockHash>(prefix, iteration - 1);
	} else {
		path_live_PVA = path_frame_17_PVA;
		path_live_VBH = path_frame_17_VBH;
	}
	LoadVolume(&warped_live_PVA, path_live_PVA, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	LoadVolume(&warped_live_VBH, path_live_VBH, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	if (iteration == 0) {
		AllocateUsingOtherVolume(warps_VBH, warped_live_VBH, TMemoryDeviceType);
	}

	// *** load canonical volume as the two different data structures
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_16_PVA;
	LoadVolume(&volume_16_PVA, path_frame_16_PVA, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_16_VBH;
	LoadVolume(&volume_16_VBH, path_frame_16_VBH, TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	AllocateUsingOtherVolume(volume_16_VBH, warped_live_VBH, TMemoryDeviceType);

	// *** perform the warp gradient computation and warp updates
	SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_PVA(trackerSwitches);

	std::cout << "==== CALCULATE PVA WARPS === " << std::endl;
	motionTracker_PVA.CalculateWarpGradient(warps_PVA, volume_16_PVA, warped_live_PVA);
	motionTracker_PVA.SmoothWarpGradient(warps_PVA, volume_16_PVA, warped_live_PVA);
	motionTracker_PVA.UpdateWarps(warps_PVA, volume_16_PVA, warped_live_PVA);

	SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motionTracker_VBH(trackerSwitches);


	std::cout << "==== CALCULATE VBH WARPS === " << std::endl;
	motionTracker_VBH.CalculateWarpGradient(warps_VBH, volume_16_VBH, warped_live_VBH);
	motionTracker_VBH.SmoothWarpGradient(warps_VBH, volume_16_VBH, warped_live_VBH);
	motionTracker_VBH.UpdateWarps(warps_VBH, volume_16_VBH, warped_live_VBH);

	BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(warps_PVA, warps_VBH, absolute_tolerance, TMemoryDeviceType));


	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	VoxelVolume<WarpVoxel, PlainVoxelArray>* loaded_warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* loaded_warps_VBH;
	LoadVolume(&loaded_warps_PVA, GetWarpsPath<PlainVoxelArray>(prefix, iteration), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	LoadVolume(&loaded_warps_VBH, GetWarpsPath<VoxelBlockHash>(prefix, iteration), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());


	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_PVA, loaded_warps_PVA, absolute_tolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_VBH, loaded_warps_VBH, absolute_tolerance, TMemoryDeviceType));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericWarpTest(const SlavchevaSurfaceTracker::Switches& switches, int iteration_limit,
                     GenericWarpTestMode mode, float absolute_tolerance) {

	std::string prefix = SwitchesToPrefix(switches);
	GenericWarpConsistencySubtest<PlainVoxelArray, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                  absolute_tolerance);
	GenericWarpConsistencySubtest<VoxelBlockHash, TMemoryDeviceType>(switches, iteration_limit, mode,
	                                                                 absolute_tolerance);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA(TMemoryDeviceType,
	                                                   snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH(TMemoryDeviceType,
	                                                  snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	switch (mode) {
		case TEST_SUCCESSIVE_ITERATIONS: {

			VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_PVA(TMemoryDeviceType,
			                                                       snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
			VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_VBH(TMemoryDeviceType,
			                                                      snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

			for (int iteration = 0; iteration < iteration_limit; iteration++) {
				std::cout << "Testing iteration " << iteration << std::endl;
				warp_field_PVA.LoadFromDisk(GetWarpsPath<PlainVoxelArray>(prefix, iteration));
				warp_field_VBH.Reset();
				warp_field_VBH.LoadFromDisk(GetWarpsPath<VoxelBlockHash>(prefix, iteration));
				BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(&warp_field_PVA, &warp_field_VBH,
				                                                  absolute_tolerance, TMemoryDeviceType));
				volume_PVA.LoadFromDisk(GetWarpedLivePath<PlainVoxelArray>(prefix, iteration));
				volume_VBH.Reset();
				volume_VBH.LoadFromDisk(GetWarpedLivePath<VoxelBlockHash>(prefix, iteration));
				BOOST_REQUIRE(contentForFlagsAlmostEqual_Verbose(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED,
				                                                 absolute_tolerance, TMemoryDeviceType));
			}
		}
			break;
		case TEST_FINAL_ITERATION_AND_FUSION: {
			volume_PVA.LoadFromDisk(GetWarpedLivePath<PlainVoxelArray>(prefix, iteration_limit - 1));
			volume_VBH.Reset();
			volume_VBH.LoadFromDisk(GetWarpedLivePath<VoxelBlockHash>(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absolute_tolerance,
					                           TMemoryDeviceType));
			volume_PVA.LoadFromDisk(GetFusedPath<PlainVoxelArray>(prefix, iteration_limit - 1));
			volume_VBH.Reset();
			volume_VBH.LoadFromDisk(GetFusedPath<VoxelBlockHash>(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentForFlagsAlmostEqual(&volume_PVA, &volume_VBH, VOXEL_NONTRUNCATED, absolute_tolerance,
					                           TMemoryDeviceType));
		}
			break;
		default:
			break;
	}
}

} // namespace test_utilities