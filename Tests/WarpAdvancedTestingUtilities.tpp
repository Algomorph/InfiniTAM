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

//local
#include "WarpAdvancedTestingUtilities.h"
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

//ITMLib
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
//CPU
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"
//CUDA
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

#endif


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateRawLiveAndCanonicalVolumes(VoxelVolume<TSDFVoxel, TIndex>** canonical_volume,
                                        VoxelVolume<TSDFVoxel, TIndex>** live_volume) {
	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(canonical_volume, &view,
	                                              "TestData/snoopy_depth_000016.png",
	                                              "TestData/snoopy_color_000016.png",
	                                              "TestData/snoopy_omask_000016.png",
	                                              "TestData/snoopy_calib.txt",
	                                              TMemoryDeviceType,
	                                              Frame16And17Fixture::InitParams<TIndex>());

	Vector2i image_size = view->depth->dimensions;

	CameraTrackingState tracking_state(image_size, TMemoryDeviceType);

	VisualizationEngine<TSDFVoxel, TIndex>* visualization_engine =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, TIndex>(TMemoryDeviceType);

	RenderState render_state(image_size, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                         configuration::get().general_voxel_volume_parameters.far_clipping_distance,
	                         TMemoryDeviceType);

	visualization_engine->CreateICPMaps(*canonical_volume, view, &tracking_state, &render_state);

	updateView(&view, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", TMemoryDeviceType);

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

	std::string prefix = switches_to_prefix(switches);
	if (iteration_limit < 2) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration limit must be at least 2");
	}

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>());
	warp_field.Reset();

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[2] = {
			new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>()),
			new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType, Frame16And17Fixture::InitParams<TIndex>())
	};
	live_volumes[0]->Reset();
	live_volumes[1]->Reset();

	const int live_index_to_start_from = 0;
	GenerateRawLiveAndCanonicalVolumes<TIndex, TMemoryDeviceType>(&canonical_volume,
	                                                              &live_volumes[live_index_to_start_from]);
	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateFromOtherVolume(live_volumes[(live_index_to_start_from + 1) % 2],
			                         live_volumes[live_index_to_start_from]);
	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateWarpVolumeFromOtherVolume(&warp_field, live_volumes[live_index_to_start_from]);

	IndexingEngine<TSDFVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateFromOtherVolume(canonical_volume, live_volumes[live_index_to_start_from]);


	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC>
			motion_tracker(switches);

	VoxelVolume<WarpVoxel, TIndex> ground_truth_warp_field(TMemoryDeviceType,
	                                                       Frame16And17Fixture::InitParams<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex> ground_truth_sdf_volume(TMemoryDeviceType,
	                                                       Frame16And17Fixture::InitParams<TIndex>());

	ground_truth_warp_field.Reset();

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, TIndex>* reconstruction_engine =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);
	WarpingEngineInterface<TSDFVoxel, WarpVoxel, TIndex>* warping_engine =
			WarpingEngineFactory::MakeWarpingEngine<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);
	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);

	//note: will be swapped before first iteration

	int source_warped_field_ix = (live_index_to_start_from + 1) % 2;
	int target_warped_field_ix = live_index_to_start_from;
	for (int iteration = 0; iteration < iteration_limit; iteration++) {
		std::swap(source_warped_field_ix, target_warped_field_ix);
		std::cout << "Subtest " << getIndexString<TIndex>() << " iteration " << std::to_string(iteration) << std::endl;
		motion_tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		motion_tracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		motion_tracker.UpdateWarps(&warp_field, canonical_volume, live_volumes[source_warped_field_ix]);
		warping_engine->WarpVolume_WarpUpdates(&warp_field, live_volumes[source_warped_field_ix],
		                                       live_volumes[target_warped_field_ix]);
		std::string path = get_path_warps(prefix, iteration);
		std::string path_warped_live = get_path_warped_live(prefix, iteration);
		switch (mode) {
			case SAVE_SUCCESSIVE_ITERATIONS:
				live_volumes[target_warped_field_ix]->SaveToDisk(std::string("../../Tests/") + path_warped_live);
				warp_field.SaveToDisk(std::string("../../Tests/") + path);
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
	std::cout << getIndexString<TIndex>() << " fusion test" << std::endl;
	switch (mode) {
		case SAVE_FINAL_ITERATION_AND_FUSION:
			warp_field.SaveToDisk(std::string("../../Tests/") + get_path_warps(prefix, iteration_limit - 1));
			live_volumes[target_warped_field_ix]->SaveToDisk(
					std::string("../../Tests/") + get_path_warped_live(prefix, iteration_limit - 1));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix],
			                                                   0);
			canonical_volume->SaveToDisk(
					std::string("../../Tests/") + get_path_fused(prefix, iteration_limit - 1));
			break;
		case TEST_FINAL_ITERATION_AND_FUSION:
			EditAndCopyEngineFactory::Instance<WarpVoxel, TIndex, TMemoryDeviceType>().ResetVolume(
					&ground_truth_warp_field);
			ground_truth_warp_field.LoadFromDisk(get_path_warps(prefix, iteration_limit - 1));
			BOOST_REQUIRE(
					contentAlmostEqual(&warp_field, &ground_truth_warp_field, absolute_tolerance, TMemoryDeviceType));
			ground_truth_sdf_volume.LoadFromDisk(get_path_warped_live(prefix, iteration_limit - 1));
			BOOST_REQUIRE(contentAlmostEqual_Verbose(live_volumes[target_warped_field_ix], &ground_truth_sdf_volume,
			                                         absolute_tolerance, TMemoryDeviceType));
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[target_warped_field_ix],
			                                                   0);
			ground_truth_sdf_volume.LoadFromDisk(get_path_fused(prefix, iteration_limit - 1));
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
	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_16_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	std::string path_frame_16_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";

	std::string prefix = switches_to_prefix(trackerSwitches);
	float absoluteTolerance = 1e-7;

	// *** initialize/load warps
	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	if (iteration > 0) {
		std::string path_warps = get_path_warps(prefix, iteration - 1);
		loadVolume(&warps_PVA, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<PlainVoxelArray>());
		loadVolume(&warps_VBH, path_warps, TMemoryDeviceType,
		           Frame16And17Fixture::InitParams<VoxelBlockHash>());
		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	} else {
		initializeVolume(&warps_PVA, Frame16And17Fixture::InitParams<PlainVoxelArray>(), TMemoryDeviceType);
		initializeVolume(&warps_VBH, Frame16And17Fixture::InitParams<VoxelBlockHash>(), TMemoryDeviceType);

		BOOST_REQUIRE(allocatedContentAlmostEqual(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));
	}

	// *** load warped live scene
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* warped_live_PVA;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* warped_live_VBH;

	std::string path_live_VBH, path_live_PVA;
	if (iteration > 0) {
		std::string path_warped_live = get_path_warped_live(prefix, iteration - 1);
		path_live_VBH = path_live_PVA = path_warped_live;
	} else {
		path_live_PVA = path_frame_17_PVA;
		path_live_VBH = path_frame_17_VBH;
	}
	loadVolume(&warped_live_PVA, path_live_PVA, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<PlainVoxelArray>());
	loadVolume(&warped_live_VBH, path_live_VBH, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<VoxelBlockHash>());
	if (iteration == 0) {
		IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>::Instance()
				.AllocateWarpVolumeFromOtherVolume(warps_VBH, warped_live_VBH);
	}

	// *** load canonical volume as the two different data structures
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_16_PVA;
	loadVolume(&volume_16_PVA, path_frame_16_PVA, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_16_VBH;
	loadVolume(&volume_16_VBH, path_frame_16_VBH, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<VoxelBlockHash>());
	IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>::Instance()
			.AllocateFromOtherVolume(volume_16_VBH, warped_live_VBH);



	//_DEBUG
//	Vector3i test_pos(-56, -9, 200);
//	Vector3s voxel_block_pos = TO_SHORT_FLOOR3(test_pos.toFloat() / VOXEL_BLOCK_SIZE);
//	configuration::get().verbosity_level = configuration::VERBOSITY_FOCUS_SPOTS;
//	configuration::get().telemetry_settings.focus_coordinates = test_pos;
//
//	int hashCode;
//	HashEntry entry = volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
//	printf("Entry %d %d %d: %d\n", voxel_block_pos.x, voxel_block_pos.y, voxel_block_pos.z, hashCode);
//
//	test_pos = Vector3i(-56, -8, 200);
//	TSDFVoxel voxelPVA_canonical = volume_16_PVA->GetValueAt(test_pos);
//	std::cout << "PVA canonical voxel of interest: ";
//	voxelPVA_canonical.print_self();
//	TSDFVoxel voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
//	std::cout << "VBH canonical voxel of interest: ";
//	voxelVBH_canonical.print_self();
//
//	TSDFVoxel voxelPVA = warped_live_PVA->GetValueAt(test_pos);
//	std::cout << "PVA live voxel of interest: ";
//	voxelPVA.print_self();
//	TSDFVoxel voxelVBH = warped_live_VBH->GetValueAt(test_pos);
//	std::cout << "VBH live voxel of interest: ";
//	voxelVBH.print_self();


//	voxelVBH_canonical = volume_16_VBH->GetValueAt(test_pos);
//	std::cout << "VBH canonical voxel of interest (after allocation): ";
//	voxelVBH_canonical.print_self();
//	entry = volume_16_VBH->index.GetHashEntryAt(voxel_block_pos, hashCode);
//	std::cout << "VBH canonical hash block: " << voxel_block_pos << " code " << hashCode << " ptr: " << entry.ptr << std::endl;

//	alternative_entry = volume_16_VBH->index.GetHashEntry(alternative_index);
//	std::cout << "VBH canonical " << alternative_index << " hash block ptr: " << alternative_entry.ptr << std::endl;
//	WarpVoxel warpPVA = warps_PVA->GetValueAt(test_pos);
//	std::cout << "PVA Warping value of interest: ";
//	warpPVA.print_self();
//	WarpVoxel warpVBH = warps_VBH->GetValueAt(test_pos);
//	std::cout << "VBH Warping value of interest: ";
//	warpVBH.print_self();

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

	// *** test content

//	WarpVoxel warpPVA = warps_PVA->GetValueAt(test_pos);
//	std::cout << "PVA Warping value of interest: ";
//	warpPVA.print_self();
//	WarpVoxel warpVBH = warps_VBH->GetValueAt(test_pos);
//	std::cout << "VBH Warping value of interest: ";
//	warpVBH.print_self();

	BOOST_REQUIRE(allocatedContentAlmostEqual_Verbose(warps_PVA, warps_VBH, absoluteTolerance, TMemoryDeviceType));


	delete volume_16_PVA;
	delete volume_16_VBH;
	delete warped_live_PVA;
	delete warped_live_VBH;

	VoxelVolume<WarpVoxel, PlainVoxelArray>* loaded_warps_PVA;
	VoxelVolume<WarpVoxel, VoxelBlockHash>* loaded_warps_VBH;
	std::string path_loaded_warps = get_path_warps(prefix, iteration);
	loadVolume(&loaded_warps_PVA, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<PlainVoxelArray>());
	loadVolume(&loaded_warps_VBH, path_loaded_warps, TMemoryDeviceType,
	           Frame16And17Fixture::InitParams<VoxelBlockHash>());


	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_PVA, loaded_warps_PVA, absoluteTolerance, TMemoryDeviceType));
	BOOST_REQUIRE(contentAlmostEqual_Verbose(warps_VBH, loaded_warps_VBH, absoluteTolerance, TMemoryDeviceType));

	delete warps_PVA;
	delete warps_VBH;
	delete loaded_warps_PVA;
	delete loaded_warps_VBH;
}