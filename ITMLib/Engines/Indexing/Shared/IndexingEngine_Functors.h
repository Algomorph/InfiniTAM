//  ================================================================
//  Created by Gregory Kramida on 2/5/20.
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

//local
#include "IndexingEngine_DiagnosticData.h"
#include "IndexingEngine_Shared.h"
#include "IndexingEngine_RayMarching.h"
#include "IndexingEngine_DiagnosticRayMarching.h"
#include "../../Telemetry/TelemetryRecorderFactory.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"
#include "../../../Utils/VoxelVolumeParameters.h"
#include "../../../Utils/Telemetry/TelemetryUtilities.h"

namespace ITMLib {


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, HashBlockVisibility THashBlockVisibility>
struct BlockVisibilitySetFunctor {
private: // instance variables
	HashBlockVisibility* block_visibility_types;
public: // instance functions
	explicit BlockVisibilitySetFunctor(VoxelVolume<TVoxel, VoxelBlockHash>* volume) :
			block_visibility_types(volume->index.GetBlockVisibilityTypes()) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const int visible_block_hash_code, const int i_visible_block) {
		block_visibility_types[visible_block_hash_code] = THashBlockVisibility;
	}
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct BlockListDeallocationFunctor {
public:
	Vector3s* colliding_positions_device = nullptr;

private: // instance variables
	VoxelBlockHash& index;
	TVoxel* voxels;
	HashEntryAllocationState* hash_entry_states;
	HashEntry* hash_table;
	int* block_allocation_list;
	int* excess_entry_list;
	const TVoxel* empty_voxel_block_device;

	DECLARE_ATOMIC(int, colliding_block_count);
	DECLARE_ATOMIC(int, last_free_voxel_block_id);
	DECLARE_ATOMIC(int, last_free_excess_list_id);

public: // instance functions
	explicit BlockListDeallocationFunctor(VoxelVolume<TVoxel, VoxelBlockHash>* volume)
			: index(volume->index),
			  voxels(volume->GetVoxels()),
			  hash_entry_states(volume->index.GetHashEntryAllocationStates()),
			  hash_table(volume->index.GetEntries()),
			  block_allocation_list(volume->index.GetBlockAllocationList()),
			  excess_entry_list(volume->index.GetExcessEntryList()) {
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, volume->index.GetLastFreeBlockListId());
		INITIALIZE_ATOMIC(int, last_free_excess_list_id, volume->index.GetLastFreeExcessListId());

		static ORUtils::MemoryBlock<TVoxel> empty_voxel_block = []() {
			ORUtils::MemoryBlock<TVoxel> empty_voxel_block(VOXEL_BLOCK_SIZE3, true, true);
			TVoxel* empty_voxel_block_CPU = empty_voxel_block.GetData(MEMORYDEVICE_CPU);
			for (int i_voxel = 0; i_voxel < VOXEL_BLOCK_SIZE3; i_voxel++) empty_voxel_block_CPU[i_voxel] = TVoxel();
			empty_voxel_block.UpdateDeviceFromHost();
			return empty_voxel_block;
		}();
		empty_voxel_block_device = empty_voxel_block.GetData(TMemoryDeviceType);
	}

	~BlockListDeallocationFunctor() {
		CLEAN_UP_ATOMIC(colliding_block_count);CLEAN_UP_ATOMIC(last_free_voxel_block_id);CLEAN_UP_ATOMIC(
				last_free_excess_list_id);
	}

	void SetCollidingBlockCount(int value) {
		SET_ATOMIC_VALUE_CPU(colliding_block_count, value);
	}

	int GetCollidingBlockCount() const {
		return GET_ATOMIC_VALUE_CPU(colliding_block_count);
	}

	void SetIndexFreeVoxelBlockIdAndExcessListId() {
		this->index.SetLastFreeBlockListId(GET_ATOMIC_VALUE_CPU(last_free_voxel_block_id));
		this->index.SetLastFreeExcessListId(GET_ATOMIC_VALUE_CPU(last_free_excess_list_id));
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const Vector3s& block_position_to_clear) {
		DeallocateBlock(block_position_to_clear, hash_entry_states, hash_table, voxels, colliding_positions_device,
		                colliding_block_count, last_free_voxel_block_id, last_free_excess_list_id,
		                block_allocation_list, excess_entry_list, empty_voxel_block_device);
	}
};


template<MemoryDeviceType TMemoryDeviceType>
struct HashEntryStateBasedAllocationFunctor_Base {
protected: // instance variables
	HashEntryAllocationState* hash_entry_states;
	Vector3s* block_coordinates;

	HashEntry* hash_table;
	const int* block_allocation_list;
	const int* excess_entry_list;
	int* utilized_block_hash_codes;

	DECLARE_ATOMIC(int, last_free_voxel_block_id);
	DECLARE_ATOMIC(int, last_free_excess_list_id);
	DECLARE_ATOMIC(int, utilized_block_count);
public: // instance functions
	HashEntryStateBasedAllocationFunctor_Base(VoxelBlockHash& index)
			: hash_entry_states(index.GetHashEntryAllocationStates()),
			  block_coordinates(index.GetAllocationBlockCoordinates()),
			  hash_table(index.GetEntries()),

			  block_allocation_list(index.GetBlockAllocationList()),
			  excess_entry_list(index.GetExcessEntryList()),
			  utilized_block_hash_codes(index.GetUtilizedBlockHashCodes()) {
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, index.GetLastFreeBlockListId());
		INITIALIZE_ATOMIC(int, last_free_excess_list_id, index.GetLastFreeExcessListId());
		INITIALIZE_ATOMIC(int, utilized_block_count, index.GetUtilizedBlockCount());
	}

	virtual ~HashEntryStateBasedAllocationFunctor_Base() {
		CLEAN_UP_ATOMIC(last_free_voxel_block_id);CLEAN_UP_ATOMIC(last_free_excess_list_id);CLEAN_UP_ATOMIC(
				utilized_block_count);
	}

	void UpdateIndexCounters(VoxelBlockHash& index) {
		index.SetLastFreeBlockListId(GET_ATOMIC_VALUE_CPU(last_free_voxel_block_id));
		index.SetLastFreeExcessListId(GET_ATOMIC_VALUE_CPU(last_free_excess_list_id));
		index.SetUtilizedBlockCount(GET_ATOMIC_VALUE_CPU(utilized_block_count));
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct HashEntryStateBasedAllocationFunctor
		: public HashEntryStateBasedAllocationFunctor_Base<TMemoryDeviceType> {
public: // instance functions
	HashEntryStateBasedAllocationFunctor(VoxelBlockHash& index)
			: HashEntryStateBasedAllocationFunctor_Base<TMemoryDeviceType>(index) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntryAllocationState& hash_entry_state, const int hash_code) {
		AllocateBlockBasedOnState<TMemoryDeviceType>(
				hash_entry_state, hash_code, this->block_coordinates, this->hash_table,
				this->last_free_voxel_block_id, this->last_free_excess_list_id, this->utilized_block_count,
				this->block_allocation_list, this->excess_entry_list, this->utilized_block_hash_codes);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct HashEntryStateBasedAllocationFunctor_SetVisibility
		: public HashEntryStateBasedAllocationFunctor_Base<TMemoryDeviceType> {
protected: // instance variables
	HashBlockVisibility* block_visibility_types;
public: // instance functions
	HashEntryStateBasedAllocationFunctor_SetVisibility(VoxelBlockHash& index)
			: HashEntryStateBasedAllocationFunctor_Base<TMemoryDeviceType>(index),
			  block_visibility_types(index.GetBlockVisibilityTypes()) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntryAllocationState& hash_entry_state, int hash_code) {
		AllocateBlockBasedOnState_SetVisibility<TMemoryDeviceType>(
				hash_entry_state, hash_code, this->block_coordinates, this->hash_table, this->block_visibility_types,
				this->last_free_voxel_block_id, this->last_free_excess_list_id, this->utilized_block_count,
				this->block_allocation_list, this->excess_entry_list, this->utilized_block_hash_codes);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct BlockListAllocationStateMarkerFunctor {
public:
	Vector3s* colliding_positions_device = nullptr;

private: // instance variables
	HashEntryAllocationState* hash_entry_states;
	Vector3s* allocation_block_coordinates;
	HashEntry* hash_table;
	DECLARE_ATOMIC(int, colliding_block_count);
public: // instance functions
	explicit BlockListAllocationStateMarkerFunctor(VoxelBlockHash& index)
			: hash_entry_states(index.GetHashEntryAllocationStates()),
			  allocation_block_coordinates(index.GetAllocationBlockCoordinates()),
			  hash_table(index.GetEntries()) {
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
	}

	~BlockListAllocationStateMarkerFunctor() {
		CLEAN_UP_ATOMIC(colliding_block_count);
	}

	void SetCollidingBlockCount(int value) {
		SET_ATOMIC_VALUE_CPU(colliding_block_count, value);
	}

	int GetCollidingBlockCount() const {
		return GET_ATOMIC_VALUE_CPU(colliding_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const Vector3s& desired_block_position) {
		MarkAsNeedingAllocationIfNotFound<false>(hash_entry_states, allocation_block_coordinates,
		                                         desired_block_position, hash_table, colliding_positions_device,
		                                         colliding_block_count);
	}
};

inline
_DEVICE_WHEN_AVAILABLE_
void PrintPixelBlocks(const int x, const int y, int pixel_block_count, const internal::PixelBlockAllocationRecord& pixel_blocks) {

	printf("Pixel blocks for pixel %d, %d: %s", x, y, red);
	int i_block;
	for (i_block = 0; i_block < pixel_block_count - 1; i_block++) {
		const Vector3s& block = pixel_blocks.values[i_block];
		printf("[%d, %d, %d], ", block.x, block.y, block.z);
	}
	const Vector3s& block = pixel_blocks.values[i_block];
	printf("[%d, %d, %d]%s", block.x, block.y, block.z, reset);
}

template<MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
struct DepthBasedAllocationStateMarkerFunctor {
public: // instance variables
	ORUtils::MemoryBlock<Vector3s> colliding_block_positions;
	VerbosityLevel verbosity;
	Vector2i focus_pixel;
protected: // instance variables

	const float near_clipping_distance;
	const float far_clipping_distance;
	const Matrix4f inverted_camera_pose;
	const Vector4f inverted_projection_parameters;

	const float surface_distance_cutoff;

	const float hash_block_size_reciprocal;
	HashEntryAllocationState* hash_entry_allocation_states;
	Vector3s* hash_block_coordinates;
	HashEntry* hash_table;

	Vector3s* colliding_block_positions_device;
	DECLARE_ATOMIC(int, colliding_block_count);

	ORUtils::MemoryBlock<bool> unresolvable_collision_encountered;
	bool* unresolvable_collision_encountered_device;
public: // instance functions
	DepthBasedAllocationStateMarkerFunctor(VoxelBlockHash& index,
	                                       const VoxelVolumeParameters& volume_parameters, const ITMLib::View* view,
	                                       Matrix4f inverted_depth_camera_pose, float surface_distance_cutoff) :

			near_clipping_distance(volume_parameters.near_clipping_distance),
			far_clipping_distance(volume_parameters.far_clipping_distance),
			inverted_camera_pose(inverted_depth_camera_pose),
			inverted_projection_parameters([&view]() {
				Vector4f params = view->calibration_information.intrinsics_d.projectionParamsSimple.all;
				params.fx = 1.0f / params.fx;
				params.fy = 1.0f / params.fy;
				return params;
			}()),
			surface_distance_cutoff(surface_distance_cutoff),
			hash_block_size_reciprocal(1.0f / (volume_parameters.voxel_size * VOXEL_BLOCK_SIZE)),
			hash_entry_allocation_states(index.GetHashEntryAllocationStates()),
			hash_block_coordinates(index.GetAllocationBlockCoordinates()),
			hash_table(index.GetEntries()),

			colliding_block_positions(index.hash_entry_count, TMemoryDeviceType),

			unresolvable_collision_encountered(1, true, TMemoryDeviceType == MemoryDeviceType::MEMORYDEVICE_CUDA),
			unresolvable_collision_encountered_device(unresolvable_collision_encountered.GetData(TMemoryDeviceType)) {
		*unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU) = false;
		unresolvable_collision_encountered.UpdateDeviceFromHost();
		colliding_block_positions_device = colliding_block_positions.GetData(TMemoryDeviceType);
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
	}

	virtual ~DepthBasedAllocationStateMarkerFunctor() {
		CLEAN_UP_ATOMIC(colliding_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const float& depth_measure, int x, int y) {
		if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
		    (depth_measure - surface_distance_cutoff) < near_clipping_distance ||
		    (depth_measure + surface_distance_cutoff) > far_clipping_distance)
			return;


		// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
		// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
		// point, in camera space
		ITMLib::Segment march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(surface_distance_cutoff,
		                                                                                       depth_measure,
		                                                                                       x, y,
		                                                                                       inverted_camera_pose,
		                                                                                       inverted_projection_parameters,
		                                                                                       hash_block_size_reciprocal);


		if (TExecutionMode == OPTIMIZED) {
			MarkVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
			                                *unresolvable_collision_encountered_device,
			                                hash_table, march_segment, colliding_block_positions_device,
			                                colliding_block_count);
		} else {
			// assume DIAGNOSTIC
			int pixel_block_count = 0;
			internal::PixelBlockAllocationRecord pixel_blocks;
			MarkVoxelHashBlocksAlongSegment_RecordPixelBlocks(
					this->hash_entry_allocation_states, this->hash_block_coordinates,
					*this->unresolvable_collision_encountered_device, this->hash_table,
					march_segment,
					this->colliding_block_positions_device, this->colliding_block_count,

					pixel_blocks,
					pixel_block_count);
			if (this->verbosity >= VerbosityLevel::VERBOSITY_FOCUS_SPOTS && x == this->focus_pixel.x && y == this->focus_pixel.y) {
				printf("March segment for pixel %d, %d: %s[%f, %f, %f],  [%f, %f, %f]%s\n", x, y, yellow,
				       march_segment.origin.x, march_segment.origin.y, march_segment.origin.z,
				       march_segment.destination().x, march_segment.destination().y, march_segment.destination().z, reset);
				PrintPixelBlocks(x, y, pixel_block_count, pixel_blocks);
			}
		}

	}

	void ResetFlagsAndCounters() {
		*unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU) = false;
		unresolvable_collision_encountered.UpdateDeviceFromHost();
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
	}

	bool EncounteredUnresolvableCollision() {
		unresolvable_collision_encountered.UpdateHostFromDevice();
		return *unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU);
	}

	int GetCollidingBlockCount() const {
		return GET_ATOMIC_VALUE_CPU(colliding_block_count);
	}
};

template<MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
struct TwoSurfaceBasedAllocationStateMarkerFunctor_Base
		: public DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType, TExecutionMode> {
protected: // instance variables
	const Matrix4f depth_camera_pose;
public: // instance functions
	TwoSurfaceBasedAllocationStateMarkerFunctor_Base(VoxelBlockHash& index,
	                                                 const VoxelVolumeParameters& volume_parameters,
	                                                 const ITMLib::View* view,
	                                                 const CameraTrackingState* tracking_state,
	                                                 float surface_distance_cutoff) :
			DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType, TExecutionMode>(index, volume_parameters, view,
			                                                                          tracking_state->pose_d->GetInvM(),
			                                                                          surface_distance_cutoff),
			depth_camera_pose(tracking_state->pose_d->GetM()) {}

protected: // instance functions

	_DEVICE_WHEN_AVAILABLE_
	inline bool
	ComputeMarchSegment(ITMLib::Segment& march_segment,
	                    Vector4f& live_surface_point_in_camera_space,
	                    Vector4f& canonical_surface_point_in_camera_space,
	                    bool& has_live_surface, bool& has_canonical_surface,
	                    const float& live_frame_depth_meters,
	                    Vector4f canonical_surface_point_in_world_space,
	                    const int x, const int y) {
		has_live_surface = has_canonical_surface = false;
		if (!(live_frame_depth_meters <= 0 || (live_frame_depth_meters - this->surface_distance_cutoff) < 0 ||
		      (live_frame_depth_meters - this->surface_distance_cutoff) < this->near_clipping_distance ||
		      (live_frame_depth_meters + this->surface_distance_cutoff) > this->far_clipping_distance))
			has_live_surface = true;

		if (canonical_surface_point_in_world_space.w > 0.0f) {
			has_canonical_surface = true;
			canonical_surface_point_in_world_space[3] = 1.0;
			canonical_surface_point_in_camera_space = WorldSpacePointToCameraSpace(canonical_surface_point_in_world_space, this->depth_camera_pose);
		}

		if (has_live_surface && has_canonical_surface) {
			live_surface_point_in_camera_space = ImageSpacePointToCameraSpace(live_frame_depth_meters, x, y,
			                                                                  this->inverted_projection_parameters);
			march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromAndBetweenTwoPoints(
					this->surface_distance_cutoff, live_surface_point_in_camera_space, canonical_surface_point_in_camera_space,
					this->inverted_camera_pose,
					this->hash_block_size_reciprocal);
		} else {
			if (has_live_surface) {
				march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(
						live_surface_point_in_camera_space, this->surface_distance_cutoff, live_frame_depth_meters, x, y,
						this->inverted_camera_pose, this->inverted_projection_parameters,
						this->hash_block_size_reciprocal);
			} else if (has_canonical_surface) {
				march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromPoint(
						this->surface_distance_cutoff, canonical_surface_point_in_camera_space, this->inverted_camera_pose,
						this->hash_block_size_reciprocal);
			} else {
				return false; // neither surface is defined at this point, nothing to do.
			}
		}
		return true;
	}

	_DEVICE_WHEN_AVAILABLE_
	inline bool
	ComputeMarchSegment(ITMLib::Segment& march_segment, const float& surface1_depth,
	                    const Vector4f& surface2_point_world_space,
	                    const int x, const int y) {
		Vector4f surface1_point_camera_space, surface2_point_camera_space;
		bool has_surface1, has_surface2;
		return ComputeMarchSegment(march_segment, surface1_point_camera_space, surface2_point_camera_space,
		                           has_surface1, has_surface2, surface1_depth, surface2_point_world_space, x, y);
	}
};

template<MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode = OPTIMIZED>
struct TwoSurfaceBasedAllocationStateMarkerFunctor;

template<MemoryDeviceType TMemoryDeviceType>
struct TwoSurfaceBasedAllocationStateMarkerFunctor<TMemoryDeviceType, OPTIMIZED>
		: public TwoSurfaceBasedAllocationStateMarkerFunctor_Base<TMemoryDeviceType, OPTIMIZED> {
public: // instance functions
	TwoSurfaceBasedAllocationStateMarkerFunctor(VoxelBlockHash& index,
	                                            const VoxelVolumeParameters& volume_parameters,
	                                            const ITMLib::View* view,
	                                            const CameraTrackingState* tracking_state,
	                                            float surface_distance_cutoff,
	                                            internal::IndexingEngine_VoxelBlockHash_ExecutionModeSpecialized<TMemoryDeviceType, OPTIMIZED>& specialized_engine)
			:
			TwoSurfaceBasedAllocationStateMarkerFunctor_Base<TMemoryDeviceType, OPTIMIZED>(index, volume_parameters, view,
			                                                                               tracking_state,
			                                                                               surface_distance_cutoff) {}

	using TwoSurfaceBasedAllocationStateMarkerFunctor_Base<TMemoryDeviceType, OPTIMIZED>::TwoSurfaceBasedAllocationStateMarkerFunctor_Base;

	void SaveDataToDisk() {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const float& surface1_depth, const Vector4f& surface2_point_world_space, const int x, const int y) {

		ITMLib::Segment march_segment;
		if (!this->ComputeMarchSegment(march_segment, surface1_depth, surface2_point_world_space, x, y)) {
			return;
		}

		MarkVoxelHashBlocksAlongSegment(this->hash_entry_allocation_states, this->hash_block_coordinates,
		                                *this->unresolvable_collision_encountered_device, this->hash_table,
		                                march_segment,
		                                this->colliding_block_positions_device, this->colliding_block_count);
	}

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType, OPTIMIZED>::colliding_block_positions;
};

template<MemoryDeviceType TMemoryDeviceType>
struct TwoSurfaceBasedAllocationStateMarkerFunctor<TMemoryDeviceType, DIAGNOSTIC>
		: public TwoSurfaceBasedAllocationStateMarkerFunctor_Base<TMemoryDeviceType, DIAGNOSTIC> {
private: // instance variables
	IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType>& diagnostic_data;
	typename IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType>::DataDevice* device_diagnostic_data;

public: // instance functions
	TwoSurfaceBasedAllocationStateMarkerFunctor(VoxelBlockHash& index,
	                                            const VoxelVolumeParameters& volume_parameters,
	                                            const ITMLib::View* view,
	                                            const CameraTrackingState* tracking_state,
	                                            float surface_distance_cutoff,
	                                            internal::IndexingEngine_VoxelBlockHash_ExecutionModeSpecialized<TMemoryDeviceType, DIAGNOSTIC>& specialized_engine)
			:
			TwoSurfaceBasedAllocationStateMarkerFunctor_Base<TMemoryDeviceType, DIAGNOSTIC>(index, volume_parameters, view,
			                                                                                tracking_state,
			                                                                                surface_distance_cutoff),
			diagnostic_data(specialized_engine.diagnostic_data),
			device_diagnostic_data(diagnostic_data.data_device.GetData(TMemoryDeviceType)) {
		if (diagnostic_data.PrepareForFrame(view->depth.dimensions)) {
			device_diagnostic_data = diagnostic_data.data_device.GetData(TMemoryDeviceType);
		}
	}

	void SaveDataToDisk() {
		std::string output_folder = telemetry::CreateAndGetOutputPathForFrame();
		diagnostic_data.SaveToDisk(output_folder);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const float& surface1_depth, const Vector4f& surface2_point_world_space, const int x, const int y) {
		ITMLib::Segment march_segment(Vector3f(0.0), Vector3f(0.0));
		Vector4f surface1_point_camera_space(0.0f), surface2_point_camera_space(0.0f);
		bool has_surface1, has_surface2;

		int pixel_block_count = 0;
		internal::PixelBlockAllocationRecord pixel_blocks;
		device_diagnostic_data->GetBlockRecordForPixel(x, y, pixel_block_count, pixel_blocks);

		if (!this->ComputeMarchSegment(march_segment, surface1_point_camera_space, surface2_point_camera_space,
		                               has_surface1, has_surface2, surface1_depth, surface2_point_world_space, x, y)) {
			device_diagnostic_data->SetPixelData(x, y, has_surface1, has_surface2, surface1_point_camera_space,
			                                     surface2_point_world_space, march_segment, pixel_blocks, pixel_block_count);
			if (this->verbosity >= VerbosityLevel::VERBOSITY_FOCUS_SPOTS && x == this->focus_pixel.x && y == this->focus_pixel.y) {
				printf("Pixel blocks for pixel %d, %d: %sNone%s", x, y, red, red);
			}
			return;
		}

		MarkVoxelHashBlocksAlongSegment_RecordPixelBlocks(
				this->hash_entry_allocation_states, this->hash_block_coordinates,
				*this->unresolvable_collision_encountered_device, this->hash_table,
				march_segment,
				this->colliding_block_positions_device, this->colliding_block_count,

				pixel_blocks,
				pixel_block_count);


		device_diagnostic_data->SetPixelData(x, y, has_surface1, has_surface2, surface1_point_camera_space,
		                                     surface2_point_camera_space, march_segment, pixel_blocks, pixel_block_count);

		if (this->verbosity >= VerbosityLevel::VERBOSITY_FOCUS_SPOTS && x == this->focus_pixel.x && y == this->focus_pixel.y) {
			PrintPixelBlocks(x, y, pixel_block_count, pixel_blocks);
		}

	}

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType, DIAGNOSTIC>::colliding_block_positions;
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ReallocateDeletedHashBlocksFunctor {
	ReallocateDeletedHashBlocksFunctor(VoxelVolume<TVoxel, VoxelBlockHash>* volume) :
			hash_block_visibility_types(volume->index.GetBlockVisibilityTypes()),
			block_allocation_list(volume->index.GetBlockAllocationList()) {
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, volume->index.GetLastFreeBlockListId());
	}

	~ReallocateDeletedHashBlocksFunctor() {
		CLEAN_UP_ATOMIC(last_free_voxel_block_id);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code) {
		if (hash_block_visibility_types[hash_code] > 0 && entry.ptr == -1) {
			int current_last_id = ATOMIC_ADD(last_free_voxel_block_id, -1);
			if (current_last_id >= 0) {
				entry.ptr = block_allocation_list[current_last_id];
			} else {
				ATOMIC_ADD(last_free_voxel_block_id, 1);
			}
		}
	}

	DECLARE_ATOMIC(int, last_free_voxel_block_id);
private:
	HashBlockVisibility* hash_block_visibility_types;
	int* block_allocation_list;
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct BuildUtilizedBlockListFunctor {
	BuildUtilizedBlockListFunctor(VoxelVolume<TVoxel, VoxelBlockHash>* volume) :
			utilized_block_list(volume->index.GetUtilizedBlockHashCodes()) {
		INITIALIZE_ATOMIC(int, utilized_block_count, 0);
	}

	~BuildUtilizedBlockListFunctor() {
		CLEAN_UP_ATOMIC(utilized_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code) {
		if (entry.ptr >= 0) {
			int current_utilized_block_index = ATOMIC_ADD(utilized_block_count, 1);
			utilized_block_list[current_utilized_block_index] = hash_code;
		}
	}

	DECLARE_ATOMIC(int, utilized_block_count);

private:
	int* utilized_block_list;
};

template<MemoryDeviceType TMemoryDeviceType>
struct VolumeBasedAllocationStateMarkerFunctor {
public:
	VolumeBasedAllocationStateMarkerFunctor(VoxelBlockHash& target_index) :
			colliding_block_positions(target_index.hash_entry_count, TMemoryDeviceType),
			hash_entry_allocation_states(target_index.GetHashEntryAllocationStates()),
			hash_block_coordinates(target_index.GetAllocationBlockCoordinates()),
			target_hash_table(target_index.GetEntries()),
			unresolvable_collision_encountered(1, true, TMemoryDeviceType == MEMORYDEVICE_CUDA) {
		colliding_block_positions_device = colliding_block_positions.GetData(TMemoryDeviceType);
		*unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU) = false;
		unresolvable_collision_encountered.UpdateDeviceFromHost();
		unresolvable_collision_encountered_device = unresolvable_collision_encountered.GetData(TMemoryDeviceType);
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
	}

	~VolumeBasedAllocationStateMarkerFunctor() {
		CLEAN_UP_ATOMIC(colliding_block_count);
	};

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& source_hash_entry, const int& source_hash_code) {
		ThreadAllocationStatus status = MarkAsNeedingAllocationIfNotFound<true>(
				hash_entry_allocation_states,
				hash_block_coordinates, source_hash_entry.pos,
				target_hash_table, colliding_block_positions_device, colliding_block_count);

		if (status == BEING_MODIFIED_BY_ANOTHER_THREAD) {
			*unresolvable_collision_encountered_device = true;
		}
	}

	ORUtils::MemoryBlock<Vector3s> colliding_block_positions;

	int getCollidingBlockCount() {
		return GET_ATOMIC_VALUE_CPU(colliding_block_count);
	}

	void resetFlagsAndCounters() {
		*this->unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU) = false;
		unresolvable_collision_encountered.UpdateDeviceFromHost();
		INITIALIZE_ATOMIC(int, colliding_block_count, 0);
	}

	bool encounteredUnresolvableCollision() {
		unresolvable_collision_encountered.UpdateHostFromDevice();
		return *this->unresolvable_collision_encountered.GetData(MEMORYDEVICE_CPU);
	}

protected:
	Vector3s* colliding_block_positions_device;
	DECLARE_ATOMIC(int, colliding_block_count);
	HashEntryAllocationState* hash_entry_allocation_states;
	Vector3s* hash_block_coordinates;
	HashEntry* target_hash_table;
	ORUtils::MemoryBlock<bool> unresolvable_collision_encountered;
	bool* unresolvable_collision_encountered_device;
};

template<MemoryDeviceType TMemoryDeviceType>
struct VolumeBasedBoundedAllocationStateMarkerFunctor :
		public VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType> {
public:
	VolumeBasedBoundedAllocationStateMarkerFunctor(VoxelBlockHash& target_index, const Extent3Di& bounds) :
			VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>(target_index), bounds(bounds) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& source_hash_entry, const int& source_hash_code) {
		Vector3i block_position_voxels = source_hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
		if (IsHashBlockFullyInBounds(block_position_voxels, bounds) ||
		    IsHashBlockPartiallyInBounds(block_position_voxels, bounds)) {
			ThreadAllocationStatus status = MarkAsNeedingAllocationIfNotFound<true>(
					hash_entry_allocation_states,
					hash_block_coordinates, source_hash_entry.pos,
					target_hash_table, colliding_block_positions_device, colliding_block_count);
			if (status == BEING_MODIFIED_BY_ANOTHER_THREAD) {
				*unresolvable_collision_encountered_device = true;
			}
		}
	}

	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_positions;

protected:
	const Extent3Di bounds;

	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_positions_device;
	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_count;
	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_entry_allocation_states;
	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_block_coordinates;
	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::target_hash_table;
	using VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::unresolvable_collision_encountered_device;
};

} // namespace ITMLib