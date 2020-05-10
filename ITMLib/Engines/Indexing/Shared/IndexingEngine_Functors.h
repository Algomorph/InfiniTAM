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
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "IndexingEngine_Shared.h"
#include "IndexingEngine_RayMarching.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"
#include "../../../Utils/VoxelVolumeParameters.h"

namespace ITMLib {


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct BlockListDeallocationFunctor {
public:
	Vector3s* colliding_positions_device = nullptr;

private: // member variables
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

public: // member functions
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
	void operator()(const Vector3s& block_position_to_clear, int i_new_block) {
		DeallocateBlock(block_position_to_clear, hash_entry_states, hash_table, voxels, colliding_positions_device,
		                colliding_block_count, last_free_voxel_block_id, last_free_excess_list_id,
		                block_allocation_list, excess_entry_list, empty_voxel_block_device);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct HashEntryStateBasedAllocationFunctor {
private: // member variables
	VoxelBlockHash& index;
	HashEntryAllocationState* hash_entry_states;
	Vector3s* block_coordinates;

	HashEntry* hash_table;
	const int* block_allocation_list;
	const int* excess_entry_list;
	int* utilized_block_hash_codes;

	DECLARE_ATOMIC(int, last_free_voxel_block_id);
	DECLARE_ATOMIC(int, last_free_excess_list_id);
	DECLARE_ATOMIC(int, utilized_block_count);
public: // member functions
	HashEntryStateBasedAllocationFunctor(VoxelBlockHash& index)
			: index(index),
			  hash_entry_states(index.GetHashEntryAllocationStates()),
			  block_coordinates(index.GetAllocationBlockCoordinates()),
			  hash_table(index.GetEntries()),

			  block_allocation_list(index.GetBlockAllocationList()),
			  excess_entry_list(index.GetExcessEntryList()),
			  utilized_block_hash_codes(index.GetUtilizedBlockHashCodes()) {
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, index.GetLastFreeBlockListId());
		INITIALIZE_ATOMIC(int, last_free_excess_list_id, index.GetLastFreeExcessListId());
		INITIALIZE_ATOMIC(int, utilized_block_count, index.GetUtilizedBlockCount());
	}

	~HashEntryStateBasedAllocationFunctor() {
		CLEAN_UP_ATOMIC(last_free_voxel_block_id);CLEAN_UP_ATOMIC(last_free_excess_list_id);CLEAN_UP_ATOMIC(
				utilized_block_count);
	}

	void UpdateIndexCounters() {
		this->index.SetLastFreeBlockListId(GET_ATOMIC_VALUE_CPU(last_free_voxel_block_id));
		this->index.SetLastFreeExcessListId(GET_ATOMIC_VALUE_CPU(last_free_excess_list_id));
		this->index.SetUtilizedBlockCount(GET_ATOMIC_VALUE_CPU(utilized_block_count));
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntryAllocationState& hash_entry_state, int hash_code) {
		AllocateBlockBasedOnState<TMemoryDeviceType>(
				hash_entry_state, hash_code, block_coordinates, hash_table,
				last_free_voxel_block_id, last_free_excess_list_id, utilized_block_count,
				block_allocation_list, excess_entry_list, utilized_block_hash_codes);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct BlockListAllocationStateMarkerFunctor {
public:
	Vector3s* colliding_positions_device = nullptr;

private: // member variables
	HashEntryAllocationState* hash_entry_states;
	Vector3s* allocation_block_coordinates;
	HashEntry* hash_table;
	DECLARE_ATOMIC(int, colliding_block_count);
public: // member functions
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
	void operator()(const Vector3s& desired_block_position, int i_new_block) {
		MarkAsNeedingAllocationIfNotFound<false>(hash_entry_states, allocation_block_coordinates,
		                                         desired_block_position, hash_table, colliding_positions_device,
		                                         colliding_block_count);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct DepthBasedAllocationStateMarkerFunctor {
public: // member variables
	ORUtils::MemoryBlock<Vector3s> colliding_block_positions;
protected: // member variables

	float near_clipping_distance;
	float far_clipping_distance;
	Matrix4f inverted_camera_pose;
	Vector4f inverted_projection_parameters;

	float surface_distance_cutoff;

	float hash_block_size_reciprocal;
	HashEntryAllocationState* hash_entry_allocation_states;
	Vector3s* hash_block_coordinates;
	HashEntry* hash_table;

	Vector3s* colliding_block_positions_device;
	DECLARE_ATOMIC(int, colliding_block_count);

	ORUtils::MemoryBlock<bool> unresolvable_collision_encountered;
	bool* unresolvable_collision_encountered_device;
public: // member functions
	DepthBasedAllocationStateMarkerFunctor(VoxelBlockHash& index,
	                                       const VoxelVolumeParameters& volume_parameters, const ITMLib::View* view,
	                                       Matrix4f depth_camera_pose, float surface_distance_cutoff) :

			near_clipping_distance(volume_parameters.near_clipping_distance),
			far_clipping_distance(volume_parameters.far_clipping_distance),
			inverted_projection_parameters(view->calib.intrinsics_d.projectionParamsSimple.all),

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
		depth_camera_pose.inv(inverted_camera_pose);
		inverted_projection_parameters.fx = 1.0f / inverted_projection_parameters.fx;
		inverted_projection_parameters.fy = 1.0f / inverted_projection_parameters.fy;
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
		//DEBUG alloc

		// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
		// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
		// point, in camera space
		ITMLib::Segment march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(surface_distance_cutoff,
		                                                                                       depth_measure,
		                                                                                       x, y,
		                                                                                       inverted_camera_pose,
		                                                                                       inverted_projection_parameters,
		                                                                                       hash_block_size_reciprocal);


		MarkVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
		                                *unresolvable_collision_encountered_device,
		                                hash_table, march_segment, colliding_block_positions_device,
		                                colliding_block_count);
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

template<MemoryDeviceType TMemoryDeviceType>
struct TwoSurfaceBasedAllocationStateMarkerFunctor
		: public DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType> {
public:
	TwoSurfaceBasedAllocationStateMarkerFunctor(VoxelBlockHash& index,
	                                            const VoxelVolumeParameters& volume_parameters,
	                                            const ITMLib::View* view,
	                                            const CameraTrackingState* tracking_state,
	                                            float surface_distance_cutoff) :
			DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>(index, volume_parameters, view,
			                                                          tracking_state->pose_d->GetM(),
			                                                          surface_distance_cutoff) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const float& surface1_depth, const Vector4f& surface2_point, int x, int y) {
		bool has_surface1 = false, has_surface2 = false;

		//_DEBUG alloc
//		if(x == 337 && y == 224){
//			int i = 10;
//		}
		if (!(surface1_depth <= 0 || (surface1_depth - surface_distance_cutoff) < 0 ||
		      (surface1_depth - surface_distance_cutoff) < near_clipping_distance ||
		      (surface1_depth + surface_distance_cutoff) > far_clipping_distance))
			has_surface1 = true;

		if (surface2_point.z > 0.0f) has_surface2 = true;

		ITMLib::Segment march_segment;

		if (has_surface1 && has_surface2) {
			Vector4f surface1_point = ImageSpacePointToCameraSpace(surface1_depth, x, y,
			                                                       inverted_projection_parameters);
			march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromAndBetweenTwoPoints(
					surface_distance_cutoff, surface1_point, surface2_point, inverted_camera_pose,
					hash_block_size_reciprocal);
		} else {
			if (has_surface1) {
				march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(
						surface_distance_cutoff, surface1_depth, x, y, inverted_camera_pose,
						inverted_projection_parameters, hash_block_size_reciprocal);
			} else if (has_surface2) {
				march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromPoint(
						surface_distance_cutoff, surface2_point, inverted_camera_pose, hash_block_size_reciprocal);
			} else {
				return; // neither surface is defined at this point, nothing to do.
			}
		}

		MarkVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
		                                *unresolvable_collision_encountered_device, hash_table, march_segment,
		                                colliding_block_positions_device, colliding_block_count);
	}

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_positions;
protected:
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::near_clipping_distance;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::far_clipping_distance;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::inverted_camera_pose;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::inverted_projection_parameters;

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::surface_distance_cutoff;

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_block_size_reciprocal;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_entry_allocation_states;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_block_coordinates;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::hash_table;

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_positions_device;
	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::colliding_block_count;

	using DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType>::unresolvable_collision_encountered_device;
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