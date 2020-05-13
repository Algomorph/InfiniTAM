//  ================================================================
//  Created by Gregory Kramida on 8/29/19.
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
//stdlib
#include <array>
#include <unordered_set>

//local
#include "../Shared/VolumeTraversal_Shared.h"
#include "../Interface/TwoVolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/Analytics/IsAltered.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"
#include "../../../Utils/ExecutionMode.h"

namespace ITMLib {


template<typename TArrayVoxel, typename THashVoxel>
class TwoVolumeTraversalEngine<TArrayVoxel, THashVoxel, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU> {
private:
	inline static bool BlockIsCompletelyOutsideArray(const Vector3i& block_max_voxels,
	                                                 const Vector3i& block_min_voxels,
	                                                 const Vector3i& array_min_voxels,
	                                                 const Vector3i& array_max_voxels) {
		return block_max_voxels.x < array_min_voxels.x || block_min_voxels.x > array_max_voxels.x ||
		       block_max_voxels.y < array_min_voxels.y || block_min_voxels.y > array_max_voxels.y ||
		       block_max_voxels.z < array_min_voxels.z || block_min_voxels.z > array_max_voxels.z;
	}

	inline static bool BlockAtLeastPartiallyOutsideArray(const Vector3i& block_max_voxels,
	                                                     const Vector3i& block_min_voxels,
	                                                     const Vector3i& array_min_voxels,
	                                                     const Vector3i& array_max_voxels) {
		return block_max_voxels.x > array_max_voxels.x || block_min_voxels.x < array_min_voxels.x ||
		       block_max_voxels.y > array_max_voxels.y || block_min_voxels.y < array_min_voxels.y ||
		       block_max_voxels.z > array_max_voxels.z || block_min_voxels.z < array_min_voxels.z;
	}

	template<typename TTwoVoxelBooleanFunctor, typename TTwoVoxelAndPositionPredicate,
			typename TArrayVoxelPredicate, typename THashVoxelPredicate>
	inline static bool HashBlocksNotFullyOverlappingArrayAllMatch(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TTwoVoxelBooleanFunctor& voxels_match_functor,
			TTwoVoxelAndPositionPredicate&& voxels_at_position_are_significant,
			TArrayVoxelPredicate&& array_voxel_is_significant_if_altered,
			THashVoxelPredicate&& hash_voxel_is_significant_if_altered,
			bool verbose
	) {
		const int hash_entry_count = hash_volume->index.hash_entry_count;
		HashEntry* hash_table = hash_volume->index.GetIndexData();
		GridAlignedBox* array_info = array_volume->index.GetIndexData();

		TArrayVoxel* array_voxels = array_volume->GetVoxels();
		THashVoxel* hash_voxels = hash_volume->GetVoxels();

		Vector3i array_min_voxels = array_info->offset;
		Vector3i array_max_voxels = array_info->offset + array_info->size;

		bool mismatch_found = false;
		// *** find voxel blocks in hash not spanned by array volume and inspect if any are altered ***
		// *** also find blocks partially overlapping array, and inspect if those outside array are altered and those inside match ***
#ifdef WITH_OPENMP
#pragma omp parallel for //default(none), shared(mismatch_found, hash_table, array_min_voxels, array_max_voxels, verbose)
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			if (mismatch_found) continue;
			HashEntry& hash_entry = hash_table[hash_code];
			if (hash_entry.ptr < 0) {
				continue;
			}
			Vector3i block_min_voxels = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i block_max_voxels = block_min_voxels + Vector3i(VOXEL_BLOCK_SIZE);
			if (BlockIsCompletelyOutsideArray(block_min_voxels, block_max_voxels, array_min_voxels, array_max_voxels)) {
				if (verbose) {
					if (isVoxelBlockAlteredPredicate(hash_voxels + hash_entry.ptr * VOXEL_BLOCK_SIZE3,
					                                 hash_voxel_is_significant_if_altered, true,
					                                 "Hash voxel unmatched in array: ", hash_entry.pos, hash_code)) {
						mismatch_found = true;
					}
				} else {
					if (isVoxelBlockAlteredPredicate(hash_voxels + hash_entry.ptr * VOXEL_BLOCK_SIZE3,
					                                 hash_voxel_is_significant_if_altered)) {
						mismatch_found = true;
					}
				}
			} else if (BlockAtLeastPartiallyOutsideArray(block_min_voxels, block_max_voxels, array_min_voxels,
			                                             array_max_voxels)) {
				// block is not completely outside but is at least partially outside == block is partially outside for sure
				THashVoxel* hash_voxel_block = hash_voxels + hash_entry.ptr * VOXEL_BLOCK_SIZE3;
				for (int linear_index_in_block = 0;
				     linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
					THashVoxel& hash_voxel = hash_voxel_block[linear_index_in_block];
					if (std::forward<THashVoxelPredicate>(hash_voxel_is_significant_if_altered)(hash_voxel) &&
					    isAltered(hash_voxel)) {
						Vector3i voxel_position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(hash_entry.pos,
						                                                                              linear_index_in_block);
						if (IsPointInBounds(voxel_position, *array_info)) {
							int linear_index_in_array = ComputeLinearIndexFromPosition_PlainVoxelArray(array_info,
							                                                                           voxel_position);
							TArrayVoxel& array_voxel = array_voxels[linear_index_in_array];
							if (!std::forward<TTwoVoxelAndPositionPredicate>(voxels_at_position_are_significant)(
									voxels_match_functor, array_voxel, hash_voxel,
									voxel_position)) {
								mismatch_found = true;
							}
						} else {
							if (verbose) {
								isAltered_VerbosePositionHash(hash_voxel, voxel_position, hash_code, hash_entry.pos,
								                              "Hash voxel unmatched in array: ");
							}
							mismatch_found = true;
						}
					}
				}
			}
		}
		return !mismatch_found;
	}

	template<typename TTwoVoxelBooleanFunctor, typename TTwoVoxelAndPositionPredicate,
			typename TArrayVoxelPredicate, typename THashVoxelPredicate>
	inline static bool
	TraversalAndCompareAll_Generic_Diagnostic(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TTwoVoxelBooleanFunctor& voxels_match_functor,
			TTwoVoxelAndPositionPredicate&& voxels_at_position_are_significant,
			TArrayVoxelPredicate&& array_voxel_is_significant_if_altered,
			THashVoxelPredicate&& hash_voxel_is_significant_if_altered,
			bool verbose) {
		if (!HashBlocksNotFullyOverlappingArrayAllMatch(array_volume, hash_volume,
		                                                voxels_match_functor,
		                                                voxels_at_position_are_significant,
		                                                array_voxel_is_significant_if_altered,
		                                                hash_voxel_is_significant_if_altered,
		                                                verbose)) {
			return false;
		}

		HashEntry* hash_table = hash_volume->index.GetIndexData();
		const GridAlignedBox* array_info = array_volume->index.GetIndexData();
		TArrayVoxel* array_voxels = array_volume->GetVoxels();
		THashVoxel* hash_voxels = hash_volume->GetVoxels();

		const Vector3i array_min_voxels = array_info->offset;
		const Vector3i array_max_voxels = array_info->offset + array_info->size;

#ifndef WITH_OPENMP
		VoxelBlockHash::IndexCache cache;
#endif
		bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int z_voxel = array_min_voxels.z; z_voxel < array_max_voxels.z; z_voxel++) {
			if (mismatch_found) continue;
			for (int y_voxel = array_min_voxels.y; y_voxel < array_max_voxels.y; y_voxel++) {
				for (int x_voxel = array_min_voxels.x; x_voxel < array_max_voxels.x; x_voxel++) {
					Vector3i voxel_position(x_voxel, y_voxel, z_voxel);
					const int linear_index_in_array = ComputeLinearIndexFromPosition_PlainVoxelArray(array_info,
					                                                                                 voxel_position);
					TArrayVoxel& array_voxel = array_voxels[linear_index_in_array];
					if (std::forward<TArrayVoxelPredicate>(array_voxel_is_significant_if_altered)(array_voxel)) {
						int vm_index;
#ifndef WITH_OPENMP
						int hash_voxel_index = findVoxel(hash_table, voxel_position, vm_index, cache);
#else
						int hash_voxel_index = findVoxel(hash_table, voxel_position, vm_index);
#endif
						if (hash_voxel_index == -1 &&
						    ((verbose && isAltered_VerbosePosition(
								    array_voxel, voxel_position,
								    "Array voxel not matched in hash: ")) || isAltered(array_voxel))) {
							// no hash block intersecting array at this point, if array voxel is altered: fail
							mismatch_found = true;
						}
					}
				}
			}
		}
		return !mismatch_found;
	}


	template<typename TTwoVoxelBooleanFunctor, typename TTwoVoxelAndPositionPredicate,
			typename TArrayVoxelPredicate, typename THashVoxelPredicate>
	inline static bool
	TraversalAndCompareAll_Generic_Optimized(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TTwoVoxelBooleanFunctor& voxels_match_functor,
			TTwoVoxelAndPositionPredicate&& voxels_at_position_are_significant,
			TArrayVoxelPredicate&& array_voxel_is_significant_if_altered,
			THashVoxelPredicate&& hash_voxel_is_significant_if_altered,
			bool verbose
	) {
		if (!HashBlocksNotFullyOverlappingArrayAllMatch(array_volume, hash_volume,
		                                                voxels_match_functor,
		                                                voxels_at_position_are_significant,
		                                                array_voxel_is_significant_if_altered,
		                                                hash_voxel_is_significant_if_altered,
		                                                verbose)) {
			return false;
		}

		HashEntry* hash_table = hash_volume->index.GetIndexData();
		GridAlignedBox* array_info = array_volume->index.GetIndexData();
		TArrayVoxel* array_voxels = array_volume->GetVoxels();
		THashVoxel* hash_voxels = hash_volume->GetVoxels();
		volatile bool mismatch_found = false;

#ifndef WITH_OPENMP
		VoxelBlockHash::IndexCache cache;
#endif

		// *** Checks whether voxels on the margin of the array that don't overlap any voxel blocks are altered ***
		auto margin_extent_has_mismatch = [&](const Extent3Di& extent) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
			for (int z = extent.min_z; z < extent.max_z; z++) {
				if (mismatch_found) continue;
				for (int y = extent.min_y; y < extent.max_y; y++) {
					for (int x = extent.min_x; x < extent.max_x; x++) {
						Vector3i position(x, y, z);
						int linear_array_index = ComputeLinearIndexFromPosition_PlainVoxelArray(array_info, position);
						TArrayVoxel& array_voxel = array_voxels[linear_array_index];
						if (std::forward<TArrayVoxelPredicate>(array_voxel_is_significant_if_altered)(array_voxel)) {
							int vm_index;
#ifndef WITH_OPENMP
							int hash_voxel_index = findVoxel(hash_table, position, vm_index, cache);
#else
							int hash_voxel_index = findVoxel(hash_table, position, vm_index);
#endif
							if (hash_voxel_index == -1 &&
							    ((verbose && isAltered_VerbosePosition(
									    array_voxel, position,
									    "Array voxel not matched in hash: ")) || isAltered(array_voxel))) {
								// no hash block intersecting array at this point, if array voxel is altered: fail
								mismatch_found = true;
							}
						}
					}
				}
			}
			return mismatch_found;
		};

		// *** checks all voxels inside array, return false if one's altered but unallocated in the hash or if
		// applying the voxels_match_functor on it and the corresponding one from the hash-block volume returns false ***

		auto central_extent_has_mismatch = [&](const Extent3Di& extent) {
			Extent3Di hash_block_extent = extent / VOXEL_BLOCK_SIZE;
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
			for (int z_block = hash_block_extent.min_z; z_block < hash_block_extent.max_z; z_block++) {
				if (mismatch_found) continue;
				int z_voxel_start = z_block * VOXEL_BLOCK_SIZE, z_voxel_end = z_voxel_start + VOXEL_BLOCK_SIZE;
				for (int y_block = hash_block_extent.min_y; y_block < hash_block_extent.max_y; y_block++) {
					if (mismatch_found) continue;
					int y_voxel_start = y_block * VOXEL_BLOCK_SIZE, y_voxel_end = y_voxel_start + VOXEL_BLOCK_SIZE;
					for (int x_block = hash_block_extent.min_x; x_block < hash_block_extent.max_x; x_block++) {
						if (mismatch_found) continue;
						int x_voxel_start = x_block * VOXEL_BLOCK_SIZE, x_voxel_end = x_voxel_start + VOXEL_BLOCK_SIZE;
						Vector3s voxel_block_position(x_block, y_block, z_block);
						int hash;
						TArrayVoxel* hash_block_voxels = nullptr;
						if (FindHashAtPosition(hash, voxel_block_position, hash_table)) {
							hash_block_voxels = hash_voxels + hash_table[hash].ptr * VOXEL_BLOCK_SIZE3;
							// traverse the current voxel block volume to see if PVA volume's voxels satisfy
							// voxels_match_functor when matched with corresponding voxels from the VBH volume.
							int idWithinBlock = 0;
							for (int z_absolute = z_voxel_start; z_absolute < z_voxel_end; z_absolute++) {
								for (int y_absolute = y_voxel_start; y_absolute < y_voxel_end; y_absolute++) {
									for (int x_absolute = x_voxel_start;
									     x_absolute < x_voxel_end; x_absolute++, idWithinBlock++) {
										Vector3i positionAbsolute(x_absolute, y_absolute, z_absolute);
										int linear_array_index =
												ComputeLinearIndexFromPosition_PlainVoxelArray(array_info,
												                                               positionAbsolute);
										TArrayVoxel& array_voxel = array_voxels[linear_array_index];
										THashVoxel& hash_voxel = hash_block_voxels[idWithinBlock];
										if (!std::forward<TTwoVoxelAndPositionPredicate>(
												voxels_at_position_are_significant)(
												voxels_match_functor, array_voxel, hash_voxel, positionAbsolute)) {
											mismatch_found = true;
										}
									}
								}
							}
						} else {
							// If the block is not allocated in the VBH volume at all but the PVA voxel is modified,
							// short-circuit to "false"
							for (int z_absolute = z_voxel_start; z_absolute < z_voxel_end; z_absolute++) {
								for (int y_absolute = y_voxel_start; y_absolute < y_voxel_end; y_absolute++) {
									for (int x_absolute = x_voxel_start; x_absolute < x_voxel_end; x_absolute++) {
										Vector3i positionAbsolute(x_absolute, y_absolute, z_absolute);
										int linear_array_index = ComputeLinearIndexFromPosition_PlainVoxelArray(
												array_info,
												positionAbsolute);
										int vm_index;
										// no hash block intersecting array at this point, yet voxel is altered: fail
										TArrayVoxel& array_voxel = array_voxels[linear_array_index];
										if (verbose) {
											if (std::forward<TArrayVoxelPredicate>(
													array_voxel_is_significant_if_altered)(
													array_voxel)
											    && isAltered_VerbosePosition(array_voxel, positionAbsolute,
											                                 "Array voxel not matched in hash: ")) {
												mismatch_found = true;
											}
										} else {
											if (std::forward<TArrayVoxelPredicate>(
													array_voxel_is_significant_if_altered)(
													array_voxel) && isAltered(array_voxel)) {
												mismatch_found = true;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			return mismatch_found;
		};

		// *** compute_allocated extents ***
		Extent3Di central_extent;
		std::vector<Extent3Di> border_extents = ComputeBoxSetOfHashAlignedCenterAndNonHashBlockAlignedArrayMargins(
				*array_info, central_extent);
		//_DEBUG alloc
		int i_extent = 0;
		int in_extent = -1;
		Vector3i point(305, 314, 167);

		for (auto& extent : border_extents) {
			if (margin_extent_has_mismatch(extent)) return false;
			in_extent = IsPointInBounds(point, extent) ? i_extent : in_extent;
			i_extent++;
		}

		//_DEBUG alloc
		bool in_central_extent = IsPointInBounds(point, central_extent);
		Extent3Di array_extent = PVA_InfoToExtent(*array_info);
		bool in_array = IsPointInBounds(point, array_extent);

		return !central_extent_has_mismatch(central_extent);
	}

	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TTwoVoxelBooleanFunctor, typename TTwoVoxelAndPositionPredicate,
			typename TArrayVoxelPredicate, typename THashVoxelPredicate>
	inline static bool TraversalAndCompareAll_Generic(VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
	                                                  VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
	                                                  TTwoVoxelBooleanFunctor& voxels_match_functor,
	                                                  TTwoVoxelAndPositionPredicate&& voxels_at_position_are_significant,
	                                                  TArrayVoxelPredicate&& array_voxel_is_significant_if_altered,
	                                                  THashVoxelPredicate&& hash_voxel_is_significant_if_altered,
	                                                  bool verbose) {
		switch (TExecutionMode) {
			case ExecutionMode::DIAGNOSTIC:
				return TraversalAndCompareAll_Generic_Diagnostic(array_volume, hash_volume, voxels_match_functor,
				                                                 voxels_at_position_are_significant,
				                                                 array_voxel_is_significant_if_altered,
				                                                 hash_voxel_is_significant_if_altered, verbose);
			case ExecutionMode::OPTIMIZED:
				return TraversalAndCompareAll_Generic_Optimized(array_volume, hash_volume, voxels_match_functor,
				                                                voxels_at_position_are_significant,
				                                                array_voxel_is_significant_if_altered,
				                                                hash_voxel_is_significant_if_altered, verbose);
		}
	};

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	TravereAndCompareAllocated_Generic(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TFunctor& functor, TFunctionCall&& functionCall) {
		volatile bool mismatch_found = false;
		int hash_entry_count = hash_volume->index.hash_entry_count;
		THashVoxel* hash_voxels = hash_volume->GetVoxels();
		TArrayVoxel* array_voxels = array_volume->GetVoxels();
		const VoxelBlockHash::IndexData* hash_table = hash_volume->index.GetIndexData();
		const PlainVoxelArray::IndexData* array_info = array_volume->index.GetIndexData();
		Vector3i start_voxel = array_info->offset;
		Vector3i array_size = array_info->size;
		Vector3i end_voxel = start_voxel + array_size; // open last traversal bound (this voxel doesn't get processed)
		Vector6i array_bounds(start_voxel.x, start_voxel.y, start_voxel.z, end_voxel.x, end_voxel.y, end_voxel.z);

#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int hash = 0; hash < hash_entry_count; hash++) {
			if (mismatch_found) continue;
			HashEntry hash_entry = hash_table[hash];
			if (hash_entry.ptr < 0) continue;
			Vector3i voxel_block_min_point = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hash_entry_max_point = voxel_block_min_point + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(voxel_block_min_point, hash_entry_max_point, array_bounds)) {
				continue;
			}
			THashVoxel* hash_voxel_block = &(hash_voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			Vector6i local_bounds = ComputeLocalBounds(voxel_block_min_point, hash_entry_max_point, array_bounds);
			for (int z = local_bounds.min_z; z < local_bounds.max_z; z++) {
				for (int y = local_bounds.min_y; y < local_bounds.max_y; y++) {
					for (int x = local_bounds.min_x; x < local_bounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_coordinate_within_block = Vector3i(x, y, z);
						Vector3i voxel_absolute_position = voxel_block_min_point + voxel_coordinate_within_block;
						Vector3i voxel_positionSansOffset = voxel_absolute_position - start_voxel;
						int linear_index = voxel_positionSansOffset.x + voxel_positionSansOffset.y * array_size.x +
						                   voxel_positionSansOffset.z * array_size.x * array_size.y;
						THashVoxel& hash_voxel = hash_voxel_block[locId];
						TArrayVoxel& array_voxel = array_voxels[linear_index];

						if (!std::forward<TFunctionCall>(functionCall)(functor, array_voxel, hash_voxel,
						                                               voxel_absolute_position)) {
							mismatch_found = true;
							break;
						}
					}
				}
			}
		}
		return !mismatch_found;

	}

public: // static functions

	static inline
	std::vector<Extent3Di>
	ComputeBoxSetOfHashAlignedCenterAndNonHashBlockAlignedArrayMargins(
			const ITMLib::GridAlignedBox& array_info, Extent3Di& center_extent) {

		//TODO: code can probably be condensed throughout to act on a per-dimension basis using Vector indexing, 
		// but not sure if code clarity will be preserved

		Vector3i array_bounds_min = array_info.offset;
		Vector3i array_bounds_max = array_bounds_min + array_info.size;

		// compute how many voxels on each side of the array extend past the first hash block fully inside the array.
		// these are the margin thicknesses, in voxels
		struct DirectionMarginSet {
			int width_margin_near;
			int width_margin_far;
//			int margin_near_start;
//			int margin_near_end;
//			Extent3Di extent_near;
//			Extent2Di extent_far;
		};

		std::array<DirectionMarginSet, 3> per_direction_margin_lengths;
		DirectionMarginSet& direction_x = per_direction_margin_lengths[0];
		DirectionMarginSet& direction_y = per_direction_margin_lengths[1];
		DirectionMarginSet& direction_z = per_direction_margin_lengths[2];

		for (int i_direction = 0; i_direction < per_direction_margin_lengths.size(); i_direction++) {
			int margin_near = array_bounds_min.values[i_direction] % VOXEL_BLOCK_SIZE;
			if (margin_near < 0) {
				margin_near += VOXEL_BLOCK_SIZE;
			}
			per_direction_margin_lengths[i_direction].width_margin_near = margin_near;
			int margin_far = -(array_bounds_max.values[i_direction]) % VOXEL_BLOCK_SIZE;
			if (margin_far < 0) {
				margin_far += VOXEL_BLOCK_SIZE;
			}
			per_direction_margin_lengths[i_direction].width_margin_far = margin_near;
		}

		// use the margin thicknesses to construct the 6 blocks, one for each face. The boxes should not overlap.
		int margin_near_x_end = array_bounds_min.x + direction_x.width_margin_near;
		int margin_far_x_start = array_bounds_max.x - direction_x.width_margin_far;
		int margin_near_y_end = array_bounds_min.y + direction_y.width_margin_near;
		int margin_far_y_start = array_bounds_max.y - direction_y.width_margin_far;
		int margin_near_z_end = array_bounds_min.z + direction_x.width_margin_near;
		int margin_far_z_start = array_bounds_max.z - direction_y.width_margin_far;

		std::vector<Extent3Di> allExtents = {
				Extent3Di{array_bounds_min.x, array_bounds_min.y, array_bounds_min.z,
				          margin_near_x_end, array_bounds_max.y, array_bounds_max.z},
				Extent3Di{margin_far_x_start, array_bounds_min.y, array_bounds_min.z,
				          array_bounds_max.x, array_bounds_max.y, array_bounds_max.z},

				Extent3Di{margin_near_x_end, array_bounds_min.y, array_bounds_min.z,
				          margin_far_x_start, margin_near_y_end, array_bounds_max.z},
				Extent3Di{margin_near_x_end, margin_far_y_start, array_bounds_min.z,
				          margin_far_x_start, array_bounds_max.y, array_bounds_max.z},

				Extent3Di{margin_near_x_end, margin_near_y_end, array_bounds_min.z,
				          margin_far_x_start, margin_far_z_start, margin_near_z_end},
				Extent3Di{margin_near_x_end, margin_near_y_end, margin_far_z_start,
				          margin_far_x_start, margin_far_z_start, array_bounds_max.z},
		};

		std::array<int, 6> margins = {
				direction_x.width_margin_near, direction_x.width_margin_far,
				direction_y.width_margin_near, direction_y.width_margin_far,
				direction_z.width_margin_near, direction_z.width_margin_far
		};

		std::vector<Extent3Di> non_zero_extents;
		for (int i_margin = 0; i_margin < margins.size(); i_margin++) {
			if (margins[i_margin] > 0) non_zero_extents.push_back(allExtents[i_margin]);
		}
		// add central extent
		center_extent = {margin_near_x_end, margin_near_y_end, margin_near_z_end,
		                 margin_far_x_start, margin_far_y_start, margin_far_z_start};
		return non_zero_extents;
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param array_volume the primary volume -- indexed using plain voxel array (PVA)
	 * \param hash_volume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TFunctor& functor, bool verbose) {
		return TraversalAndCompareAll_Generic<TExecutionMode>(array_volume, hash_volume, functor, []
				(TFunctor& functor1, TArrayVoxel& array_voxel,
				 THashVoxel& hash_voxel, const Vector3i& voxel_position) {
			return functor1(array_voxel, hash_voxel);
		}, [](TArrayVoxel& array_voxel) { return true; }, [](THashVoxel& hash_voxel) { return true; }, verbose);
	}


	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED,typename TFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TFunctor& functor, bool verbose) {
		return TraversalAndCompareAll_Generic<TExecutionMode>(array_volume, hash_volume, functor, []
				(TFunctor& functor, TArrayVoxel& array_voxel, THashVoxel& hash_voxel,
				 const Vector3i& voxel_position) {
			return functor(array_voxel, hash_voxel, voxel_position);
		}, [](TArrayVoxel& array_voxel) { return true; }, [](THashVoxel& hash_voxel) { return true; }, verbose);
	}

	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED,typename TFunctor>
	inline static bool
	TraverseAndCompareMatchingFlags(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			VoxelFlags flags,
			TFunctor& functor, bool verbose) {
		return TraversalAndCompareAll_Generic<TExecutionMode>(
				array_volume, hash_volume, functor,
				[&flags](TFunctor& functor1,
				         TArrayVoxel& array_voxel, THashVoxel& hash_voxel, const Vector3i& voxel_position) {
					return (array_voxel.flags != flags && hash_voxel.flags != flags) ||
					       functor1(array_voxel, hash_voxel);
				},
				[&flags](TArrayVoxel& array_voxel) { return array_voxel.flags == flags; },
				[&flags](THashVoxel& hash_voxel) { return hash_voxel.flags == flags; }, verbose);
	}


	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TFunctor>
	inline static bool
	TraverseAndCompareMatchingFlagsWithPosition(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			VoxelFlags flags, TFunctor& functor, bool verbose) {
		return TraversalAndCompareAll_Generic<TExecutionMode>(
				array_volume, hash_volume, functor,
				[&flags](TFunctor& functor,
				         TArrayVoxel& array_voxel, THashVoxel& hash_voxel, const Vector3i& voxel_position) {
					return (array_voxel.flags != flags && hash_voxel.flags != flags) ||
					       functor(array_voxel, hash_voxel, voxel_position);
				},
				[&flags](TArrayVoxel& array_voxel) { return array_voxel.flags == flags; },
				[&flags](THashVoxel& hash_voxel) { return hash_voxel.flags == flags; }, verbose);
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location, which ignores unallocated spaces in either scene.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated. Areas where either or both of the scenes don't have allocated voxels are
	 * ignored, even if only one of the volumes does, in fact, have potentially-altered voxels there.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param array_volume the primary volume -- indexed using plain voxel array (PVA)
	 * \param hash_volume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllocated(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TFunctor& functor) {
		return TravereAndCompareAllocated_Generic(array_volume, hash_volume, functor, []
				(TFunctor& functor, TArrayVoxel& voxelPrimary, THashVoxel& voxelSecondary,
				 const Vector3i& voxel_position) {
			return functor(voxelPrimary, voxelSecondary);
		});
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location, which ignores unallocated spaces in either scene.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated. Areas where either or both of the scenes don't have allocated voxels are
	 * ignored, even if only one of the volumes does, in fact, have potentially-altered voxels there.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param array_volume the primary volume -- indexed using plain voxel array (PVA)
	 * \param hash_volume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels and their mutually shared position by reference as arguments
	 * and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllocatedWithPosition(
			VoxelVolume<TArrayVoxel, PlainVoxelArray>* array_volume,
			VoxelVolume<THashVoxel, VoxelBlockHash>* hash_volume,
			TFunctor& functor) {
		return TravereAndCompareAllocated_Generic(array_volume, hash_volume, functor, []
				(TFunctor& functor, TArrayVoxel& voxelPrimary, THashVoxel& voxelSecondary,
				 Vector3i& voxel_position) {
			return functor(voxelPrimary, voxelSecondary, voxel_position);
		});
	}


};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, VoxelBlockHash, PlainVoxelArray, MEMORYDEVICE_CPU> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param hash_volume the primary volume -- indexed using voxel block hash table (VBH)
	 * \param array_volume the secondary volume -- indexed using plain voxel array (PVA)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* hash_volume,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* array_volume,
			TFunctor& functor, bool verbose) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flip_functor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU>::
		TraverseAndCompareAll(array_volume, hash_volume, flip_functor, verbose);

	}


	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllocated(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* hash_volume,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* array_volume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flip_functor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU>::
		TraverseAndCompareAllocated(array_volume, hash_volume, flip_functor);
	}
};
} // namespace ITMLib