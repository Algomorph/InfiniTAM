//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/4/20.
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

#include "IndexingEngine_RayMarching.h"
#include "IndexingEngine_DiagnosticData.h"

namespace ITMLib {

_DEVICE_WHEN_AVAILABLE_ inline void
TryToMarkBlockForAllocation_RecordPixelBlocks(
		const Vector3s& block_position,
		ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
		Vector3s* hash_block_coordinates,
		bool& unresolvable_collision_encountered,
		const CONSTPTR(HashEntry)* hash_table,
		THREADPTR(Vector3s)* colliding_block_positions,
		ATOMIC_ARGUMENT(int) colliding_block_count,

		internal::PixelBlockAllocationRecord& pixel_blocks,
		int& pixel_block_count) {

	ThreadAllocationStatus resulting_status = MarkAsNeedingAllocationIfNotFound<true>(
			hash_entry_allocation_states,
			hash_block_coordinates, block_position,
			hash_table, colliding_block_positions, colliding_block_count);

	//_DEBUG alloc
	switch (resulting_status) {
		case BEING_MODIFIED_BY_ANOTHER_THREAD:
			unresolvable_collision_encountered = true;
			break;
		case MARKED_FOR_ALLOCATION_IN_ORDERED_LIST:
		case MARKED_FOR_ALLOCATION_IN_EXCESS_LIST: {
			if (pixel_block_count < internal::max_pixel_block_count) {
				pixel_blocks.values[pixel_block_count] = block_position;
				pixel_block_count++;
			}
		}
			break;
		default:
			break;
	}
}


_DEVICE_WHEN_AVAILABLE_ inline void
MarkVoxelHashBlocksAlongSegment_RecordPixelBlocks(
		ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
		Vector3s* hash_block_coordinates,
		bool& unresolvable_collision_encountered,
		const CONSTPTR(HashEntry)* hash_table,
		const ITMLib::Segment& segment_in_hash_blocks,
		THREADPTR(Vector3s)* colliding_block_positions,
		ATOMIC_ARGUMENT(int) colliding_block_count,

		internal::PixelBlockAllocationRecord& pixel_blocks,
		int& pixel_block_count) {

// number of steps to take along the truncated SDF band
	int step_count = (int) std::ceil(2.0f * segment_in_hash_blocks.length());

// a single stride along the sdf band segment from one step to the next
	Vector3f stride_vector = segment_in_hash_blocks.vector_to_destination / (float) (step_count - 1);

	Vector3s previous_block_position;
	Vector3f check_position = segment_in_hash_blocks.origin;

//add neighbouring blocks
	for (int i = 0; i < step_count; i++) {
//find block position at current step
		Vector3s current_block_position = TO_SHORT_FLOOR3(check_position);
		int directional_increment_count;
		if (i > 0 && (directional_increment_count = GetDifferingComponentCount(current_block_position,
		                                                                       previous_block_position)) > 1) {
			if (directional_increment_count == 2) {
				for (int i_direction = 0; i_direction < 3; i_direction++) {
					if (current_block_position.values[i_direction] != previous_block_position.values[i_direction]) {
						Vector3s potentially_missed_block_position = previous_block_position;
						potentially_missed_block_position.values[i_direction] = current_block_position.values[i_direction];
						if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
						                                       TO_FLOAT3(potentially_missed_block_position),
						                                       1.0f)) {
							TryToMarkBlockForAllocation_RecordPixelBlocks(
									potentially_missed_block_position,
									hash_entry_allocation_states,
									hash_block_coordinates,
									unresolvable_collision_encountered,
									hash_table,
									colliding_block_positions,
									colliding_block_count,

									pixel_blocks,
									pixel_block_count
							);
						}
					}
				}
			} else {
//directional_increment_count == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentially_missed_block_position = previous_block_position;
					potentially_missed_block_position.values[iDirection] = current_block_position.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentially_missed_block_position),
					                                       1.0f)) {
						TryToMarkBlockForAllocation_RecordPixelBlocks(
								potentially_missed_block_position,
								hash_entry_allocation_states,
								hash_block_coordinates,
								unresolvable_collision_encountered,
								hash_table,
								colliding_block_positions,
								colliding_block_count,

								pixel_blocks,
								pixel_block_count
						);
					}
					potentially_missed_block_position = current_block_position;
					potentially_missed_block_position.values[iDirection] = previous_block_position.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentially_missed_block_position),
					                                       1.0f)) {
						TryToMarkBlockForAllocation_RecordPixelBlocks(
								potentially_missed_block_position,
								hash_entry_allocation_states,
								hash_block_coordinates,
								unresolvable_collision_encountered,
								hash_table,
								colliding_block_positions,
								colliding_block_count,

								pixel_blocks,
								pixel_block_count
						);
					}
				}
			}
		}
		TryToMarkBlockForAllocation_RecordPixelBlocks(
				current_block_position,
				hash_entry_allocation_states,
				hash_block_coordinates,
				unresolvable_collision_encountered,
				hash_table,
				colliding_block_positions,
				colliding_block_count,

				pixel_blocks,
				pixel_block_count

		);
		check_position += stride_vector;
		previous_block_position = current_block_position;
	}
}

} // namespace ITMLib
