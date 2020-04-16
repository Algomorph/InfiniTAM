//  ================================================================
//  Created by Gregory Kramida on 11/5/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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

#include "EditAndCopyEngine_CPU.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../Traversal/Shared/VolumeTraversal_Shared.h"
#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../Shared/EditAndCopyEngine_Functors.h"


using namespace ITMLib;


// region ==================================== Voxel Array Volume EditAndCopy Engine ===================================

template<typename TVoxel>
void ITMLib::EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::ResetVolume(
		ITMLib::VoxelVolume<TVoxel, ITMLib::PlainVoxelArray>* volume) {
	const unsigned int voxel_count = volume->index.GetMaxVoxelCount();
	TVoxel* voxels = volume->GetVoxels();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels)
#endif
	for (unsigned int i_voxel = 0; i_voxel < voxel_count; ++i_voxel) voxels[i_voxel] = TVoxel();
}

template<typename TVoxel>
bool
EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::SetVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
                                                         Vector3i at, TVoxel voxel) {
	int voxel_found = 0;
	int linear_index_in_array = findVoxel(volume->index.GetIndexData(), at, voxel_found);
	if (voxel_found) {
		volume->GetVoxels()[linear_index_in_array] = voxel;
		return true;
	} else {
		return false;
	}

}

template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
                                                          Vector3i at) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(volume->index.GetIndexData(), at, vmIndex);
	if (arrayIndex < 0) {
		TVoxel voxel;
		return voxel;
	}
	return volume->GetVoxels()[arrayIndex];
}

template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
                                                          Vector3i at,
                                                          PlainVoxelArray::IndexCache& cache) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(volume->index.GetIndexData(), at, vmIndex);
	if (arrayIndex < 0) {
		TVoxel voxel;
		return voxel;
	}
	return volume->GetVoxels()[arrayIndex];
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::
IsPointInBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const Vector3i& point) {
	ITMLib::PlainVoxelArray::IndexData* index_bounds = volume->index.GetIndexData();
	Vector3i point2 = point - index_bounds->offset;
	return !((point2.x < 0) || (point2.x >= index_bounds->size.x) ||
	         (point2.y < 0) || (point2.y >= index_bounds->size.y) ||
	         (point2.z < 0) || (point2.z >= index_bounds->size.z));
}


template<typename TVoxel>
void
EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::OffsetWarps(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume,
		Vector3f offset) {
	OffsetWarpsFunctor<TVoxel, PlainVoxelArray, TVoxel::hasCumulativeWarp>::OffsetWarps(volume, offset);
}


template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::CopyVolumeSlice(
		VoxelVolume<TVoxel, PlainVoxelArray>* target_volume, VoxelVolume<TVoxel, PlainVoxelArray>* source_volume,
		Vector6i bounds, const Vector3i& offset) {

	Vector3i min_pt_source = Vector3i(bounds.min_x, bounds.min_y, bounds.min_z);
	Vector3i max_pt_source = Vector3i(bounds.max_x - 1, bounds.max_y - 1, bounds.max_z - 1);

	if (!EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::IsPointInBounds(source_volume, min_pt_source) ||
	    !EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::IsPointInBounds(source_volume, max_pt_source)) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Specified source volume is at least partially out of bounds of the source scene.");
	}

	Vector6i bounds_destination;

	bounds_destination.min_x = bounds.min_x + offset.x;
	bounds_destination.max_x = bounds.max_x + offset.x;
	bounds_destination.min_y = bounds.min_y + offset.y;
	bounds_destination.max_y = bounds.max_y + offset.y;
	bounds_destination.min_z = bounds.min_z + offset.z;
	bounds_destination.max_z = bounds.max_z + offset.z;

	Vector3i min_pt_destination = Vector3i(bounds_destination.min_x, bounds_destination.min_y,
	                                       bounds_destination.min_z);
	Vector3i max_pt_destination = Vector3i(bounds_destination.max_x - 1, bounds_destination.max_y - 1,
	                                       bounds_destination.max_z - 1);

	if (!EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::IsPointInBounds(target_volume, min_pt_destination) ||
	    !EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::IsPointInBounds(target_volume, max_pt_destination)) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Targeted volume is at least partially out of bounds of the destination scene.");
	}
	TVoxel* sourceVoxels = source_volume->GetVoxels();
	TVoxel* destinationVoxels = target_volume->GetVoxels();
	if (offset == Vector3i(0)) {
		const PlainVoxelArray::IndexData* sourceIndexData = source_volume->index.GetIndexData();
		const PlainVoxelArray::IndexData* destinationIndexData = target_volume->index.GetIndexData();
		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.max_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					int vmIndex = 0;
					int linearSourceIndex = findVoxel(sourceIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					int linearDestinationIndex = findVoxel(destinationIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					memcpy(&destinationVoxels[linearDestinationIndex], &sourceVoxels[linearSourceIndex], sizeof(TVoxel));
				}
			}
		}
	} else {
		const PlainVoxelArray::IndexData* sourceIndexData = source_volume->index.GetIndexData();
		const PlainVoxelArray::IndexData* destinationIndexData = target_volume->index.GetIndexData();
		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.max_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					int vmIndex = 0;
					int destination_z = source_z + offset.z, destination_y = source_y + offset.y, destination_x =
							source_x + offset.x;
					int linearSourceIndex = findVoxel(sourceIndexData, Vector3i(source_x, source_y, source_z), vmIndex);
					int linearDestinationIndex = findVoxel(destinationIndexData,
					                                       Vector3i(destination_x, destination_y, destination_z),
					                                       vmIndex);
					memcpy(&destinationVoxels[linearDestinationIndex], &sourceVoxels[linearSourceIndex],
					       sizeof(TVoxel));
				}
			}
		}
	}
	return true;
}

template<typename TVoxel>
inline static Vector6i GetSceneBounds(VoxelVolume<TVoxel, PlainVoxelArray>* source) {
	PlainVoxelArray::IndexData* indexData = source->index.GetIndexData();
	return {indexData->offset.x, indexData->offset.y, indexData->offset.z,
	        indexData->offset.x + indexData->size.x,
	        indexData->offset.y + indexData->size.y,
	        indexData->offset.z + indexData->size.z};
}


template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::CopyVolume(
		VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume,
		VoxelVolume<TVoxel, PlainVoxelArray>* sourceVolume,
		const Vector3i& offset) {
	EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::ResetVolume(targetVolume);
	//TODO: this bounds treatment isn't quite correct -- it assumes same bounds for source & dest. Need to fix.
	Vector6i bounds = GetSceneBounds(sourceVolume);
	if (offset.x > 0) {
		bounds.max_x -= offset.x;
	} else {
		bounds.min_x -= offset.x;
	}
	if (offset.y > 0) {
		bounds.max_y -= offset.y;
	} else {
		bounds.min_y -= offset.y;
	}
	if (offset.z > 0) {
		bounds.max_z -= offset.z;
	} else {
		bounds.min_z -= offset.z;
	}
	return EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray>::CopyVolumeSlice(targetVolume, sourceVolume, bounds,
	                                                                       offset);
}

//endregion ============================================================================================================