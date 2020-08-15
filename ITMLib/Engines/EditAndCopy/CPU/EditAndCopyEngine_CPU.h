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
#pragma once

#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../GlobalTemplateDefines.h"
#include "../../../Utils/Enums/HashBlockProperties.h"
#include "../../../Utils/Logging/ConsolePrintColors.h"
#include "../Interface/EditAndCopyEngineInterface.h"


//TODO: Make CUDA versions -Greg (GitHub: Algomorph)

namespace ITMLib {

// region ==================== SCENE MANIPULATION ENGINE ===============================================================

template<typename TVoxel, typename TIndex>
class EditAndCopyEngine_CPU : public EditAndCopyEngineInterface<TVoxel, TIndex> {
};


template<typename TVoxel>
class EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>
		: public EditAndCopyEngineInterface<TVoxel, VoxelBlockHash> {
public:
	//can be used as a singleton, but doesn't HAVE TO be
	static EditAndCopyEngine_CPU& Inst() {
		static EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;
	bool SetVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at, TVoxel voxel) override;
	bool SetVoxelNoAllocation(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at, TVoxel voxel);
	TVoxel ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at) override;
	TVoxel ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at,
	                 VoxelBlockHash::IndexCache& cache) override;
	void OffsetWarps(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3f offset) override;
	bool
	CopyVolumeSlice(VoxelVolume<TVoxel, VoxelBlockHash>* target_volume,
	                VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
	                Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyVolume(VoxelVolume<TVoxel, VoxelBlockHash>* target_volume,
	                VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
	                const Vector3i& offset = Vector3i(0)) override;
	virtual ~EditAndCopyEngine_CPU() = default;
};

template<typename TVoxel>
class EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray> :
		public EditAndCopyEngineInterface<TVoxel, PlainVoxelArray> {
public:
	// can use this as singleton, but don't have to
	static EditAndCopyEngine_CPU& Inst() {
		static EditAndCopyEngine_CPU<TVoxel, PlainVoxelArray> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume) override;
	bool SetVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at) override;
	TVoxel ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at,
	                 PlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const Vector3i& at);
	void OffsetWarps(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3f offset) override;
	bool CopyVolumeSlice(VoxelVolume<TVoxel, PlainVoxelArray>* target_volume,
	                     VoxelVolume<TVoxel, PlainVoxelArray>* source_volume,
	                     Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyVolume(VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume,
	                VoxelVolume<TVoxel, PlainVoxelArray>* sourceVolume,
	                const Vector3i& offset = Vector3i(0)) override;

};

// endregion ================= SCENE MANIPULATION ENGINE ===============================================================

typedef EditAndCopyEngine_CPU<TSDFVoxel, PlainVoxelArray> ManipulationEngine_CPU_PVA_Voxel;
typedef EditAndCopyEngine_CPU<TSDFVoxel, VoxelBlockHash> ManipulationEngine_CPU_VBH_Voxel;
typedef EditAndCopyEngine_CPU<WarpVoxel, PlainVoxelArray> ManipulationEngine_CPU_PVA_Warp;
typedef EditAndCopyEngine_CPU<WarpVoxel, VoxelBlockHash> ManipulationEngine_CPU_VBH_Warp;

// region ======================================== HELPER RANGE COMPUTATION / CHECK ROUTINES ===========================
// =====================================================================================================================
inline
void BoundsFromExtrema(Vector6i& bounds, const Vector3i& extremum1, const Vector3i& extremum2) {
	// ** set min/max **
	if (extremum1.x > extremum2.x) {
		bounds.min_x = extremum2.x;
		bounds.max_x = extremum1.x;
	} else {
		bounds.min_x = extremum1.x;
		bounds.max_x = extremum2.x;
	}
	if (extremum1.y > extremum2.y) {
		bounds.min_y = extremum2.y;
		bounds.max_y = extremum1.y;
	} else {
		bounds.min_y = extremum1.y;
		bounds.max_y = extremum2.y;
	}
	if (extremum1.z > extremum2.z) {
		bounds.min_z = extremum2.z;
		bounds.max_z = extremum1.z;
	} else {
		bounds.min_z = extremum1.z;
		bounds.max_z = extremum2.z;
	}
}

inline
void
ComputeCopyRanges(int& xRangeStart, int& xRangeEnd, int& yRangeStart, int& yRangeEnd, int& zRangeStart, int& zRangeEnd,
                  const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	zRangeStart = ORUTILS_MAX(0, bounds.min_z - hashBlockPositionVoxels.z);
	zRangeEnd = ORUTILS_MIN(VOXEL_BLOCK_SIZE, bounds.max_z - hashBlockPositionVoxels.z + 1);
	yRangeStart = ORUTILS_MAX(0, bounds.min_y - hashBlockPositionVoxels.y);
	yRangeEnd = ORUTILS_MIN(VOXEL_BLOCK_SIZE, bounds.max_y - hashBlockPositionVoxels.y + 1);
	xRangeStart = ORUTILS_MAX(0, bounds.min_x - hashBlockPositionVoxels.x);
	xRangeEnd = ORUTILS_MIN(VOXEL_BLOCK_SIZE, bounds.max_x - hashBlockPositionVoxels.x + 1);
}

// endregion ===========================================================================================================

}//namespace ITMLib