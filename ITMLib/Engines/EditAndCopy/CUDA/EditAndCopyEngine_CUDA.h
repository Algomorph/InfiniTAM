//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../Interface/EditAndCopyEngineInterface.h"
#include "../../../Utils/HashBlockProperties.h"
#include "../../../GlobalTemplateDefines.h"


namespace ITMLib {

template<typename TVoxel, typename TIndex>
class EditAndCopyEngine_CUDA : public EditAndCopyEngineInterface<TVoxel, TIndex> {
};


template<typename TVoxel>
class EditAndCopyEngine_CUDA<TVoxel, PlainVoxelArray> :
		public EditAndCopyEngineInterface<TVoxel, PlainVoxelArray> {
private:
	void* readVoxelResult_device;
	void* readVoxelResult_host;

public:
	EditAndCopyEngine_CUDA();
	~EditAndCopyEngine_CUDA();

	//can be used as a singleton, but doesn't HAVE TO be
	static EditAndCopyEngine_CUDA& Inst() {
		static EditAndCopyEngine_CUDA<TVoxel, PlainVoxelArray> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume) override;
	bool SetVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at);
	TVoxel ReadVoxel(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3i at,
	                 PlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const Vector3i& at);
	void OffsetWarps(VoxelVolume<TVoxel, PlainVoxelArray>* volume, Vector3f offset) override;
	bool CopyVolumeSlice(
			VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume,
			VoxelVolume<TVoxel, PlainVoxelArray>* sourceVolume,
			Vector6i bounds, const Vector3i& offset) override;
	bool CopyVolume(VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume,
	                VoxelVolume<TVoxel, PlainVoxelArray>* sourceVolume,
	                const Vector3i& offset = Vector3i(0)) override;
};


template<typename TVoxel>
class EditAndCopyEngine_CUDA<TVoxel, VoxelBlockHash> :
		public EditAndCopyEngineInterface<TVoxel, VoxelBlockHash> {
private:
	void* allocationTempData_device;
	void* allocationTempData_host;

	void* readVoxelResult_device;
	void* readVoxelResult_host;

public:
	EditAndCopyEngine_CUDA();
	~EditAndCopyEngine_CUDA();

	//can be used as a singleton, but doesn't HAVE TO be
	static EditAndCopyEngine_CUDA& Inst() {
		static EditAndCopyEngine_CUDA<TVoxel, VoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;
	bool SetVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at) override;
	TVoxel
	ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at,
	          VoxelBlockHash::IndexCache& cache) override;
	TVoxel
	ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3i at, int& where,
	          VoxelBlockHash::IndexCache& cache);

	void OffsetWarps(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3f offset) override;
	bool CopyVolumeSlice(VoxelVolume<TVoxel, VoxelBlockHash>* target_volume,
	                     VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
	                     Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;

	bool CopyVolume(VoxelVolume<TVoxel, VoxelBlockHash>* target_volume,
	                VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
	                const Vector3i& offset = Vector3i(0)) override;

};

typedef EditAndCopyEngine_CUDA<TSDFVoxel, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Voxel;
typedef EditAndCopyEngine_CUDA<TSDFVoxel, VoxelBlockHash> ManipulationEngine_CUDA_VBH_Voxel;
typedef EditAndCopyEngine_CUDA<WarpVoxel, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Warp;
typedef EditAndCopyEngine_CUDA<WarpVoxel, VoxelBlockHash> ManipulationEngine_CUDA_VBH_Warp;


}//namespace ITMLib