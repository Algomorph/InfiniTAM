//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../../Objects/Scene/PlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../Interface/EditAndCopyEngineInterface.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../ITMLibDefines.h"


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

	void ResetScene(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene) override ;
	bool SetVoxel(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene, Vector3i at, TVoxel voxel) override ;
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene, Vector3i at);
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene, Vector3i at,
	                 PlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene, const Vector3i& at);
	void OffsetWarps(ITMVoxelVolume <TVoxel, PlainVoxelArray>* scene, Vector3f offset) override;
	bool CopySceneSlice(
			ITMVoxelVolume <TVoxel, PlainVoxelArray>* destination,
			ITMVoxelVolume <TVoxel, PlainVoxelArray>* source,
			Vector6i bounds, const Vector3i& offset) override;
	bool CopyScene(ITMVoxelVolume <TVoxel, PlainVoxelArray>* destination,
	               ITMVoxelVolume <TVoxel, PlainVoxelArray>* source,
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
	void ResetScene(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene);
	bool SetVoxel(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene, Vector3i at) override;
	TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene, Vector3i at,
	          VoxelBlockHash::IndexCache& cache) override;
	TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene, Vector3i at, int& where,
	          VoxelBlockHash::IndexCache& cache);

	void OffsetWarps(ITMVoxelVolume <TVoxel, VoxelBlockHash>* scene, Vector3f offset) override;
	bool CopySceneSlice(ITMVoxelVolume <TVoxel, VoxelBlockHash>* destination, ITMVoxelVolume <TVoxel, VoxelBlockHash>* source,
	                    Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;

	bool CopyScene(ITMVoxelVolume <TVoxel, VoxelBlockHash>* destination,
	               ITMVoxelVolume <TVoxel, VoxelBlockHash>* source,
	               const Vector3i& offset = Vector3i(0)) override;

};

typedef EditAndCopyEngine_CUDA<ITMVoxel, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Voxel;
typedef EditAndCopyEngine_CUDA<ITMVoxel, VoxelBlockHash> ManipulationEngine_CUDA_VBH_Voxel;
typedef EditAndCopyEngine_CUDA<ITMWarp, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Warp;
typedef EditAndCopyEngine_CUDA<ITMWarp, VoxelBlockHash> ManipulationEngine_CUDA_VBH_Warp;


}//namespace ITMLib