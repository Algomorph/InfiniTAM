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
#include "../Interface/VolumeEditAndCopyEngineInterface.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../ITMLibDefines.h"


namespace ITMLib {

template<typename TVoxel, typename TIndex>
class VolumeEditAndCopyEngine_CUDA : public VolumeEditAndCopyEngineInterface<TVoxel, TIndex> {
};


template<typename TVoxel>
class VolumeEditAndCopyEngine_CUDA<TVoxel, PlainVoxelArray> :
		public VolumeEditAndCopyEngineInterface<TVoxel, PlainVoxelArray> {
private:
	void* readVoxelResult_device;
	void* readVoxelResult_host;

public:
	VolumeEditAndCopyEngine_CUDA();
	~VolumeEditAndCopyEngine_CUDA();
	//can be used as a singleton, but doesn't HAVE TO be
	static VolumeEditAndCopyEngine_CUDA& Inst() {
		static VolumeEditAndCopyEngine_CUDA<TVoxel, PlainVoxelArray> instance; // Guaranteed to be destroyed.
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
class VolumeEditAndCopyEngine_CUDA<TVoxel, ITMVoxelBlockHash> :
		public VolumeEditAndCopyEngineInterface<TVoxel, ITMVoxelBlockHash> {
private:
	void* allocationTempData_device;
	void* allocationTempData_host;

	void* readVoxelResult_device;
	void* readVoxelResult_host;

public:
	VolumeEditAndCopyEngine_CUDA();
	~VolumeEditAndCopyEngine_CUDA();
	//can be used as a singleton, but doesn't HAVE TO be
	static VolumeEditAndCopyEngine_CUDA& Inst() {
		static VolumeEditAndCopyEngine_CUDA<TVoxel, ITMVoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}
	void ResetScene(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene);
	bool SetVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at) override;
	TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at,
	          ITMVoxelBlockHash::IndexCache& cache) override;
	TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, int& where,
	          ITMVoxelBlockHash::IndexCache& cache);

	void OffsetWarps(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) override;
	bool CopySceneSlice(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* source,
	                    Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;

	bool CopyScene(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* source,
	               const Vector3i& offset = Vector3i(0)) override;

};

typedef VolumeEditAndCopyEngine_CUDA<ITMVoxel, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Voxel;
typedef VolumeEditAndCopyEngine_CUDA<ITMVoxel, ITMVoxelBlockHash> ManipulationEngine_CUDA_VBH_Voxel;
typedef VolumeEditAndCopyEngine_CUDA<ITMWarp, PlainVoxelArray> ManipulationEngine_CUDA_PVA_Warp;
typedef VolumeEditAndCopyEngine_CUDA<ITMWarp, ITMVoxelBlockHash> ManipulationEngine_CUDA_VBH_Warp;


}//namespace ITMLib