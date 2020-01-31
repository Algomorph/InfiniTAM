// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Engines/MultiScene/VoxelMapGraphManager.h"
#include "../Scene/MultiSceneAccess.h"
#include "RenderState.h"

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class RenderStateMultiScene : public RenderState
	{
	private:
		MemoryDeviceType memoryType;

	public:
		typedef typename MultiIndex<TIndex>::IndexData MultiIndexData;
		typedef MultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, TIndex> MultiSceneManager;

#ifndef COMPILE_WITHOUT_CUDA
		MultiIndexData *indexData_device;
		MultiVoxelData *voxelData_device;
#endif
		MultiIndexData indexData_host;
		MultiVoxelData voxelData_host;

		float mu;
		float voxelSize;

		RenderStateMultiScene(const Vector2i &imgSize, float vf_min, float vf_max, MemoryDeviceType _memoryType)
			: RenderState(imgSize, vf_min, vf_max, _memoryType)
		{
			memoryType = _memoryType;

#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaMalloc((void**)&indexData_device, sizeof(MultiIndexData)));
				ORcudaSafeCall(cudaMalloc((void**)&voxelData_device, sizeof(MultiVoxelData)));
			}
#endif
		}

		~RenderStateMultiScene(void)
		{
#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaFree(indexData_device));
				ORcudaSafeCall(cudaFree(voxelData_device));
			}
#endif
		}

		void PrepareLocalMaps(const MultiSceneManager & sceneManager)
		{
			voxelSize = sceneManager.getLocalMap(0)->scene->sceneParams->voxel_size;
			mu = sceneManager.getLocalMap(0)->scene->sceneParams->narrow_band_half_width;

			int num = (int)sceneManager.numLocalMaps();
			if (num > MAX_NUM_LOCALMAPS) num = MAX_NUM_LOCALMAPS;
			indexData_host.numLocalMaps = num;
			for (int localMapId = 0; localMapId < num; ++localMapId) 
			{
				indexData_host.poses_vs[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetM();
				indexData_host.poses_vs[localMapId].m30 /= voxelSize;
				indexData_host.poses_vs[localMapId].m31 /= voxelSize;
				indexData_host.poses_vs[localMapId].m32 /= voxelSize;
				indexData_host.posesInv[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetInvM();
				indexData_host.index[localMapId] = sceneManager.getLocalMap(localMapId)->scene->index.GetIndexData();
				voxelData_host.voxels[localMapId] = sceneManager.getLocalMap(localMapId)->scene->localVBA.GetVoxelBlocks();
			}

#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaMemcpy(indexData_device, &(indexData_host), sizeof(MultiIndexData), cudaMemcpyHostToDevice));
				ORcudaSafeCall(cudaMemcpy(voxelData_device, &(voxelData_host), sizeof(MultiVoxelData), cudaMemcpyHostToDevice));
			}
#endif
		}
	};

}

