//  ================================================================
//  Created by Gregory Kramida on 10/1/19.
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

#include <vector>
#include "../../../Math.h"
#include "../../../../Objects/Volume/VoxelVolume.h"
#include "../Interface/SceneStatisticsCalculatorInterface.h"
#include "../../../../GlobalTemplateDefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CUDA> :
		public SceneStatisticsCalculatorInterface<TVoxel, TIndex> {
public:
	static ITMSceneStatisticsCalculator& Instance() {
		static ITMSceneStatisticsCalculator instance;
		return instance;
	}

	ITMSceneStatisticsCalculator(ITMSceneStatisticsCalculator const&) = delete;
	void operator=(ITMSceneStatisticsCalculator const&) = delete;

	Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) override;
	std::vector<int> GetFilledHashBlockIds(VoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, TIndex>* scene) override;

	int ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) override;
	unsigned int ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) override;
	unsigned int CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* scene, float value) override;
	double ComputeNonTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* scene) override;
	double ComputeTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* scene) override;
	double ComputeFramewiseWarpMin(VoxelVolume<TVoxel,TIndex>* scene) override;
	double ComputeFramewiseWarpMax(VoxelVolume<TVoxel,TIndex>* scene) override;
	double ComputeFramewiseWarpMean(VoxelVolume<TVoxel,TIndex>* scene) override;

	Extent3D FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* scene) override;

	float FindMaxGradient0LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
	float FindMaxGradient1LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
private:
	ITMSceneStatisticsCalculator() = default;
	~ITMSceneStatisticsCalculator() = default;
};


typedef ITMSceneStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> SceneStatCalc_CUDA_VBH_Voxel;
typedef ITMSceneStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> SceneStatCalc_CUDA_PVA_Voxel;
typedef ITMSceneStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> SceneStatCalc_CUDA_VBH_Warp;
typedef ITMSceneStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> SceneStatCalc_CUDA_PVA_Warp;

} // namespace ITMLib