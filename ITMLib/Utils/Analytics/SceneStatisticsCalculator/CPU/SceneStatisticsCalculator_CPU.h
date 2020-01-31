//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
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

#include <vector>
#include "../../../Math.h"
#include "../../../../Objects/Volume/VoxelVolume.h"
#include "../Interface/SceneStatisticsCalculatorInterface.h"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU> :
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

	Vector6i FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* scene) override;

	float FindMaxGradient0LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
	float FindMaxGradient1LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
private:
	ITMSceneStatisticsCalculator() = default;
	~ITMSceneStatisticsCalculator() = default;
};

typedef ITMSceneStatisticsCalculator<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> SceneStatCalc_CPU_VBH_Voxel;
typedef ITMSceneStatisticsCalculator<ITMVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> SceneStatCalc_CPU_PVA_Voxel;
typedef ITMSceneStatisticsCalculator<ITMWarp, VoxelBlockHash, MEMORYDEVICE_CPU> SceneStatCalc_CPU_VBH_Warp;
typedef ITMSceneStatisticsCalculator<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU> SceneStatCalc_CPU_PVA_Warp;

}//end namespace ITMLib

