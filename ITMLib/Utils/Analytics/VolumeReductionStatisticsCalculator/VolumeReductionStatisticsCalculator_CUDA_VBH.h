//  ================================================================
//  Created by Gregory Kramida on 2/21/20.
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

//local
#include "VolumeReductionStatisticsCalculator.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../GlobalTemplateDefines.h"

namespace ITMLib{

template<typename TVoxel>
class VolumeReductionStatisticsCalculator<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> :
		public VolumeReductionStatisticsCalculatorInterface<TVoxel, VoxelBlockHash> {
public:
	static VolumeReductionStatisticsCalculator& Instance() {
		static VolumeReductionStatisticsCalculator instance;
		return instance;
	}

	VolumeReductionStatisticsCalculator(VolumeReductionStatisticsCalculator const&) = delete;
	void operator=(VolumeReductionStatisticsCalculator const&) = delete;

	void ComputeWarpUpdateMax(float& max, Vector3i& position, VoxelVolume<TVoxel,VoxelBlockHash>* volume) override;
private:
	VolumeReductionStatisticsCalculator() = default;
	~VolumeReductionStatisticsCalculator() = default;
};

extern template class VolumeReductionStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;

} // namespace ITMLib