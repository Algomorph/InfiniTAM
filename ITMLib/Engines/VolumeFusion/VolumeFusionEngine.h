//  ================================================================
//  Created by Gregory Kramida on 1/30/20.
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

//#include

#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../Objects/Volume/VoxelVolume.h"

namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class VolumeFusionEngineInterface {
public:
	/**
	 * \brief Fuses the live scene into the canonical scene
	 * \details Operation happens after the motion is tracked, at this point sourceTsdfVolume should be as close to the canonical
	 * as possible
	 * \param targetTsdfVolume the canonical voxel grid, representing the state at the beginning of the sequence
	 * \param sourceTsdfVolume the live voxel grid, a TSDF generated from a single recent depth image
	 * \param liveSourceFieldIndex index of the sdf field to use at live scene voxels
	 */
	virtual void FuseOneTsdfVolumeIntoAnother(VoxelVolume <TVoxel, TIndex>* targetVolume,
	                                          VoxelVolume <TVoxel, TIndex>* sourceVolume) = 0;
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeFusionEngine :
		public VolumeFusionEngineInterface<TVoxel, TWarp, TIndex> {
public:
	void FuseOneTsdfVolumeIntoAnother(VoxelVolume <TVoxel, TIndex>* targetVolume,
	                                  VoxelVolume <TVoxel, TIndex>* sourceVolume) override;
};

} // namespace ITMLib
