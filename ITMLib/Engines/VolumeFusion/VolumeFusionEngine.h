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

//local
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../Common/Configurable.h"
#include "VolumeFusionSettings.h"

namespace ITMLib {

template<typename TVoxel, typename TIndex>
class VolumeFusionEngineInterface : public Configurable<VolumeFusionSettings> {
protected:
	using Configurable<VolumeFusionSettings>::parameters;
public:
	using Configurable<VolumeFusionSettings>::Configurable;
	using Configurable<VolumeFusionSettings>::GetParameters;

	virtual ~VolumeFusionEngineInterface() = default;
	/**
	 * \brief Fuses the live scene into the canonical scene
	 * \details Operation happens after the motion is tracked, at this point sourceTsdfVolume should be as Close to the canonical
	 * as possible
	 * \param target_volume the target volume: often, the canonical volume, representing state of the surfaces at the beginning of the sequence
	 * \param source_volume the source volume: often, the live volume, i.e. a TSDF generated from a single recent depth image
	 * \param liveSourceFieldIndex index of the sdf field to use at live scene voxels
	 */
	virtual void FuseOneTsdfVolumeIntoAnother(VoxelVolume <TVoxel, TIndex>* target_volume,
	                                          VoxelVolume <TVoxel, TIndex>* source_volume,
	                                          unsigned short timestamp) = 0;
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeFusionEngine :
		public VolumeFusionEngineInterface<TVoxel, TIndex> {
private:
	// Negated "mu" from KinectFusion, negative "eta" from Killing- & SobolevFusion,
	// expressed in sdf distance units, i.e.:
	// negative_surface_thickness_sdf_scale = -surface_thickness (meters) / [truncation distance (voxels) * voxel size (meters)]
	float negative_surface_thickness_sdf_scale;
public:
	VolumeFusionEngine();
	void FuseOneTsdfVolumeIntoAnother(VoxelVolume <TVoxel, TIndex>* target_volume,
	                                  VoxelVolume <TVoxel, TIndex>* source_volume,
	                                  unsigned short timestamp) override;
};

} // namespace ITMLib
