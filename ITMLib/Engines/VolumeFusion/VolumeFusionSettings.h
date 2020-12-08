//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/8/20.
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
#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"

namespace ITMLib {

#define VOLUME_FUSION_SETTINGS_STRUCT_DESCRIPTION VolumeFusionSettings, "volume_fusion_settings", \
    (bool, use_surface_thickness_cutoff, true, PRIMITIVE, "When set to false, surface_thickness parameter is ignored."), \
    (float, surface_thickness, 0.012, PRIMITIVE, "Estimated surface thickness (in meters). "\
    "The value is used to compute parameter eta from 'SDF-2-SDF Registration for Real-Time 3D Reconstruction from RGB-D Data " \
	"Sec. 3.2, and SobolevFusion, Sec. 3.1, as well as 'mu' from original KinectFusion(2011). " \
	"The parameter value is computed as: surface_thickness (meters) / (truncation distance (voxels) * voxel size (meters))" \
	"Note that this parameter is only active during volume fusion, not depth-to-TSDF fusion. "\
	"There is a separate parameter for depth fusion.")


DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(VOLUME_FUSION_SETTINGS_STRUCT_DESCRIPTION);


} // namespace ITMLib
