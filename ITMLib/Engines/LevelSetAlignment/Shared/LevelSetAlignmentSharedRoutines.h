//  ================================================================
//  Created by Gregory Kramida on 4/26/18.
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

#include "../../../Utils/Enums/VoxelFlags.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "WarpHessian.h"


namespace ITMLib {


#define TRACKING_CONDITION_LIVE_KNOWN
//#define TRACKING_CONDITION_LIVE_NONTRUNCATED



template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline
bool VoxelIsConsideredForAlignment(const TVoxel& voxel_canonical, const TVoxel& voxel_live) {
#if defined(TRACKING_CONDITION_LIVE_KNOWN)
	return voxel_live.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(TRACKING_CONDITION_LIVE_NONTRUNCATED)
	return voxel_live.flags == ITMLib::VOXEL_NONTRUNCATED;
#endif
};


} // namespace ITMLib