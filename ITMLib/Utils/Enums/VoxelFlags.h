//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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

#include <iostream>
#include "../../../ORUtils/PlatformIndependence.h"

namespace ITMLib {
/**
 * \brief basic semantic information about the voxel. Currently, cannot be used as a binary flag, i.e. flag values are mutually-exclusive
 * \details Due to lack of reflection in the current C++ standard, printing these has to be supported by magic strings. Please, change
 * these strings in @refitem VoxelFlagsAsCString when changing the category names.
 */
//TODO: rename to VoxelCategory --Greg(GitHub: Algomorph)
enum VoxelFlags : unsigned char{
	VOXEL_UNKNOWN = 0,
	VOXEL_TRUNCATED = 1,
	VOXEL_NONTRUNCATED = 2,
	VOXEL_REGISTERED = 3
};

_CPU_AND_GPU_CODE_
inline
const char* VoxelFlagsAsCString(VoxelFlags flags){
	switch (flags){
		case VOXEL_UNKNOWN:
			return "VOXEL_UNKNOWN";
		case VOXEL_TRUNCATED:
			return "VOXEL_TRUNCATED";
		case VOXEL_NONTRUNCATED:
			return "VOXEL_NONTRUNCATED";
		default:
			printf("Unknown voxel flags value: %d.\n",flags);
			return ("UNRECOGNIZED VOXEL FLAG");
	}
}

}//namespace ITMLib
