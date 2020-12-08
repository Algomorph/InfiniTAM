//  ================================================================
//  Created by Gregory Kramida on 5/8/19.
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

#include "VoxelVolume.tpp"
#include "../../Objects/Volume/VoxelTypes.h"

namespace ITMLib {
template
class VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>;
template
class VoxelVolume<TSDFVoxel_f_flags, VoxelBlockHash>;
template
class VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>;
template
class VoxelVolume<TSDFVoxel_f_rgb, VoxelBlockHash>;
template
class VoxelVolume<WarpVoxel, PlainVoxelArray>;
template
class VoxelVolume<WarpVoxel, VoxelBlockHash>;
}  // namespace ITMLib