//  ================================================================
//  Created by Gregory Kramida on 10/17/19.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/MathUtils.h"
//TODO: separate out common geometry object, i.e. PVA info is the logical equivalent of a box, make a separate Box geometry class
// (need to eliminate as much interdependence between classes in folders as possible, i.e. reduce coupling & increase cohesion)
#include "../../Objects/Volume/PlainVoxelArray.h"
#include "../Math.h"

_CPU_AND_GPU_CODE_
inline
Vector6i PVA_InfoToExtent(const ITMLib::GridAlignedBox& info) {
	return {info.offset.x, info.offset.y, info.offset.z, info.offset.x + info.size.x, info.offset.y + info.size.y,
	        info.offset.z + info.size.z};
}