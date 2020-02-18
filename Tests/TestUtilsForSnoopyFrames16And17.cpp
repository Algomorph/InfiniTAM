//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
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
#include "TestUtilsForSnoopyFrames16And17.h"

template<>
std::string Frame16And17Fixture::partial_frame_16_path<PlainVoxelArray>(){
	return "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
}
template<>
std::string Frame16And17Fixture::partial_frame_16_path<VoxelBlockHash>(){
	return "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";
}
template<>
std::string Frame16And17Fixture::partial_frame_17_path<PlainVoxelArray>(bool expanded_allocation){
	return "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
}
template<>
std::string Frame16And17Fixture::partial_frame_17_path<VoxelBlockHash>(bool expanded_allocation){
	std::string path = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	return expanded_allocation ? path + "expanded_" : path;
}

template<>
PlainVoxelArray::InitializationParameters Frame16And17Fixture::InitParams<PlainVoxelArray>() {
	return {Vector3i(88, 96, 160), Vector3i(-72, -24, 160)};
}

template<>
VoxelBlockHash::InitializationParameters Frame16And17Fixture::InitParams<VoxelBlockHash>() {
	return {0x0800, 0x20000};
}
