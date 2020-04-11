//  ================================================================
//  Created by Gregory Kramida on 10/8/19.
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
#include "LocalVBA.h"

using namespace ITMLib;

template<typename TVoxel>
LocalVBA<TVoxel>::LocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize):
		memoryType(memoryType),
		allocatedSize(noBlocks * blockSize),
		voxelBlocks(new ORUtils::MemoryBlock<TVoxel>(noBlocks * blockSize, memoryType)),
		allocationList(new ORUtils::MemoryBlock<int>(noBlocks, memoryType)) {}

template<class TVoxel>
LocalVBA<TVoxel>::~LocalVBA() {
	delete voxelBlocks;
	delete allocationList;

}

template<class TVoxel>
LocalVBA<TVoxel>::LocalVBA(const LocalVBA& other, MemoryDeviceType memoryType):
		memoryType(memoryType),
		allocatedSize(other.allocatedSize),
		lastFreeBlockId(other.lastFreeBlockId),
		voxelBlocks(new ORUtils::MemoryBlock<TVoxel>(other.allocatedSize, memoryType)),
		allocationList(new ORUtils::MemoryBlock<int>(other.allocationList->size(), memoryType)) {
	this->SetFrom(other);
}

template<class TVoxel>
void LocalVBA<TVoxel>::SetFrom(const LocalVBA& other) {
	lastFreeBlockId = other.lastFreeBlockId;
	MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memoryType, other.memoryType);
	voxelBlocks->SetFrom(other.voxelBlocks, memoryCopyDirection);
	allocationList->SetFrom(other.allocationList, memoryCopyDirection);
}
