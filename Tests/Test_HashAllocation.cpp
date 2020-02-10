//  ================================================================
//  Created by Gregory Kramida on 2/9/20.
//  Copyright (c)  2020 Gregory Kramida
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
#define BOOST_TEST_MODULE HashAllocation
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>
#include <random>

//ITMLib
#include "../ITMLib/Utils/FileIO/CSV_Utilities.h"
#include "../ITMLib/Utils/Math.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/Interface/VolumeStatisticsCalculatorInterface.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CPU/VolumeStatisticsCalculator_CPU.h"

using namespace ITMLib;


template<typename T>
static ORUtils::MemoryBlock<T> std_vector_to_ORUtils_MemoryBlock(std::vector<T> vector, MemoryDeviceType memoryDeviceType){
	ORUtils::MemoryBlock<T> block(vector.size(), memoryDeviceType == MEMORYDEVICE_CPU, memoryDeviceType == MEMORYDEVICE_CUDA);
	switch (memoryDeviceType){
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(block.GetData(MEMORYDEVICE_CUDA), vector.data(), vector.size() * sizeof(T), cudaMemcpyHostToDevice));
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
			break;
		case MEMORYDEVICE_CPU:
			memcpy(block.GetData(MEMORYDEVICE_CPU), vector.data(), vector.size() * sizeof(T));
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Device type not supported by operation.");
	}
	return block;
}


class CollisionHashFixture {
public:
	CollisionHashFixture() {
		std::ifstream file("TestData/collision_hash_coordinates.csv");

		for (CSVIterator row_iter(file); row_iter != CSVIterator(); ++row_iter) {
			if ((*row_iter).size() > 0) {
				int hash_code = std::stoi((*row_iter)[0]);
				if(position_by_hash.find(hash_code) == position_by_hash.end()){
					position_by_hash.insert({hash_code, std::vector<Vector3s>()});
				}
				std::vector<Vector3s>& position_for_hash_list = position_by_hash[hash_code];
				int position_count = (static_cast<int>((*row_iter).size()) - 1) / 3;
				for (int i_positon = 0; i_positon < position_count; i_positon++) {
					block_positions.emplace_back(
							std::stoi((*row_iter)[i_positon * 3 + 0]),
							std::stoi((*row_iter)[i_positon * 3 + 1]),
							std::stoi((*row_iter)[i_positon * 3 + 2])
					);
				}
			}
		}

		std::random_device rd;
		//seed = rd();
		//_DEBUG
		seed = 2684291675;
		std::mt19937 gen(seed);
		std::shuffle(block_positions.begin(), block_positions.end(), gen);

	}

	~CollisionHashFixture() {

	}

	std::vector<Vector3s> block_positions;
	std::unordered_map<int, std::vector<Vector3s>> position_by_hash;
	unsigned int seed;

};

BOOST_FIXTURE_TEST_CASE(TestAllocateHashBlockList_CPU, CollisionHashFixture) {
	VoxelVolume<TSDFVoxel,VoxelBlockHash> volume(MEMORYDEVICE_CPU, {0x80000, 0x20000});
	volume.Reset();
	IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>& indexer = IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::Instance();
	ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions, MEMORYDEVICE_CPU);
	indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.dataSize);
	int allocated_blocks = StatCalc_CPU_VBH_Voxel::Instance().ComputeAllocatedHashBlockCount(&volume);

	//vector<int>
	std::cout << allocated_blocks << std::endl;
	std::cout << block_positions.size() << std::endl;
	std::cout << seed << std::endl;

}