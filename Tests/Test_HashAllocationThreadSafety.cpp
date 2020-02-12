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
#define BOOST_TEST_MODULE HashAllocationThreadSafety
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <unordered_set>

//boost
#include <boost/test/unit_test.hpp>
#include <random>

//ITMLib
#include "../ITMLib/Utils/FileIO/CSV_Utilities.h"
#include "../ITMLib/Utils/Math.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/Interface/VolumeStatisticsCalculatorInterface.h"
#include "../ITMLib/Objects/Volume/RepresentationAccess.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CPU/VolumeStatisticsCalculator_CPU.h"
//(CUDA)
#ifndef COMPILE_WITH_CUDA
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CUDA/VolumeStatisticsCalculator_CUDA.h"
#endif

using namespace ITMLib;


namespace std {
template<>
struct hash<Vector3s> {
	size_t operator()(const Vector3s& vector) const {
		return std::hash<size_t>()(static_cast<size_t>(vector.x) << 16u | static_cast<size_t>(vector.y) << 8u |
		                           static_cast<size_t>(vector.z));
	}
};
} // namespace std

template<typename T>
static ORUtils::MemoryBlock<T>
std_vector_to_ORUtils_MemoryBlock(std::vector<T> vector, MemoryDeviceType memoryDeviceType) {
	ORUtils::MemoryBlock<T> block(vector.size(), memoryDeviceType == MEMORYDEVICE_CPU,
	                              memoryDeviceType == MEMORYDEVICE_CUDA);
	switch (memoryDeviceType) {
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(block.GetData(MEMORYDEVICE_CUDA), vector.data(), vector.size() * sizeof(T),
			                          cudaMemcpyHostToDevice));
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
				if (position_by_hash.find(hash_code) == position_by_hash.end()) {
					position_by_hash.insert({hash_code, std::vector<Vector3s>()});
				}
				std::vector<Vector3s>& position_for_hash_list = position_by_hash[hash_code];
				int position_count = (static_cast<int>((*row_iter).size()) - 1) / 3;
				for (int i_positon = 0; i_positon < position_count; i_positon++) {
					Vector3s block_position(std::stoi((*row_iter)[(i_positon * 3 + 0) + 1]),
					                        std::stoi((*row_iter)[(i_positon * 3 + 1) + 1]),
					                        std::stoi((*row_iter)[(i_positon * 3 + 2) + 1]));
					if (hash_bucket_by_position.find(block_position) != hash_bucket_by_position.end()) {
						std::cerr << "Filtering duplicate position " << block_position
						          << ", currently listed under hash code " << hash_code
						          << ", while previously listed under hash code " <<
						          hash_bucket_by_position[block_position] << "." << std::endl;
						DIEWITHEXCEPTION_REPORTLOCATION("Duplicate position encountered in data, aborting.");
					} else {
						block_positions.push_back(block_position);
						hash_bucket_by_position.insert({block_position, hash_code});
					}
				}
				if (hash_bucket_codes.find(hash_code) != hash_bucket_codes.end()) {
					std::cerr << "Duplicate hash code, " << hash_code << ", encountered in data." << std::endl;
					DIEWITHEXCEPTION_REPORTLOCATION("Duplicate hash encountered in data, aborting.");
				} else {
					hash_bucket_codes.insert(hash_code);
					required_min_ordered_entry_count++;
				}
			}
		}

		std::random_device rd;
		seed = rd();
		std::mt19937 gen(seed);
		std::shuffle(block_positions.begin(), block_positions.end(), gen);

		required_max_excess_list_size = block_positions.size() - required_min_ordered_entry_count;

	}

	~CollisionHashFixture() {

	}

	std::vector<Vector3s> block_positions;
	std::unordered_map<int, std::vector<Vector3s>> position_by_hash;
	std::unordered_map<Vector3s, int> hash_bucket_by_position;
	std::unordered_set<int> hash_bucket_codes;
	int required_min_ordered_entry_count = 0;
	int required_max_excess_list_size = 0;
	unsigned int seed;

};


BOOST_FIXTURE_TEST_CASE(TestAllocateHashBlockList_CPU, CollisionHashFixture) {
	const int excess_list_size = 0x6FFFF;
	BOOST_TEST_CONTEXT("Seed: " << seed);
	BOOST_REQUIRE_GE(excess_list_size, required_max_excess_list_size);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CPU, {0x80000, excess_list_size});
	volume.Reset();
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions,
	                                                                                              MEMORYDEVICE_CPU);
	indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.dataSize);

	int allocated_block_count = StatCalc_CPU_VBH_Voxel::Instance().ComputeAllocatedHashBlockCount(&volume);
	BOOST_REQUIRE_EQUAL(allocated_block_count, block_positions.size());

	std::vector<int> allocated_hashes = StatCalc_CPU_VBH_Voxel::Instance().GetAllocatedHashCodes(&volume);
	std::unordered_set<int> allocated_hash_set(allocated_hashes.begin(), allocated_hashes.end());
	std::vector<Vector3s> allocated_block_positions = StatCalc_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&volume);
	std::unordered_set<Vector3s> allocated_block_position_set(allocated_block_positions.begin(),
	                                                          allocated_block_positions.end());

	bool position_not_allocated = false;
	Vector3s bad_position;
	for (auto position : block_positions) {
		if (allocated_block_position_set.find(position) == allocated_block_position_set.end()) {
			position_not_allocated = true;
			bad_position = position;
			break;
		}
	}
	BOOST_REQUIRE_MESSAGE(!position_not_allocated,
	                      "Position " << bad_position << " was not allocated in the spatial hash. Seed: " << seed);

	bool wrong_position_allocated = false;
	bool wrong_hash_bucket = false;
	int bad_hash_code = 0;
	for (auto allocated_position : allocated_block_positions) {
		if (hash_bucket_by_position.find(allocated_position) == hash_bucket_by_position.end()) {
			wrong_position_allocated = true;
			bad_position = allocated_position;
			break;
		}
		int hash_bucket_code = HashCodeFromBlockPosition(allocated_position);
		if (hash_bucket_by_position[allocated_position] != hash_bucket_code) {
			wrong_hash_bucket = true;
			bad_position = allocated_position;
			bad_hash_code = hash_bucket_code;
			break;
		}
	}
	BOOST_REQUIRE_MESSAGE(!wrong_position_allocated,
	                      "Position " << bad_position << " was allocated erroneously in the spatial hash. Seed: "
	                                  << seed);
	BOOST_REQUIRE_MESSAGE(!wrong_hash_bucket, "Hash bucket code for position " << bad_position << ", " << bad_hash_code
	                                                                           << ", somehow doesn't correspond to hash bucket code in the test data for the same position, i.e. "
	                                                                           << hash_bucket_by_position[bad_position]
	                                                                           << ". Seed: " << seed);
	bool unallocated_bucket_code = false;
	for (auto hash_bucket_code : hash_bucket_codes) {
		if (allocated_hash_set.find(hash_bucket_code) == allocated_hash_set.end()) {
			unallocated_bucket_code = true;
			bad_hash_code = hash_bucket_code;
			break;
		}
	}
	BOOST_REQUIRE_MESSAGE(!unallocated_bucket_code,
	                      "Bucket code " << bad_hash_code << " was not allocated in the spatial hash. Seed: " << seed);
}

#ifndef COMPILE_WITH_CUDA
BOOST_FIXTURE_TEST_CASE(TestAllocateHashBlockList_CUDA, CollisionHashFixture) {
	const int excess_list_size = 0x6FFFF;
	BOOST_TEST_CONTEXT("Seed: " << seed) {
		BOOST_REQUIRE_GE(excess_list_size, required_max_excess_list_size);
		VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CUDA, {0x80000, excess_list_size});
		volume.Reset();
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
		ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions,
		                                                                                              MEMORYDEVICE_CUDA);
		indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.dataSize);

		int allocated_block_count = StatCalc_CUDA_VBH_Voxel::Instance().ComputeAllocatedHashBlockCount(&volume);
		BOOST_REQUIRE_EQUAL(allocated_block_count, block_positions.size());

		std::vector<int> allocated_hashes = StatCalc_CUDA_VBH_Voxel::Instance().GetAllocatedHashCodes(&volume);
		std::unordered_set<int> allocated_hash_set(allocated_hashes.begin(), allocated_hashes.end());
		std::vector<Vector3s> allocated_block_positions = StatCalc_CUDA_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
				&volume);
		std::unordered_set<Vector3s> allocated_block_position_set(allocated_block_positions.begin(),
		                                                          allocated_block_positions.end());

		bool position_not_allocated = false;
		Vector3s bad_position;
		for (auto position : block_positions) {
			if (allocated_block_position_set.find(position) == allocated_block_position_set.end()) {
				position_not_allocated = true;
				bad_position = position;
				break;
			}
		}
		BOOST_REQUIRE_MESSAGE(!position_not_allocated,
		                      "Position " << bad_position << " was not allocated in the spatial hash.");

		bool wrong_position_allocated = false;
		bool wrong_hash_bucket = false;
		int bad_hash_code = 0;
		for (auto allocated_position : allocated_block_positions) {
			if (hash_bucket_by_position.find(allocated_position) == hash_bucket_by_position.end()) {
				wrong_position_allocated = true;
				bad_position = allocated_position;
				break;
			}
			int hash_bucket_code = HashCodeFromBlockPosition(allocated_position);
			if (hash_bucket_by_position[allocated_position] != hash_bucket_code) {
				wrong_hash_bucket = true;
				bad_position = allocated_position;
				bad_hash_code = hash_bucket_code;
				break;
			}
		}
		BOOST_REQUIRE_MESSAGE(!wrong_position_allocated,
		                      "Position " << bad_position << " was allocated erroneously in the spatial hash.");
		BOOST_REQUIRE_MESSAGE(!wrong_hash_bucket,
		                      "Hash bucket code for position " << bad_position << ", " << bad_hash_code
		                                                       << ", somehow doesn't correspond to hash bucket code in the test data for the same position, i.e. "
		                                                       << hash_bucket_by_position[bad_position]);
		bool unallocated_bucket_code = false;
		for (auto hash_bucket_code : hash_bucket_codes) {
			if (allocated_hash_set.find(hash_bucket_code) == allocated_hash_set.end()) {
				unallocated_bucket_code = true;
				bad_hash_code = hash_bucket_code;
				break;
			}
		}
		BOOST_REQUIRE_MESSAGE(!unallocated_bucket_code,
		                      "Bucket code " << bad_hash_code << " was not allocated in the spatial hash.");
	}
}

#endif