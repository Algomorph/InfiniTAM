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
#include "../ITMLib/Engines/Analytics/AnalyticsEngineInterface.h"
#include "../ITMLib/Objects/Volume/RepresentationAccess.h"
#include "../ITMLib/Utils/Collections/MemoryBlock_StdContainer_Convertions.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

#endif

using namespace ITMLib;


class CollisionHashFixture {
public:
	CollisionHashFixture() {
		std::ifstream file("TestData/collision_hash_coordinates.csv");

		for (CSV_Iterator row_iter(file); row_iter != CSV_Iterator(); ++row_iter) {
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
		//_DEBUG alloc
		//seed = rd();
		seed = 2567557357;
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
	indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.size());

	int allocated_block_count = Analytics_CPU_VBH_Voxel::Instance().CountAllocatedHashBlocks(&volume);
	BOOST_REQUIRE_EQUAL(allocated_block_count, block_positions.size());

	std::vector<int> allocated_hashes = Analytics_CPU_VBH_Voxel::Instance().GetAllocatedHashCodes(&volume);
	std::unordered_set<int> allocated_hash_set(allocated_hashes.begin(), allocated_hashes.end());
	std::vector<Vector3s> allocated_block_positions = Analytics_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
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

BOOST_FIXTURE_TEST_CASE(TestDeallocateHashBlockList_CPU, CollisionHashFixture) {

	// for sorting later
	struct {
		bool operator()(Vector3s a, Vector3s b) const {
			return a.z == b.z ? (a.y == b.y ? (a.x < b.x) : a.y < b.y) : a.z < b.z;
		}
	} vector3s_coordinate_less;

	const int excess_list_size = 0x6FFFF;
	BOOST_TEST_CONTEXT("Seed: " << seed);
	BOOST_REQUIRE_GE(excess_list_size, required_max_excess_list_size);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CPU, {0x80000, excess_list_size});
	volume.Reset();
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions,
	                                                                                              MEMORYDEVICE_CPU);
	indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.size());

	std::vector<Vector3s> utilized_blocks = Analytics_CPU_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(&volume);

	std::sort(utilized_blocks.begin(), utilized_blocks.end(), vector3s_coordinate_less);

	const size_t blocks_to_remove_count = utilized_blocks.size() / 2;
	std::vector<Vector3s> blocks_to_remove_vector(utilized_blocks.begin(),
	                                              utilized_blocks.begin() + blocks_to_remove_count);
	std::vector<Vector3s> remaining_blocks_ground_truth(utilized_blocks.begin() + blocks_to_remove_count,
	                                                    utilized_blocks.end());

	ORUtils::MemoryBlock<Vector3s> blocks_to_remove_block = std_vector_to_ORUtils_MemoryBlock(blocks_to_remove_vector,
	                                                                                          MEMORYDEVICE_CPU);

	indexer.DeallocateBlockList(&volume, blocks_to_remove_block, blocks_to_remove_block.size());

	std::vector<Vector3s> remaining_utilized_blocks = Analytics_CPU_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(
			&volume);
	std::vector<Vector3s> remaining_allocated_blocks = Analytics_CPU_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&volume);


	std::sort(remaining_blocks_ground_truth.begin(), remaining_blocks_ground_truth.end(), vector3s_coordinate_less);
	std::sort(remaining_utilized_blocks.begin(), remaining_utilized_blocks.end(), vector3s_coordinate_less);
	std::sort(remaining_allocated_blocks.begin(), remaining_allocated_blocks.end(), vector3s_coordinate_less);

	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks.size(), remaining_allocated_blocks.size());
	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks.size(), remaining_blocks_ground_truth.size());

	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks, remaining_allocated_blocks);
	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks, remaining_blocks_ground_truth);

	const size_t blocks_to_re_add_count = blocks_to_remove_vector.size() / 2;

	std::vector<Vector3s> blocks_to_re_add(blocks_to_remove_vector.begin(),
	                                       blocks_to_remove_vector.begin() + blocks_to_re_add_count);
	std::vector<Vector3s> final_block_set_ground_truth(remaining_blocks_ground_truth.begin(),
	                                                   remaining_blocks_ground_truth.end());
	final_block_set_ground_truth.insert(final_block_set_ground_truth.end(), blocks_to_re_add.begin(),
	                                    blocks_to_re_add.end());


	ORUtils::MemoryBlock<Vector3s> blocks_to_re_add_block = std_vector_to_ORUtils_MemoryBlock(blocks_to_re_add,
	                                                                                          MEMORYDEVICE_CPU);

	indexer.AllocateBlockList(&volume, blocks_to_re_add_block, blocks_to_re_add_block.size());

	std::vector<Vector3s> final_block_set = Analytics_CPU_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(&volume);

	std::sort(final_block_set_ground_truth.begin(), final_block_set_ground_truth.end(), vector3s_coordinate_less);
	std::sort(final_block_set.begin(), final_block_set.end(), vector3s_coordinate_less);

	BOOST_REQUIRE_EQUAL(final_block_set.size(), final_block_set_ground_truth.size());
	BOOST_REQUIRE_EQUAL(final_block_set, final_block_set_ground_truth);
}


#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(TestAllocateHashBlockList_CUDA, CollisionHashFixture) {
	const int excess_list_size = 0x6FFFF;
	BOOST_TEST_CONTEXT("Seed: " << seed) {
		BOOST_REQUIRE_GE(excess_list_size, required_max_excess_list_size);
		VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CUDA, {0x80000, excess_list_size});
		volume.Reset();
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
		ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions,
		                                                                                              MEMORYDEVICE_CUDA);
		indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.size());

		int allocated_block_count = Analytics_CUDA_VBH_Voxel::Instance().CountAllocatedHashBlocks(&volume);
		BOOST_REQUIRE_EQUAL(allocated_block_count, block_positions.size());

		std::vector<int> allocated_hashes = Analytics_CUDA_VBH_Voxel::Instance().GetAllocatedHashCodes(&volume);
		std::unordered_set<int> allocated_hash_set(allocated_hashes.begin(), allocated_hashes.end());
		std::vector<Vector3s> allocated_block_positions = Analytics_CUDA_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
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

BOOST_FIXTURE_TEST_CASE(TestDeallocateHashBlockList_CUDA, CollisionHashFixture) {

	// for sorting later
	struct {
		bool operator()(Vector3s a, Vector3s b) const {
			return a.z == b.z ? (a.y == b.y ? (a.x < b.x) : a.y < b.y) : a.z < b.z;
		}
	} vector3s_coordinate_less;

	const int excess_list_size = 0x6FFFF;
	BOOST_TEST_CONTEXT("Seed: " << seed);
	BOOST_REQUIRE_GE(excess_list_size, required_max_excess_list_size);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CUDA, {0x80000, excess_list_size});
	volume.Reset();
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer = IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	ORUtils::MemoryBlock<Vector3s> hash_position_memory_block = std_vector_to_ORUtils_MemoryBlock(block_positions,
	                                                                                              MEMORYDEVICE_CUDA);
	indexer.AllocateBlockList(&volume, hash_position_memory_block, hash_position_memory_block.size());

	std::vector<Vector3s> utilized_blocks = Analytics_CUDA_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(&volume);

	std::sort(utilized_blocks.begin(), utilized_blocks.end(), vector3s_coordinate_less);

	const size_t blocks_to_remove_count = utilized_blocks.size() / 2;
	std::vector<Vector3s> blocks_to_remove_vector(utilized_blocks.begin(),
	                                              utilized_blocks.begin() + blocks_to_remove_count);
	std::vector<Vector3s> remaining_blocks_ground_truth(utilized_blocks.begin() + blocks_to_remove_count,
	                                                    utilized_blocks.end());

	ORUtils::MemoryBlock<Vector3s> blocks_to_remove_block = std_vector_to_ORUtils_MemoryBlock(blocks_to_remove_vector,
	                                                                                          MEMORYDEVICE_CUDA);

	indexer.DeallocateBlockList(&volume, blocks_to_remove_block, blocks_to_remove_block.size());

	std::vector<Vector3s> remaining_utilized_blocks = Analytics_CUDA_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(
			&volume);
	std::vector<Vector3s> remaining_allocated_blocks = Analytics_CUDA_VBH_Voxel::Instance().GetAllocatedHashBlockPositions(
			&volume);


	std::sort(remaining_blocks_ground_truth.begin(), remaining_blocks_ground_truth.end(), vector3s_coordinate_less);
	std::sort(remaining_utilized_blocks.begin(), remaining_utilized_blocks.end(), vector3s_coordinate_less);
	std::sort(remaining_allocated_blocks.begin(), remaining_allocated_blocks.end(), vector3s_coordinate_less);

	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks.size(), remaining_allocated_blocks.size());
	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks.size(), remaining_blocks_ground_truth.size());

	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks, remaining_allocated_blocks);
	BOOST_REQUIRE_EQUAL(remaining_utilized_blocks, remaining_blocks_ground_truth);

	const size_t blocks_to_re_add_count = blocks_to_remove_vector.size() / 2;

	std::vector<Vector3s> blocks_to_re_add(blocks_to_remove_vector.begin(),
	                                       blocks_to_remove_vector.begin() + blocks_to_re_add_count);
	std::vector<Vector3s> final_block_set_ground_truth(remaining_blocks_ground_truth.begin(),
	                                                   remaining_blocks_ground_truth.end());
	final_block_set_ground_truth.insert(final_block_set_ground_truth.end(), blocks_to_re_add.begin(),
	                                    blocks_to_re_add.end());


	ORUtils::MemoryBlock<Vector3s> blocks_to_re_add_block = std_vector_to_ORUtils_MemoryBlock(blocks_to_re_add,
	                                                                                          MEMORYDEVICE_CUDA);

	indexer.AllocateBlockList(&volume, blocks_to_re_add_block, blocks_to_re_add_block.size());

	std::vector<Vector3s> final_block_set = Analytics_CUDA_VBH_Voxel::Instance().GetUtilizedHashBlockPositions(&volume);

	std::sort(final_block_set_ground_truth.begin(), final_block_set_ground_truth.end(), vector3s_coordinate_less);
	std::sort(final_block_set.begin(), final_block_set.end(), vector3s_coordinate_less);

	BOOST_REQUIRE_EQUAL(final_block_set.size(), final_block_set_ground_truth.size());
	BOOST_REQUIRE_EQUAL(final_block_set, final_block_set_ground_truth);
}

#endif