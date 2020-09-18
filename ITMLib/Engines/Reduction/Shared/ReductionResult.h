//  ================================================================
//  Created by Gregory Kramida on 2/26/20.
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

#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"


namespace ITMLib {

template<typename TValue, typename TIndex>
struct ReductionResult;

template<typename TValue>
struct ReductionResult<TValue, VoxelBlockHash> {
	TValue value;
	unsigned int index_within_block;
	int hash_code;

	// ReductionResult() : value(0), index_within_block(0u), hash_code(0) {}
	//
	// ReductionResult(TValue value, unsigned int index_within_block, int hash_code) :
	// 		value(std::move(value)), index_within_block(index_within_block), hash_code(hash_code) {}

	friend inline std::ostream& operator<<(std::ostream& stream, const ReductionResult<TValue, VoxelBlockHash>& result) {
		stream << result.hash_code << " / " << result.index_within_block << ": " << result.value;
		return stream;
	}
};


template<typename TValue>
struct ReductionResult<TValue, PlainVoxelArray> {
	TValue value;
	unsigned int index_within_array;

	// ReductionResult() : value(0), index_within_array(0u) {}
	//
	// ReductionResult(TValue value, unsigned int index_within_array) :
	// 		value(std::move(value)), index_within_array(index_within_array) {}

	friend inline std::ostream& operator<<(std::ostream& stream, const ReductionResult<TValue, PlainVoxelArray>& result) {
		stream << result.index_within_array << ": " << result.value;
		return stream;
	}
};

template<typename TValue, typename TIndex>
struct ReductionResultInitializer;

template<typename TValue>
struct ReductionResultInitializer<TValue, VoxelBlockHash>{
	static ReductionResult<TValue, VoxelBlockHash> Default(){
		return {TValue(0), 0u, 0};
	}
};

template<typename TValue>
struct ReductionResultInitializer<TValue, PlainVoxelArray>{
	static ReductionResult<TValue, PlainVoxelArray> Default(){
		return {TValue(0), 0u};
	}
};

}// namespace ITMLib