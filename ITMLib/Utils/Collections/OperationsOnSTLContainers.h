//  ================================================================
//  Created by Gregory Kramida on 3/6/20.
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
//stdlib
#include <vector>
#include <set>
#include <unordered_set>

//ORUtils
#include "../../../ORUtils/IStreamWrapper.h"
#include "../../../ORUtils/OStreamWrapper.h"

namespace std {

template<typename Key, typename Compare, typename Allocator>
std::set<Key, Compare, Allocator>
operator-=(std::set<Key, Compare, Allocator>&& source,
           const std::set<Key, Compare, Allocator>& to_remove) {
	if (source.empty()) { return source; }
	// First narrow down the overlapping range:
	const auto rhsbeg = to_remove.lower_bound(*source.begin());
	const auto rhsend = to_remove.upper_bound(*source.rbegin());
	for (auto i = rhsbeg; i != rhsend; ++i) {
		source.erase(*i);
	}
	return std::move(source);
}

template<typename _Value, typename _Hash, typename _Pred, typename _Alloc>
std::unordered_set<_Value, _Hash, _Pred, _Alloc>
operator-(const std::unordered_set<_Value, _Hash, _Pred, _Alloc>& source,
          const std::unordered_set<_Value, _Hash, _Pred, _Alloc>& to_remove) {
	std::unordered_set<_Value, _Hash, _Pred, _Alloc> target(source);
	for (const auto& elem: to_remove){
		target.erase(elem);
	}
	return target;
}

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
	std::vector<T> values;
	for (T value = start; value < stop; value += step)
		values.push_back(value);
	return values;
}

} // namespace std

namespace ITMLib {

template<typename T>
std::vector<T> ReadStdVectorFromFile(ORUtils::IStreamWrapper& file){
	size_t element_count;
	file.IStream().read(reinterpret_cast<char*>(&element_count), sizeof(size_t));
	std::vector<T> vector(element_count);
	file.IStream().read(reinterpret_cast<char*>(vector.data()), element_count * sizeof(T));
	return vector;
}

template<typename T>
void WriteStdVectorToFile(ORUtils::OStreamWrapper& file, const std::vector<T> vector){
	size_t element_count = vector.size();
	file.OStream().write(reinterpret_cast<const char*>(&element_count), sizeof(size_t));
	file.OStream().write(reinterpret_cast<const char*>(vector.data()), element_count * sizeof(T));
}

} // namespace ITMLib