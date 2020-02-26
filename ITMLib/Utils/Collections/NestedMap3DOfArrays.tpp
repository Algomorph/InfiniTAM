//  ================================================================
//  Created by Gregory Kramida on 2/2/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>

#include "NestedMap3DOfArrays.h"

using namespace ITMLib;

template<typename T>
bool NestedMap3DOfArrays<T>::InsertOrdered(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0) {

	if (this->internalMap.find(keyLevel3) == (this->internalMap).end()) {
		internalMap[keyLevel3] = std::map<int, std::map<int, std::vector<T>>>();
	}
	auto& valueLevel3 = internalMap[keyLevel3];
	if (valueLevel3.find(keyLevel2) == valueLevel3.end()) {
		valueLevel3[keyLevel2] = std::map<int, std::vector<T>>();
	}
	auto& valueLevel2 = valueLevel3[keyLevel2];
	if (valueLevel2.find(keyLevel1) == valueLevel2.end()) {
		valueLevel2[keyLevel1] = std::vector<T>();
	}
	std::vector<T>& valueLevel1 = valueLevel2[keyLevel1];
	if (valueLevel1.empty() ||
	    valueLevel1[valueLevel1.size() - 1] != valueLevel0) {
		valueLevel1.push_back(valueLevel0);
		return true;
	}
	return false;
}

template<typename T>
NestedMap3DOfArrays<T>::NestedMap3DOfArrays(const char* prefixLevel3, const char* prefixLevel2,
                                            const char* prefixLevel1,
                                            const char* prefixLevel0) :
		internalMap(),
		prefixLevel3(prefixLevel3),
		prefixLevel2(prefixLevel2),
		prefixLevel1(prefixLevel1),
		prefixLevel0(prefixLevel0) {}

template<typename T>
bool NestedMap3DOfArrays<T>::SaveToFile(std::string path) {
	std::ofstream file = std::ofstream(path, std::ios::binary | std::ios::out);
	if (!file) {
		std::cerr
				<< ("Could not open " + std::string(path) + " for writing. "  __FILE__ ": " + std::to_string(__LINE__))
				<< std::endl;
		return false;
	}

	size_t sizeLevel3 = internalMap.size();
	file.write(reinterpret_cast<const char* >(&sizeLevel3), sizeof(size_t));
	for (auto elementLevel3 : internalMap) {
		file.write(reinterpret_cast<const char* >(&elementLevel3.first), sizeof(int));
		size_t sizeLevel2 = elementLevel3.second.size();
		file.write(reinterpret_cast<const char* >(&sizeLevel2), sizeof(size_t));
		for (auto elementLevel2 : elementLevel3.second) {
			file.write(reinterpret_cast<const char* >(&elementLevel2.first), sizeof(int));
			size_t sizeLevel1 = elementLevel2.second.size();
			file.write(reinterpret_cast<const char* >(&sizeLevel1), sizeof(size_t));
			for (auto elementLevel1 : elementLevel2.second) {
				file.write(reinterpret_cast<const char* >(&elementLevel1.first), sizeof(int));
				size_t sizeLevel0 = elementLevel1.second.size();
				file.write(reinterpret_cast<const char* >(&sizeLevel0), sizeof(size_t));
				for (auto elementLevel0 : elementLevel1.second) {
					file.write(reinterpret_cast<const char* >(&elementLevel0), sizeof(T));
				}
			}
		}
	}
	file.close();
	return true;
}

template<typename T>
bool NestedMap3DOfArrays<T>::LoadFromFile(std::string path) {
	internalMap.clear();
	std::ifstream file = std::ifstream(path, std::ios::binary | std::ios::in);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for reading. " __FILE__ ": " + std::to_string(__LINE__))
		          << std::endl;
		return false;
	}
	size_t level3ElementCount, level2ElementCount, level1ElementCount, level0ElementCount;
	int level3Element, level2Element, level1Element;
	T level0Element;
	file.read(reinterpret_cast<char*>(&level3ElementCount), sizeof(size_t));
	for (int iLevel3Element = 0; iLevel3Element < level3ElementCount; iLevel3Element++) {
		file.read(reinterpret_cast<char*>(&level3Element), sizeof(int));
		internalMap[level3Element] = std::map<int, std::map<int, std::vector<T>>>();
		file.read(reinterpret_cast<char*>(&level2ElementCount), sizeof(size_t));
		for (int iLevel2Element = 0; iLevel2Element < level2ElementCount; iLevel2Element++) {
			file.read(reinterpret_cast<char*>(&level2Element), sizeof(int));
			internalMap[level3Element][level2Element] = std::map<int, std::vector<T>>();
			file.read(reinterpret_cast<char*>(&level1ElementCount), sizeof(size_t));
			for (int iLevel1Element = 0; iLevel1Element < level1ElementCount; iLevel1Element++) {
				file.read(reinterpret_cast<char*>(&level1Element), sizeof(int));
				internalMap[level3Element][level2Element][level1Element] = std::vector<T>();
				std::vector<T>& array = internalMap[level3Element][level2Element][level1Element];
				file.read(reinterpret_cast<char*>(&level0ElementCount), sizeof(size_t));
				for (int iLevel0Element = 0; iLevel0Element < level0ElementCount; iLevel0Element++) {
					file.read(reinterpret_cast<char*>(&level0Element), sizeof(T));
					array.push_back(level0Element);
				}
			}
		}
	}
	file.close();
	return true;
}


namespace ITMLib {
template<typename T>
std::ostream& operator<<(std::ostream& stream, const NestedMap3DOfArrays<T>& nestedMap3D) {
	for (auto elementLevel3 : nestedMap3D.internalMap) {
		stream << nestedMap3D.prefixLevel3 << ": " << elementLevel3.first << std::endl;
		for (auto elementLevel2 : elementLevel3.second) {
			stream << "  " << nestedMap3D.prefixLevel2 << ": " << elementLevel2.first << std::endl;
			for (auto elementLevel1 : elementLevel2.second) {
				stream << "    " << nestedMap3D.prefixLevel1 << ": " << elementLevel1.first << std::endl;
				for (auto elementLevel0 : elementLevel1.second) {
					stream << "      " << nestedMap3D.prefixLevel0 << ": " << elementLevel0 << std::endl;
				}
			}
		}
	}
	return stream;
}
}//namespace ITMLib

template<typename T>
bool NestedMap3DOfArrays<T>::operator==(const NestedMap3DOfArrays<T>& other) const {
	if (internalMap.size() != other.internalMap.size()) {
		return false;
	}
	auto itThisLevel3 = internalMap.begin();
	auto itOtherLevel3 = other.internalMap.begin();

	while (itThisLevel3 != internalMap.end()) {
		if (itThisLevel3->first != itOtherLevel3->first ||
		    itThisLevel3->second.size() != itOtherLevel3->second.size()) {
			return false;
		}
		auto itThisLevel2 = itThisLevel3->second.begin();
		auto itOtherLevel2 = itOtherLevel3->second.begin();
		while (itThisLevel2 != itThisLevel3->second.end()) {
			if (itThisLevel2->first != itOtherLevel2->first ||
			    itThisLevel2->second.size() != itOtherLevel2->second.size()) {
				return false;
			}
			auto itThisLevel1 = itThisLevel2->second.begin();
			auto itOtherLevel1 = itOtherLevel2->second.begin();
			while (itThisLevel1 != itThisLevel2->second.end()) {
				if (itThisLevel1->first != itOtherLevel1->first ||
				    itThisLevel1->second.size() != itOtherLevel1->second.size()) {
					return false;
				}
				auto itThisLevel0 = itThisLevel1->second.begin();
				auto itOtherLevel0 = itOtherLevel1->second.begin();
				while (itThisLevel0 != itThisLevel1->second.end()) {
					if (*itThisLevel0 != *itOtherLevel0) {
						return false;
					}
					itThisLevel0++;
					itOtherLevel0++;
				}
				itThisLevel1++;
				itOtherLevel1++;
			}
			itThisLevel2++;
			itOtherLevel2++;
		}
		itThisLevel3++;
		itOtherLevel3++;
	}
	return true;
}

template<typename T>
std::vector<int> NestedMap3DOfArrays<T>::GetLevel3Keys() const {
	std::vector<int> keys;
	for (auto& iter : internalMap) {
		keys.push_back(iter.first);
	}
	return keys;
}

template<typename T>
NestedMap3DOfArrays<T> NestedMap3DOfArrays<T>::FilterBasedOnLevel0Lengths(int minThreshold) {
	NestedMap3DOfArrays filtered(prefixLevel3, prefixLevel2, prefixLevel1, prefixLevel0);
	for (auto elementLevel3 : internalMap) {
		for (auto elementLevel2 : elementLevel3.second) {
			for (auto elementLevel1 : elementLevel2.second) {
				if (elementLevel1.second.size() >= minThreshold) {
					for (auto value : elementLevel1.second) {
						filtered.InsertOrdered(elementLevel3.first, elementLevel2.first, elementLevel1.first, value);
					}
				}
			}
		}
	}
	return filtered;
}

template<typename T>
bool NestedMap3DOfArrays<T>::SaveToTextFile(std::string path) {
	std::ofstream file = std::ofstream(path, std::ios::out);
	if (!file) {
		std::cerr
				<< ("Could not open " + std::string(path) + " for writing. "  __FILE__ ": " + std::to_string(__LINE__))
				<< std::endl;
		return false;
	}
	file << *this << std::endl;
	file.close();
	return true;
}

template<typename T>
NestedMap3DOfArrays<T>::NestedMap3DOfArrays(const NestedMap3DOfArrays<T>& nestedMap3D) {
	for (auto elementLevel3 : nestedMap3D.internalMap) {
		for (auto elementLevel2 : elementLevel3.second) {
			for (auto elementLevel1 : elementLevel2.second) {
				for (T value : elementLevel1.second) {
					this->InsertOrdered(elementLevel3.first, elementLevel2.first, elementLevel1.first, value);
				}
			}
		}
	}
}

template<typename T>
NestedMap3DOfArrays<T>::~NestedMap3DOfArrays() {
}

template<typename T>
NestedMap3DOfArrays<T>& NestedMap3DOfArrays<T>::operator=(NestedMap3DOfArrays&& other) noexcept {
	if (this == &other) {
		return *this;
	}
	this->internalMap = other.internalMap;
	prefixLevel0 = other.prefixLevel0;
	prefixLevel1 = other.prefixLevel1;
	prefixLevel2 = other.prefixLevel2;
	prefixLevel3 = other.prefixLevel3;
}

template<typename T>
NestedMap3DOfArrays<T>::NestedMap3DOfArrays() :internalMap(),
                                               prefixLevel3("level3"),
                                               prefixLevel2("level2"),
                                               prefixLevel1("level1"),
                                               prefixLevel0("level0") {}

template<typename T>
NestedMap3DOfArrays<T>& NestedMap3DOfArrays<T>::operator=(NestedMap3DOfArrays& other) {
	NestedMap3DOfArrays tmp(other);
	*this = std::move(tmp);
	return *this;
}


template<typename T>
bool NestedMap3DOfArrays<T>::Contains(int keyLevel3, int keyLevel2, int keyLevel1) const {
	if (this->internalMap.find(keyLevel3) == (this->internalMap).end()) { return false; }
	auto& valueLevel3 = internalMap.at(keyLevel3);
	if (valueLevel3.find(keyLevel2) == valueLevel3.end()) { return false; }
	auto& valueLevel2 = valueLevel3.at(keyLevel2);
	return valueLevel2.find(keyLevel1) != valueLevel2.end();
}

template<typename T>
bool NestedMap3DOfArrays<T>::Contains(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0) const {
	if (this->internalMap.find(keyLevel3) == (this->internalMap).end()) { return false; }
	auto& valueLevel3 = internalMap.at(keyLevel3);
	auto iteratorLevel3 = valueLevel3.find(keyLevel2);
	if (iteratorLevel3 == valueLevel3.end()) { return false; }
	auto& valueLevel2 = (*iteratorLevel3).second;
	auto iteratorLevel2 = valueLevel2.find(keyLevel1);
	if (iteratorLevel2 == valueLevel2.end()) { return false; }
	const std::vector<T>& valueLevel1 = (*iteratorLevel2).second;
	return !(std::find(valueLevel1.begin(), valueLevel1.end(), valueLevel0) == valueLevel1.end());
}

template<typename T>
bool NestedMap3DOfArrays<T>::Contains(const int& keyLevel3, const int& keyLevel2) const {
	if (this->internalMap.find(keyLevel3) == (this->internalMap).end()) { return false; }
	auto& valueLevel3 = internalMap.at(keyLevel3);
	return !(valueLevel3.find(keyLevel2) == valueLevel3.end());
}

template<typename T>
const std::vector<T>* NestedMap3DOfArrays<T>::GetFirstArray() const {
	if (internalMap.empty()) return nullptr;
	return &(*(*(*this->internalMap.begin()).second.begin()).second.begin()).second;
}

template<typename T>
const std::vector<T>* NestedMap3DOfArrays<T>::GetLastArray() const {
	if (internalMap.empty()) return nullptr;
	return &(*(--(*(--(*(--this->internalMap.end())).second.end())).second.end())).second;
}

template<typename T>
const std::vector<T>* NestedMap3DOfArrays<T>::GetArrayAfter(int keyLevel3, int keyLevel2, int keyLevel1) const {
	auto iteratorLevel3 = this->internalMap.find(keyLevel3);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel3 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel3 = (*iteratorLevel3).second;
	auto iteratorLevel2 = valueLevel3.find(keyLevel2);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel2 == valueLevel3.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	if (iteratorLevel1 == valueLevel2.end()) { return nullptr; }
	//otherwise, we have to check if level 1 map has more elements and return the next one if it does
	iteratorLevel1++;
	if (iteratorLevel1 != valueLevel2.end()) { return &(*iteratorLevel1).second; }
	//there are no elements left in the current level 1 map, have to seek the next level 1 map in the level 2 map
	iteratorLevel2++;
	//if there are more elements in the current level 2 map, return the first element in the first one
	if (iteratorLevel2 != valueLevel3.end()) { return &(*(*iteratorLevel2).second.begin()).second; }
	// otherwise, we have to check the next level 2 map in the level 3 Map
	iteratorLevel3++;
	if (iteratorLevel3 != internalMap.end()) { return &(*(*(*iteratorLevel3).second.begin()).second.begin()).second; }
	// the provided keys had to point to the very last element of the map. Rewind to the first element.
	return GetFirstArray();
}

template<typename T>
const std::vector<T>* NestedMap3DOfArrays<T>::GetArrayBefore(int keyLevel3, int keyLevel2, int keyLevel1) const {
	auto iteratorLevel3 = this->internalMap.find(keyLevel3);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel3 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel3 = (*iteratorLevel3).second;
	auto iteratorLevel2 = valueLevel3.find(keyLevel2);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel2 == valueLevel3.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	if (iteratorLevel1 == valueLevel2.end()) { return nullptr; }
	//otherwise, we have to check if level 1 map has more elements and return the previous one if it does
	if (iteratorLevel1 != valueLevel2.begin()) { return &(*(--iteratorLevel1)).second; }
	//there are no elements left in the current level 1 map, have to seek the next level 1 map in the level 2 map
	//if there are more elements in the current level 2 map, return the last element in the last one
	if (iteratorLevel2 != valueLevel3.begin()) { return &(*(--(*(--iteratorLevel2)).second.end())).second; }
	// otherwise, we have to check the next level 2 map in the level 3 Map
	if (iteratorLevel3 !=
	    internalMap.begin()) { return &(*(--(*(--(*(--iteratorLevel3)).second.end())).second.end())).second; }
	// the provided keys had to point to the very first element of the map. Fast-forward to the last element to loop around.
	return GetLastArray();
}

template<typename T>
const std::vector<T>* NestedMap3DOfArrays<T>::GetArrayAt(int keyLevel3, int keyLevel2, int keyLevel1) const {
	auto iteratorLevel3 = this->internalMap.find(keyLevel3);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel3 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel3 = (*iteratorLevel3).second;
	auto iteratorLevel2 = valueLevel3.find(keyLevel2);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel2 == valueLevel3.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	return iteratorLevel1 == valueLevel2.end() ? nullptr : &(*iteratorLevel1).second;
}

template<typename T>
std::vector<std::vector<T>> NestedMap3DOfArrays<T>::GetArrays() const {
	std::vector<std::vector<T>> out;
	for (auto elementLevel3 : internalMap) {
		for (auto elementLevel2 : elementLevel3.second) {
			for (auto elementLevel1 : elementLevel2.second) {
				out.push_back(elementLevel1.second);
			}
		}
	}
	return out;
}

template<typename T>
void NestedMap3DOfArrays<T>::Clear() {
	internalMap.clear();
}







