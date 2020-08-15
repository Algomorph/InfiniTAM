//  ================================================================
//  Created by Gregory Kramida on 2/2/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "NestedMap3D.h"
#include "../Logging/ConsolePrintColors.h"
#include "../Logging/PrettyPrinters.h"

using namespace ITMLib;

template<typename T>
bool NestedMap3D<T>::InsertOrdered(int keyLevel2, int keyLevel1, int keyLevel0, T valueLevel0) {

	if (this->internalMap.find(keyLevel2) == (this->internalMap).end()) {
		internalMap[keyLevel2] = std::map<int, std::map<int, T>>();
	}
	auto& valueLevel2 = internalMap[keyLevel2];
	if (valueLevel2.find(keyLevel1) == valueLevel2.end()) {
		valueLevel2[keyLevel1] = std::map<int, T>();
	}
	auto& valueLevel1 = valueLevel2[keyLevel1];
	valueLevel1[keyLevel0] = valueLevel0;
	return true;
}

template<typename T>
NestedMap3D<T>::NestedMap3D(const char* prefixLevel2, const char* prefixLevel1, const char* prefixLevel0) :
		internalMap(),
		prefixLevel2(prefixLevel2),
		prefixLevel1(prefixLevel1),
		prefixLevel0(prefixLevel0) {}

template<typename T>
bool NestedMap3D<T>::SaveToFile(const char* path) {
	std::ofstream file = std::ofstream(path, std::ios::binary | std::ios::out);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}

	size_t sizeLevel2 = internalMap.size();
	file.write(reinterpret_cast<const char* >(&sizeLevel2), sizeof(size_t));
	for (auto elementLevel2 : internalMap) {
		file.write(reinterpret_cast<const char* >(&elementLevel2.first), sizeof(int));
		size_t sizeLevel1 = elementLevel2.second.size();
		file.write(reinterpret_cast<const char* >(&sizeLevel1), sizeof(size_t));
		for (auto elementLevel1 : elementLevel2.second) {
			file.write(reinterpret_cast<const char* >(&elementLevel1.first), sizeof(int));
			size_t sizeLevel0 = elementLevel1.second.size();
			file.write(reinterpret_cast<const char* >(&sizeLevel0), sizeof(size_t));
			for (auto elementLevel0 : elementLevel1.second) {
				file.write(reinterpret_cast<const char* >(&elementLevel0.first), sizeof(int));
				file.write(reinterpret_cast<const char* >(&elementLevel0.second), sizeof(T));
			}
		}
	}
	file.close();
	return true;
}

template<typename T>
bool NestedMap3D<T>::LoadFromFile(const char* path) {
	internalMap.clear();
	std::ifstream file = std::ifstream(path, std::ios::binary | std::ios::in);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}
	size_t level2ElementCount, level1ElementCount, level0ElementCount;
	int level2Element, level1Element, level0Element;
	file.read(reinterpret_cast<char*>(&level2ElementCount), sizeof(size_t));
	for (int iLevel2Element = 0; iLevel2Element < level2ElementCount; iLevel2Element++) {
		file.read(reinterpret_cast<char*>(&level2Element), sizeof(int));
		internalMap[level2Element] = std::map<int, std::map<int, T>>();
		file.read(reinterpret_cast<char*>(&level1ElementCount), sizeof(size_t));
		for (int iLevel1Element = 0; iLevel1Element < level1ElementCount; iLevel1Element++) {
			file.read(reinterpret_cast<char*>(&level1Element), sizeof(int));
			internalMap[level2Element][level1Element] = std::map<int, T>();
			file.read(reinterpret_cast<char*>(&level0ElementCount), sizeof(size_t));
			for (int iLevel0Element = 0; iLevel0Element < level1ElementCount; iLevel0Element++) {
				file.read(reinterpret_cast<char*>(&level0Element), sizeof(int));
				internalMap[level2Element][level1Element][level0Element] = T();
				T& value = internalMap[level2Element][level1Element][level0Element];
				file.read(reinterpret_cast<char*>(&value), sizeof(T));
			}
		}
	}
	file.close();
	return true;
}


namespace ITMLib {
template<typename T>
std::ostream& operator<<(std::ostream& stream, const NestedMap3D<T>& nestedMap3D) {
	for (auto elementLevel2 : nestedMap3D.internalMap) {
		stream << nestedMap3D.prefixLevel2 << ": " << elementLevel2.first << std::endl;
		for (auto elementLevel1 : elementLevel2.second) {
			stream << "  " << nestedMap3D.prefixLevel1 << ": " << elementLevel1.first << std::endl;
			for (auto elementLevel0 : elementLevel1.second) {
				stream << "    " << nestedMap3D.prefixLevel0 << ": " << elementLevel0.first << std::endl;
				stream << "    " << "value: " << elementLevel0.second << std::endl;
			}
		}
	}
	return stream;
}
}//namespace ITMLib

template<typename T>
bool NestedMap3D<T>::operator==(const NestedMap3D<T>& other) const {
	if (internalMap.size() != other.internalMap.size()) {
		return false;
	}
	auto itThisLevel2 = internalMap.begin();
	auto itOtherLevel2 = other.internalMap.begin();

	while (itThisLevel2 != internalMap.end()) {
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
				if (itThisLevel0->first != itOtherLevel0->first || itThisLevel0->second != itOtherLevel0->second) {
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
	return true;
}

template<typename T>
std::vector<int> NestedMap3D<T>::GetLevel2Keys() {
	std::vector<int> keys;
	for (auto& iter : internalMap) {
		keys.push_back(iter.first);
	}
	return keys;
}


template<typename T>
bool NestedMap3D<T>::SaveToTextFile(const char* path) {
	std::ofstream file = std::ofstream(path, std::ios::out);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}
	file << *this << std::endl;
	file.close();
	return true;
}

template<typename T>
NestedMap3D<T>::NestedMap3D(const NestedMap3D<T>& nestedMap3D) {
	for (auto elementLevel2 : nestedMap3D.internalMap) {
		for (auto elementLevel1 : elementLevel2.second) {
			for (auto elementLevel0 : elementLevel1.second) {
				this->InsertOrdered(elementLevel2.first, elementLevel1.first, elementLevel0.first,
				                    elementLevel0.second);
			}
		}
	}
}

template<typename T>
NestedMap3D<T>::~NestedMap3D() {
}

template<typename T>
NestedMap3D<T>& NestedMap3D<T>::operator=(NestedMap3D&& other) noexcept {
	if (this == &other) {
		return *this;
	}
	this->internalMap = other.internalMap;
	prefixLevel0 = other.prefixLevel0;
	prefixLevel1 = other.prefixLevel1;
	prefixLevel2 = other.prefixLevel2;
	return *this;
}

template<typename T>
NestedMap3D<T>::NestedMap3D() :internalMap(),
                               prefixLevel2("level2"),
                               prefixLevel1("level1"),
                               prefixLevel0("level1") {}

template<typename T>
NestedMap3D<T>& NestedMap3D<T>::operator=(NestedMap3D& other) {
	NestedMap3D tmp(other);
	*this = std::move(tmp);
	return *this;
}


template<typename T>
bool NestedMap3D<T>::Contains(int keyLevel2, int keyLevel1, int keyLevel0) {
	if (this->internalMap.find(keyLevel2) == (this->internalMap).end()) { return false; }
	auto& valueLevel2 = internalMap[keyLevel2];
	if (valueLevel2.find(keyLevel1) == valueLevel2.end()) { return false; }
	auto& valueLevel1 = valueLevel2[keyLevel1];
	return valueLevel1.find(keyLevel0) != valueLevel1.end();
}

template<typename T>
bool NestedMap3D<T>::Contains(int keyLevel2, int keyLevel1, int keyLevel0, T valueLevel0) {
	if (this->internalMap.find(keyLevel2) == (this->internalMap).end()) { return false; }
	auto& valueLevel2 = internalMap[keyLevel2];
	auto iteratorLevel2 = valueLevel2.find(keyLevel1);
	if (iteratorLevel2 == valueLevel2.end()) { return false; }
	auto& valueLevel1 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel1.find(keyLevel0);
	if (iteratorLevel1 == valueLevel1.end()) { return false; }
	return (*iteratorLevel1).second == valueLevel0;

}

template<typename T>
bool NestedMap3D<T>::Contains(int keyLevel2, int keyLevel1) {
	if (this->internalMap.find(keyLevel2) == (this->internalMap).end()) { return false; }
	auto& valueLevel2 = internalMap[keyLevel2];
	return !(valueLevel2.find(keyLevel1) == valueLevel2.end());
}

template<typename T>
const T* NestedMap3D<T>::GetFirstValue() const {
	if (internalMap.empty()) return nullptr;
	return &(*(*(*this->internalMap.begin()).second.begin()).second.begin()).second;
}

template<typename T>
const T* NestedMap3D<T>::GetLastValue() const {
	if (internalMap.empty()) return nullptr;
	return &(*(--(*(--(*(--this->internalMap.end())).second.end())).second.end())).second;
}

template<typename T>
const T* NestedMap3D<T>::GetValueAfter(int keyLevel2, int keyLevel1, int keyLevel0) const {
	auto iteratorLevel2 = this->internalMap.find(keyLevel2);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel2 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel1 == valueLevel2.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel1 = (*iteratorLevel1).second;
	auto iteratorLevel0 = valueLevel1.find(keyLevel0);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	if (iteratorLevel0 == valueLevel1.end()) { return nullptr; }
	//otherwise, we have to check if level 1 map has more elements and return the next one if it does
	iteratorLevel0++;
	if (iteratorLevel0 != valueLevel1.end()) { return &(*iteratorLevel0).second; }
	//there are no elements left in the current level 1 map, have to seek the next level 1 map in the level 2 map
	iteratorLevel1++;
	//if there are more elements in the current level 2 map, return the first element in the first one
	if (iteratorLevel1 != valueLevel2.end()) { return &(*(*iteratorLevel1).second.begin()).second; }
	// otherwise, we have to check the next level 2 map in the level 3 Map
	iteratorLevel2++;
	if (iteratorLevel2 != internalMap.end()) { return &(*(*(*iteratorLevel2).second.begin()).second.begin()).second; }
	// the provided keys had to point to the very last element of the map. Rewind to the first element.
	return GetFirstValue();
}

template<typename T>
const T* NestedMap3D<T>::GetValueBefore(int keyLevel2, int keyLevel1, int keyLevel0) const {
	auto iteratorLevel2 = this->internalMap.find(keyLevel2);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel2 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel1 == valueLevel2.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel1 = (*iteratorLevel1).second;
	auto iteratorLevel0 = valueLevel1.find(keyLevel0);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	if (iteratorLevel0 == valueLevel1.end()) { return nullptr; }
	//otherwise, we have to check if level 1 map has more elements and return the previous one if it does
	if (iteratorLevel0 != valueLevel1.begin()) { return &(*(--iteratorLevel0)).second; }
	//there are no elements left in the current level 1 map, have to seek the next level 1 map in the level 2 map
	//if there are more elements in the current level 2 map, return the last element in the last one
	if (iteratorLevel1 != valueLevel2.begin()) { return &(*(--(*(--iteratorLevel1)).second.end())).second; }
	// otherwise, we have to check the next level 2 map in the level 3 Map
	if (iteratorLevel2 !=
	    internalMap.begin()) { return &(*(--(*(--(*(--iteratorLevel2)).second.end())).second.end())).second; }
	// the provided keys had to point to the very first element of the map. Fast-forward to the last element to loop around.
	return GetLastValue();
}

template<typename T>
const T* NestedMap3D<T>::GetValueAt(int keyLevel2, int keyLevel1, int keyLevel0) const {
	auto iteratorLevel2 = this->internalMap.find(keyLevel2);
	//check if top-level map contains third-level key, if it doesn't, return null pointer
	if (iteratorLevel2 == (this->internalMap).end()) { return nullptr; }
	//if it does, we have to check the second level to see if we can retrieve the next level1 element
	auto& valueLevel2 = (*iteratorLevel2).second;
	auto iteratorLevel1 = valueLevel2.find(keyLevel1);
	//if the second level doesn't contain the original 2nd level key, return null pointer
	if (iteratorLevel1 == valueLevel2.end()) { return nullptr; }
	//otherwise, it does, we have to check the third level to see if we can retrieve the next level1 element
	auto& valueLevel1 = (*iteratorLevel1).second;
	auto iteratorLevel0 = valueLevel1.find(keyLevel0);
	//if we could not find the original level 1 key in the level 1 map, return nullptr
	return iteratorLevel0 == valueLevel1.end() ? nullptr : &(*iteratorLevel0).second;
}

template<typename T>
std::vector<T> NestedMap3D<T>::GetValues() const {
	std::vector<T> out;
	for (auto elementLevel2 : internalMap) {
		for (auto elementLevel1 : elementLevel2.second) {
			for (auto elementLevel0 : elementLevel1.second) {
				out.push_back(elementLevel0.second);
			}
		}
	}
	return out;
}







