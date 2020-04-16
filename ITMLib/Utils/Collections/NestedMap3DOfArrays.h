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
#pragma once

//stdlib
#include <vector>
#include <ostream>
#include <map>

namespace ITMLib{

template<typename T>
class NestedMap3DOfArrays {
public:

	//*** Constructors and destructors ***
	NestedMap3DOfArrays();
	NestedMap3DOfArrays(const char* prefixLevel3, const char* prefixLevel2, const char* prefixLevel1,
	                    const char* prefixLevel0);
	NestedMap3DOfArrays(const NestedMap3DOfArrays& nestedMap3D);
	~NestedMap3DOfArrays();

	//*** Operator overloads ***
	NestedMap3DOfArrays& operator=(NestedMap3DOfArrays& other);
	NestedMap3DOfArrays& operator=(NestedMap3DOfArrays&& other) noexcept;
	bool operator==(const NestedMap3DOfArrays &other) const;

	//*** I/O ****
	bool SaveToFile(std::string path);
	bool SaveToTextFile(std::string path);
	bool LoadFromFile(std::string path);
	template<typename FT>
	friend std::ostream& operator<<(std::ostream& stream, const NestedMap3DOfArrays<FT>& nestedMap3D);

	//*** Insertions and filtering ***
	bool InsertOrdered(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0);
	NestedMap3DOfArrays FilterBasedOnLevel0Lengths(int minThreshold);
	void Clear();

	//*** Traversal, checks, and getters ***
	const std::vector<T>* GetFirstArray() const;
	const std::vector<T>* GetLastArray() const;
	const std::vector<T>* GetArrayAfter(int keyLevel3, int keyLevel2, int keyLevel1) const;
	const std::vector<T>* GetArrayBefore(int keyLevel3, int keyLevel2, int keyLevel1) const;
	const std::vector<T>* GetArrayAt(int keyLevel3, int keyLevel2, int keyLevel1) const;


	std::vector<int> GetLevel3Keys() const;
	std::vector<int> GetOuterLevelKeys() const{
		return GetLevel3Keys();
	};
	std::vector<std::vector<T>> GetArrays() const;

	bool Contains(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0) const;
	bool Contains(int keyLevel3, int keyLevel2, int keyLevel1) const;
	bool Contains(const int& keyLevel3, const int& keyLevel2) const;


private:
	std::map<int, std::map<int, std::map<int, std::vector<T>>>> internalMap;
	const char* prefixLevel3;
	const char* prefixLevel2;
	const char* prefixLevel1;
	const char* prefixLevel0;

};

}//namespace ITMLib


