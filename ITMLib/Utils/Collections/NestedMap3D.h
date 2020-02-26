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
#pragma once

//stdlib
#include <vector>
#include <ostream>
#include <map>

namespace ITMLib{

template<typename T>
class NestedMap3D {
public:

	//*** Constructors and destructors ***
	NestedMap3D();
	NestedMap3D(const char* prefixLevel2, const char* prefixLevel1, const char* prefixLevel0);
	NestedMap3D(const NestedMap3D& nestedMap3D);
	~NestedMap3D();

	//*** Operator overloads ***
	NestedMap3D& operator=(NestedMap3D& other);
	NestedMap3D& operator=(NestedMap3D&& other) noexcept;
	bool operator==(const NestedMap3D &other) const;

	//*** I/O ****
	bool SaveToFile(const char* path);
	bool SaveToTextFile(const char* path);
	bool LoadFromFile(const char* path);
	template<typename FT>
	friend std::ostream& operator<<(std::ostream& stream, const NestedMap3D<FT>& nestedMap3D);

	//*** Insertions and transformations ***
	bool InsertOrdered(int keyLevel2, int keyLevel1, int keyLevel0, T valueLevel0);

	//*** Traversal, checks, and getters ***
	const T* GetFirstValue() const;
	const T* GetLastValue() const;
	const T* GetValueAfter(int keyLevel2, int keyLevel1, int keyLevel0) const;
	const T* GetValueBefore(int keyLevel2, int keyLevel1, int keyLevel0) const;
	const T* GetValueAt(int keyLevel2, int keyLevel1, int keyLevel0) const;
	std::vector<int> GetLevel2Keys();
	std::vector<int> GetOuterLevelKeys(){
		return GetLevel2Keys();
	};
	std::vector<T> GetValues() const;

	bool Contains(int keyLevel2, int keyLevel1, int keyLevel0, T valueLevel0);
	bool Contains(int keyLevel2, int keyLevel1, int keyLevel0);
	bool Contains(int keyLevel2, int keyLevel1);


private:
	std::map<int, std::map<int, std::map<int, T>>> internalMap;
	const char* prefixLevel2;
	const char* prefixLevel1;
	const char* prefixLevel0;

};

}//namespace ITMLib


