//  ================================================================
//  Created by Gregory Kramida on 1/16/18.
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

#include <map>
#include <vector>


namespace ITMLib {
class ITMIntArrayMap3D {
public:
	ITMIntArrayMap3D();
	ITMIntArrayMap3D(const char* prefixLevel3, const char* prefixLevel2, const char* prefixLevel1,
	                 const char* prefixLevel0);
	ITMIntArrayMap3D(const ITMIntArrayMap3D& intArrayMap);
	ITMIntArrayMap3D& operator=(ITMIntArrayMap3D& other);
	ITMIntArrayMap3D& operator=(ITMIntArrayMap3D&& other) noexcept;
	bool operator==(const ITMIntArrayMap3D &other) const;

	~ITMIntArrayMap3D();

	bool InsertOrdered(int keyLevel3, int keyLevel2, int keyLevel1, int valueLevel0);
	bool SaveToFile(const char* path);
	bool SaveToTextFile(const char* path);
	bool LoadFromFile(const char* path);
	ITMIntArrayMap3D FilterBasedOnLevel0Lengths(int minThreshold);


	std::vector<int> GetLevel3Keys();
	std::vector<int> GetOuterLevelKeys(){
		return GetLevel3Keys();
	};
	bool Contains(int keyLevel3, int keyLevel2, int keyLevel1, int valueLevel0);
	bool Contains(int keyLevel3, int keyLevel2);


	friend std::ostream& operator<<(std::ostream& stream, const ITMIntArrayMap3D& intArrayMap3D);

private:
	std::map<int, std::map<int, std::map<int, std::vector<int>>>> internalMap;
	const char* prefixLevel3;
	const char* prefixLevel2;
	const char* prefixLevel1;
	const char* prefixLevel0;

};
}

