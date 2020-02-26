//  ================================================================
//  Created by Gregory Kramida on 9/3/19.
//  Copyright (c) 2019 Gregory Kramida
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
//stdlib
#include <iostream>

#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <iostream>

//boost
#include <boost/test/unit_test.hpp>
#include "../ITMLib/Utils/Collections/NestedMap3DOfArrays.h"


using namespace ITMLib;


BOOST_AUTO_TEST_CASE(testITMIntArrayMap3D) {
		NestedMap3DOfArrays<int> map("one", "two", "three", "four");
		const int maxElementsOnEachLevel = 3;

		for (int keyLevel3 = 0; keyLevel3 < maxElementsOnEachLevel; keyLevel3++) {
			for (int keyLevel2 = 0; keyLevel2 < maxElementsOnEachLevel; keyLevel2++) {
				for (int keyLevel1 = 0; keyLevel1 < maxElementsOnEachLevel; keyLevel1++) {
					for (int valueLevel0 = 0; valueLevel0 < maxElementsOnEachLevel; valueLevel0++) {
						map.InsertOrdered(keyLevel3, keyLevel2, keyLevel1, valueLevel0);
					}
				}
			}
		}
		const char* testFilename = "int_array_map_test.dat";
		map.SaveToFile(testFilename);
		NestedMap3DOfArrays<int> map2("one", "two", "three", "four");
		map2.LoadFromFile(testFilename);
		BOOST_REQUIRE(map == map2);


		NestedMap3DOfArrays<int> map3("one", "two", "three", "four");
		map3.InsertOrdered(84651, 358, 1, 5);
		map3.InsertOrdered(84651, 358, 1, 6);
		map3.InsertOrdered(102821, 436, 1, 1);
		map3.InsertOrdered(155667, 495, 1, 2);
		map3.InsertOrdered(179874, 446, 1, 28);
		map3.InsertOrdered(179874, 446, 1, 30);
		map3.SaveToFile(testFilename);
		NestedMap3DOfArrays<int> map4("one", "two", "three", "four");
		map4.LoadFromFile(testFilename);
		BOOST_REQUIRE(map3 == map4);
}