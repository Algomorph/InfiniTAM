//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 10/8/20.
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

#include "TestCaseOrganizationBySwitches.h"
#include <TestUtilitiesConfig.h>
#include "../TestUtilities.h"

using namespace ITMLib;

namespace test_utilities {


template<typename TIndex>
std::string GetPathBase(std::string prefix, int iteration) {
	return GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/" + prefix + "_iter_" + std::to_string(iteration) + "_";
}

template<typename TIndex>
std::string GetWarpsPath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "warps.dat";
}

template<typename TIndex>
std::string GetWarpedLivePath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "warped_live.dat";
}

template<typename TIndex>
std::string GetFusedPath(std::string prefix, int iteration) {
	return GetPathBase<TIndex>(prefix, iteration) + "fused.dat";
}

} // end namespace test_utilities