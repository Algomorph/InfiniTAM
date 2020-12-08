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

#include "../../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"

namespace test {

template<typename TIndex>
std::string GetWarpsPath(std::string prefix, int iteration);

template<typename TIndex>
std::string GetWarpedLivePath(std::string prefix, int iteration);

template<typename TIndex>
std::string GetFusedPath(std::string prefix, int iteration);

unsigned int SwitchesToIntCode(const ITMLib::LevelSetAlignmentSwitches& switches);
std::string SwitchesToPrefix(const ITMLib::LevelSetAlignmentSwitches& switches);

} // namespace test_utilities