//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/8/20.
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

#include <utility>
#include <unordered_map>

namespace test {

constexpr const char* rgb_tracker_preset_rrbb = "type=rgb,levels=rrbb";
constexpr const char* rgb_tracker_preset_rrrbb = "type=rgb,levels=rrrbb";
constexpr const char* rgb_tracker_preset_rrbrb = "type=rgb,levels=rrbrb";
constexpr const char* extended_tracker_preset1 = "type=extended,levels=bbb,useDepth=1,useColour=1,colourWeight=0.3,"
                                                 "minstep=1e-4,outlierColourC=0.175,outlierColourF=0.005,"
                                                 "outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=50,"
                                                 "tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

inline const std::unordered_map<const char*, std::string> matrix_file_name_by_preset = [] {
	std::unordered_map<const char*, std::string> map(
			{
					{rgb_tracker_preset_rrbb,  "rgb_rrbb.mat"},
					{rgb_tracker_preset_rrrbb, "rgb_rrrbb.mat"},
					{rgb_tracker_preset_rrbrb, "rgb_rrbrb.mat"},
					{extended_tracker_preset1, "extended1.mat"}
			});
	return map;
}();

} // namespace test_utilities







