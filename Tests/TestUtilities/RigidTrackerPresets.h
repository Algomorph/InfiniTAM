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

constexpr const char* rgb_tracker_preset_t = "type=rgb,levels=t";
constexpr const char* rgb_tracker_preset_r = "type=rgb,levels=r";
constexpr const char* rgb_tracker_preset_b = "type=rgb,levels=b";
constexpr const char* rgb_tracker_preset_rrbb = "type=rgb,levels=rrbb";
constexpr const char* rgb_tracker_preset_rrrbb = "type=rgb,levels=rrrbb";
constexpr const char* rgb_tracker_preset_rrrbrb = "type=rgb,levels=rrrbrb";
constexpr const char* extended_tracker_preset1 = "type=extended,levels=bbb,useDepth=1,useColour=1,colourWeight=0.3,"
                                                 "minstep=1e-4,outlierColourC=0.175,outlierColourF=0.005,"
                                                 "outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=50,"
                                                 "tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";
constexpr const char* extended_tracker_preset2 = "type=extended,levels=rrrb,useDepth=1,useColour=1,colourWeight=0.1,"
                                                 "minstep=2e-4,outlierColourC=0.125,outlierColourF=0.01,"
                                                 "outlierSpaceC=0.2,outlierSpaceF=0.005,numiterC=22,numiterF=52,"
                                                 "tukeyCutOff=9,framesToSkip=10,framesToWeight=50,failureDec=20.0";

constexpr const char* depth_tracker_preset_default =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=5.0";
constexpr const char* depth_tracker_preset_loop_closure =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=20.0";
constexpr const char* depth_tracker_preset_rrbrb =
		"type=icp,levels=rrbrb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=5.0";
constexpr const char* depth_tracker_preset_rr =
		"type=icp,levels=rr,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=5.0";
constexpr const char* depth_tracker_preset_b =
		"type=icp,levels=b,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=5.0";
constexpr const char* depth_tracker_preset_b_bigger_minstep =
		"type=icp,levels=b,minstep=2e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=20,numiterF=4,failureDec=5.0";
constexpr const char* depth_tracker_preset_b_more_iters =
		"type=icp,levels=b,minstep=1e-3,"
		"outlierC=0.005,outlierF=0.001,"
		"numiterC=40,numiterF=8,failureDec=5.0";


extern const std::unordered_map<std::string, std::string> matrix_file_name_by_preset;
extern const std::array<std::string, 7> depth_tracker_presets;
extern const std::array<std::string, 6> color_tracker_presets;

} // namespace test







