//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/14/20.
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
#include "RigidTrackerPresets.h"

namespace test {

const std::unordered_map<std::string, std::string> matrix_file_name_by_preset = [] {
	std::unordered_map<std::string, std::string> map(
			{
					{rgb_tracker_preset_rrbb,  "rgb_rrbb.mat"},
					{rgb_tracker_preset_rrrbb, "rgb_rrrbb.mat"},
					{rgb_tracker_preset_rrbrb, "rgb_rrbrb.mat"},
					{extended_tracker_preset1, "extended1.mat"}
			});
	return map;
}();
} // namespace test