//  ================================================================
//  Created by Gregory Kramida on 5/3/18.
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
#include <string>
#include <ostream>

namespace ITMLib {
namespace Bench {
void StartTimer(std::string name);
void StopTimer(std::string name);
void all_times_to_stream(std::ostream& out, bool colors_enabled);
void PrintAllCumulativeTimes();
void SaveAllCumulativeTimesToDisk();
double StopTimerAndGetCumulativeTime(std::string name);
double StopTimerAndGetLastTime(std::string name);
double GetCumulativeTime(std::string name);
}//namespace Bench
}//namespace ITMLib

