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
namespace bench {
void start_timer(std::string name);
void stop_timer(std::string name);
void all_cumulative_times_to_stream(std::ostream& out, bool colors_enabled);
void print_all_cumulative_times_to_stdout();
void save_all_cumulative_times_to_disk();
void log_all_timers();
double stop_timer_and_get_cumulative_time(std::string name);
double stop_timer_and_get_last_time(std::string name);
double get_cumulative_time(std::string name);
}//namespace bench
}//namespace ITMLib

