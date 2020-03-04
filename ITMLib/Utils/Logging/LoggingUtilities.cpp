//  ================================================================
//  Created by Gregory Kramida on 3/4/20.
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
#include "LoggingUtilities.h"
#include <map>
#include <utility>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace ITMLib{
namespace logging{
std::map<std::string, Log> text_logs;

Log::Log(std::string path) : stream(path.c_str(), std::ios_base::out | std::ios_base::ate), path(std::move(path)) {}


void start_log(const std::string& name, const std::string& extension, const std::string& directory) {

	if(text_logs.find(name) == text_logs.end()){
		text_logs[name] = Log((fs::path(directory) / fs::path(name + "." + extension)).string());
	}

}



Log& get_log(std::string name) {
	if(text_logs.find(name) == text_logs.end()){
		start_log(name);
	}
	return text_logs[name];
}


} // namespace logging
} // namespace ITMLib


