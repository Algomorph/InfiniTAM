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
#pragma once

#include <string>
#include <fstream>
#include "../Configuration/Configuration.h"

namespace ITMLib {
namespace logging {

class Log {
public:
	Log() = default;
	explicit Log(std::string path);
	template<typename T>
	void operator << (T item){
		if(!path.empty()){
			stream << item;
		}
	}
private:
	std::ofstream stream;
	std::string path = "";
};

void start_log(const std::string& name, const std::string& extension = "txt", const std::string& directory = configuration::Get().paths.output_path);
Log& get_log(std::string name);

}//namespace logging
}//namespace ITMLib