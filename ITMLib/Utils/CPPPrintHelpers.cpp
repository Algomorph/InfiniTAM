//  ================================================================
//  Created by Gregory Kramida on 3/22/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include "CPPPrintHelpers.h"

namespace ITMLib{

std::string int_to_padded_string(int i, int fill_width){
	std::stringstream stringstream;
	if(i > 0){
		stringstream << std::setfill('0') << std::setw(fill_width) << i;
	}else{
		stringstream << '-' << std::setfill('0') << std::setw(fill_width-1) << -i;
	}
	return stringstream.str();
}

const std::string red("\033[0;31m");
const std::string green("\033[0;32m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string cyan("\033[0;36m");


const std::string bright_red("\033[0;91m");
const std::string bright_cyan("\033[0;96m");
const std::string reset("\033[0m");
}// namespace ITMLib