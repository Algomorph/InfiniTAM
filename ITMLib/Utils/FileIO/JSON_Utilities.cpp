//  ================================================================
//  Created by Gregory Kramida on 1/16/20.
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

#include "JSON_Utilities.h"


namespace boost {
namespace property_tree {

void write_json_no_quotes(std::basic_ostream<
		typename ptree::key_type::value_type
>& stream, const pt::ptree& ptree, bool pretty) {
	std::ostringstream oss;
	pt::write_json(oss, ptree, pretty);
	std::regex reg("\\\"(-?[0-9]+\\.{0,1}[0-9]*|true|false)\\\"");
	std::string result = std::regex_replace(oss.str(), reg, "$1");
	stream << result;
}

void write_json_no_quotes(const std::string& path, const pt::ptree& ptree, bool pretty) {
	std::ostringstream oss;
	pt::write_json(oss, ptree, pretty);
	std::regex reg("\\\"(-?[0-9]+\\.{0,1}[0-9]*|true|false)\\\"");
	std::string result = std::regex_replace(oss.str(), reg, "$1");

	std::ofstream file;
	file.open(path);
	file << result;
	file.close();
}
} //namespace property_tree
}//namespace boost