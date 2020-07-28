//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/28/20.
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
#include "DeferrableSerializableStruct_Impl.h"

std::string generate_cli_argument_short_identifier_from_long_identifier(const std::string& long_identifier) {
	size_t pos = 0;
	std::string token;
	std::string tmp = long_identifier;

	std::string short_identifier;

	while ((pos = tmp.find('.')) != std::string::npos) {
		token = tmp.substr(0, pos);
		short_identifier = compile_sub_struct_parse_path(short_identifier, find_snake_case_lowercase_acronym(token));
		tmp.erase(0, pos + 1);
	}
	short_identifier = compile_sub_struct_parse_path(short_identifier, find_snake_case_lowercase_acronym(token));
	return short_identifier;
}