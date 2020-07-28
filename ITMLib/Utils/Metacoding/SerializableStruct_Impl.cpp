//  ================================================================
//  Created by Gregory Kramida on 1/8/20.
//  Copyright (c)  2020 Gregory Kramida
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

#include "SerializableStruct_Impl.h"

std::string preprocess_path(const std::string& path, const std::string& origin) {
	if (origin.empty()) return path;
	const std::regex configuration_directory_regex("^<CONFIGURATION_DIRECTORY>");
	std::string resulting_path;
	if (std::regex_search(path, configuration_directory_regex)) {
		std::string cleared = std::regex_replace(path, configuration_directory_regex, "");
		resulting_path = (boost::filesystem::path(origin).parent_path() / boost::filesystem::path(cleared)).string();
	} else {
		resulting_path = path;
	}
	return resulting_path;
}

std::string postprocess_path(const std::string& path, const std::string& origin) {
	if (origin.empty()) return path;
	const std::string configuration_directory_substitute = "<CONFIGURATION_DIRECTORY>";
	std::regex configuration_directory_regex(boost::filesystem::path(origin).parent_path().string());
	std::string resulting_path;
	if (std::regex_search(path, configuration_directory_regex)) {
		resulting_path = std::regex_replace(path, configuration_directory_regex, configuration_directory_substitute);
	} else {
		resulting_path = path;
	}
	return resulting_path;
}

boost::optional<std::string>
ptree_to_optional_path(const boost::property_tree::ptree& tree, const pt::ptree::key_type& key, const std::string& origin) {
	boost::optional<std::string> optional = tree.get_optional<std::string>(key);
	return optional ? boost::optional<std::string>(preprocess_path(optional.get(), origin)) : boost::optional<std::string>{};
}

std::string compile_sub_struct_parse_path(const std::string& current_parse_path, const std::string& sub_struct_instance_name) {
	if (current_parse_path.empty()) {
		return sub_struct_instance_name;
	} else {
		return current_parse_path + "." + sub_struct_instance_name;
	}
}

std::string find_snake_case_lowercase_acronym(const std::string& snake_case_identifier) {
	size_t pos = 0;
	std::string token;
	std::string acronym;
	std::string tmp = snake_case_identifier;

	while ((pos = tmp.find('_')) != std::string::npos) {
		token = tmp.substr(0, pos);
		acronym += token[0];
		tmp.erase(0, pos + 1);
	}
	acronym += tmp[0];
	return acronym;
}

void generate_cli_argument_identifiers_snake_case(const boost::program_options::options_description& options_description,
                                                  const std::string& parent_long_identifier, const std::string& parent_short_identifier,
                                                  const std::string& field_name, std::string& long_identifier, std::string& short_identifier) {
	std::string child_short_identifier = find_snake_case_lowercase_acronym(field_name);

	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, field_name);
	if (options_description.find_nothrow(long_identifier, false, false, false) != nullptr) {
		std::stringstream ss;
		ss << "There is a duplicate argument with full identifier \"" << long_identifier <<
		   "\". Perhaps a deferrable serializable struct's default parse path is equivalent to that of another serializable struct?";
		std::string error_string = ss.str();
		DIEWITHEXCEPTION_REPORTLOCATION(error_string +);
	}


	const int maximum_similar_shorthands_allowed = 5;
	std::string base_child_short_identifier = child_short_identifier;
	short_identifier = compile_sub_struct_parse_path(parent_short_identifier, child_short_identifier);

	for (int try_count = 0; options_description.find_nothrow(short_identifier, false, false, false) != nullptr &&
	                        try_count < maximum_similar_shorthands_allowed; try_count++) {
		const std::regex expression_ending_with_digit("\\w+(\\d)");
		std::smatch match;
		if (std::regex_match(child_short_identifier, match, expression_ending_with_digit)) {
			int index = std::stoi(match[1].str());
			child_short_identifier = base_child_short_identifier + std::to_string(index + 1);
		} else {
			child_short_identifier += "1";
		}
		short_identifier = compile_sub_struct_parse_path(parent_short_identifier, child_short_identifier);
	}
	if (options_description.find_nothrow(child_short_identifier, false, false, false) != nullptr) {
		std::stringstream ss;
		ss << "There are too many argument identifiers with similar shorthands based on the shorthand \"" << short_identifier <<
		   "\", please come up with an alternative argument name for argument \"" << long_identifier << "\".";
		std::string error_string = ss.str();
		DIEWITHEXCEPTION_REPORTLOCATION(error_string +);
	}
}