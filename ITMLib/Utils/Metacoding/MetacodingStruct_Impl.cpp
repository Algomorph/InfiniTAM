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

#include "MetacodingStruct_Impl.h"

std::string preprocess_path(const std::string& path, const std::string& origin) {
	const std::regex configuration_directory_regex("^<CONFIGURATION_DIRECTORY>");
	std::string resulting_path;
	if (origin != "" && std::regex_search(path, configuration_directory_regex)) {
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
	if (origin != "" && std::regex_search(path, configuration_directory_regex)) {
		resulting_path = std::regex_replace(path, configuration_directory_regex, configuration_directory_substitute);
	} else {
		resulting_path = path;
	}
	return resulting_path;
}

boost::optional<std::string> ptree_to_optional_path(const boost::property_tree::ptree& tree, const pt::ptree::key_type& key, const std::string& origin ){
	boost::optional<std::string> optional = tree.get_optional<std::string>(key);
	return optional ? boost::optional<std::string>(preprocess_path(optional.get(), origin)) : boost::optional<std::string>{};
}