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
//Boost
#include <boost/program_options.hpp>

// local
#include "../../ITMLib/Utils/Configuration/Configuration.h"


namespace po = boost::program_options;
using namespace ITMLib;

int main(int argc, char** argv) {
	po::options_description options_description{"Arguments"};
	options_description.add_options()
			("help,h", "Print help screen")
			( "config,cfg", po::value<std::string>(),
			  "Configuration file in JSON format, e.g.  ./default_config_cuda.json "
			  "WARNING: using this option will invalidate any other command line arguments.");
	configuration::CompileOptionDescription(options_description);
	options_description.add_options()("config_output,co", po::value<std::string>(), " where to save the generated config file.");
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options_description).
			style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);
	po::notify(vm);
	auto print_help = [&options_description, &argv]() {
		std::cout << options_description << std::endl;
	};
	if (vm.count("help")) {
		print_help();
		return EXIT_SUCCESS;
	}
	if (vm["config"].empty()) {
		configuration::LoadConfigurationFromVariableMap(vm);
	} else {
		auto size = vm["size"];

		std::string config_path = vm["config"].as<std::string>();
		configuration::LoadConfigurationFromJSONFile(config_path);
		configuration::UpdateConfigurationFromVariableMap(vm);
	}

	if(!vm["config_output"].empty()){
		configuration::SaveConfigurationToJSONFile(vm["config_output"].as<std::string>());
	}
}