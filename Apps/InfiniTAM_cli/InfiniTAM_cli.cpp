// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
//stdlib
#include <cstdlib>
#include <iostream>

#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//local
#include "CLIEngine.h"
#include "CreateDefaultImageSource.h"

//InputSource
#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"

//ITMLib
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Engines/Main/MainEngineFactory.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace po = boost::program_options;
namespace pt = boost::property_tree;

int main(int argc, char** argv) {
	try {
		po::options_description arguments{"Arguments"};
		po::positional_options_description positional_arguments;

		arguments.add_options()
				("help,h", "Print help screen")
				("halp,h", "Funkaaay")
				( "config,cfg", po::value<std::string>(),
				  "Configuration file in JSON format, e.g.  ./default_config_cuda.json "
				  "WARNING: using this option will invalidate any other command line arguments.");

		ITMLib::configuration::Configuration::AddToOptionsDescription(arguments);

		positional_arguments.add("calibration_file", 1);
		positional_arguments.add("input_path", 3);

		po::variables_map vm;

		po::store(po::command_line_parser(argc, argv).options(arguments).positional(positional_arguments).style(
				po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

		po::notify(vm);


		auto printHelp = [&arguments, &positional_arguments, &argv]() {
			std::cout << arguments << std::endl;
			std::cout << "Positional arguments: " << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(0) << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(1) << std::endl;
			printf("examples:\n"
			       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
			       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0]);
		};

		if (vm.count("help")) {
			printHelp();
			return EXIT_SUCCESS;
		}

		if (vm.count("halp")) {
			printf("Ya didn't think it would work, did ya now?\n");
			printHelp();
			return EXIT_SUCCESS;
		}

		if (vm["config"].empty()) {
			configuration::LoadConfigurationFromVariableMap(vm);
		} else {
			std::string configPath = vm["config"].as<std::string>();
			configuration::LoadConfigurationFromJSONFile(configPath);
		}
		auto& configuration = configuration::Get();

		printf("initialising ...\n");
		ImageSourceEngine* imageSource = nullptr;
		IMUSourceEngine* imuSource = nullptr;

		CreateDefaultImageSource(imageSource, imuSource, configuration.paths);
		FusionAlgorithm* mainEngine = BuildMainEngine(imageSource->getCalib(),
		                                              imageSource->GetRGBImageSize(),
		                                              imageSource->GetDepthImageSize());

		CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, configuration::Get().device_type);
		CLIEngine::Instance()->Run();
		CLIEngine::Instance()->Shutdown();

		delete mainEngine;
		delete imageSource;
		delete imuSource;
		return 0;
	}
	catch (std::exception& e) {
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
}