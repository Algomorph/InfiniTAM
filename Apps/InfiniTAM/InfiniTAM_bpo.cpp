// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//TODO: not supported on platforms besides Linux, adjust via CMake -Greg(GitHub: Algomorph)
#ifndef WIN32
#include <X11/Xlib.h>
#endif

//stdlib
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#ifdef WITH_VTK
//VTK
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkContextScene.h>

//ITMLib/VTK
#include "../../ITMLib/Utils/Visualization/VisualizationWindowManager.h"
#endif

//ITMLib
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Engines/Main/BasicVoxelEngine.h"
#include "../../ITMLib/Engines/Main/BasicSurfelEngine.h"
#include "../../ITMLib/Engines/Main/MultiEngine.h"
#include "../../ITMLib/Engines/Main/DynamicSceneVoxelEngine.h"
#include "../../ITMLib/Engines/Main/MainEngineFactory.h"

//local
#include "UIEngine_BPO.h"
#include "prettyprint.hpp"
#include "CreateDefaultImageSource.h"

// *** namespaces ***

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace po = boost::program_options;
namespace pt = boost::property_tree;


TelemetryRecorder_Interface& GetLogger(configuration::IndexingMethod method) {
	switch (method) {
		case configuration::INDEX_HASH: {
			return static_cast<TelemetryRecorder_Interface&>(TelemetryRecorder<TSDFVoxel, WarpVoxel, VoxelBlockHash>::Instance());
		}
		case configuration::INDEX_ARRAY: {
			return static_cast<TelemetryRecorder_Interface&>(TelemetryRecorder<TSDFVoxel, WarpVoxel, PlainVoxelArray>::Instance());
		}
	}
};

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
			configuration::load_configuration_from_variable_map(vm);
		} else {
			std::string configPath = vm["config"].as<std::string>();
			configuration::load_configuration_from_json_file(configPath);
		}
		auto& configuration = configuration::get();

		printf("initialising ...\n");
		ImageSourceEngine* imageSource = nullptr;
		IMUSourceEngine* imuSource = nullptr;

		CreateDefaultImageSource(imageSource, imuSource, configuration.paths);
		if (imageSource == nullptr) {
			std::cerr << "Failed to open any image stream." << std::endl;
			printHelp();
			return EXIT_FAILURE;
		}

// region ================================ BUILD MAIN ENGINE ========================================================
		configuration::IndexingMethod chosenIndexingMethod = configuration.indexing_method;
		//TelemetryRecorder_Interface& logger = GetLogger(chosenIndexingMethod);
		MainEngine* mainEngine = BuildMainEngine(imageSource->getCalib(),
		                                         imageSource->GetRGBImageSize(),
		                                         imageSource->GetDepthImageSize(),
		                                         false);

// endregion ===========================================================================================================

// region =========================== SET UI ENGINE SETTINGS WITH CLI ARGUMENTS ========================================


		//TODO (see top of file)
#if !defined(WIN32) && defined(WITH_VTK)
		XInitThreads();
#endif
		UIEngine_BPO::Instance().Initialize(argc, argv, imageSource, imuSource, mainEngine, configuration);


// endregion ===========================================================================================================

		//VisualizationWindowManager::get().Run();
		UIEngine_BPO::Instance().Run();
		UIEngine_BPO::Instance().Shutdown();

// region ========================================= CLEANUP ============================================================

		delete mainEngine;
		delete imageSource;
		delete imuSource;

		//VisualizationWindowManager::get().ShutDown();
// endregion ===========================================================================================================
		return EXIT_SUCCESS;
	} catch (std::exception& e) {
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
}

