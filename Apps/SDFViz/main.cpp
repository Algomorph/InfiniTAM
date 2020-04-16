//  ================================================================
//  Created by Gregory Kramida on 12/21/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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
#include "SDFViz.h"
#include <boost/program_options.hpp>


namespace po = boost::program_options;

int main(int argc, const char* argv[]) {
	try {
		//visibility boolean flags
		bool hideNonInterestCanonicalVoxels = false;
		bool hideLiveVoxels = false;
		bool hideInterestCanonicalRegions = false;
		bool hideUnknownVoxels = false;
		bool loadSlices = false;
		bool slicesOnly = false;

		po::options_description description{"Options"};
		description.add_options()
				("help,h", "Help screen")
				("directory,d",
				 po::value<std::string>()->default_value("/media/algomorph/Data/Reconstruction/debug_output/scene"),
				 "Directory where to load the SDF scene from.")
				("hide_non_interest,hni", po::bool_switch(&hideNonInterestCanonicalVoxels),
				 "Hide non-interest canonical voxels on startup.")
				("hide_live,hl", po::bool_switch(&hideLiveVoxels),
				 "Hide live voxels on startup.")
				("hide_interest,hi", po::bool_switch(&hideInterestCanonicalRegions),
				 "Hide interest canonical voxels on startup.")
				("hide_unknown,hu", po::bool_switch(&hideUnknownVoxels),
				 "Hide canonical voxels marked as 'unknown' on startup.")
				("initial_focus_coord,ifc", po::value<std::vector<int>>()->multitoken(),
				 "Coordinate of voxel where to focus on startup. Must follow format:\n x y z\n, all integers.")
				("frame_index,f", po::value<unsigned int>()->default_value(0),
				 "Which frame to load first by default.")
				("load_slices,ls", po::bool_switch(&loadSlices),
				 "Load all existing slices for the initial frame.")
				("slices_only,so", po::bool_switch(&slicesOnly),
				 "Load only existing slices for the initial frame, don't load the full scene.");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
		po::notify(vm);

		if (vm.count("help") ||
		    //validate we have the right number of arguments to initial_focus_coord
		    (!vm["initial_focus_coord"].empty() && (vm["initial_focus_coord"].as<std::vector<int> >()).size() == 2)) {
			std::cout << description << std::endl;
		} else {
			bool haveUserInitialCoordinate = !vm["initial_focus_coord"].empty();
			Vector3i initialCoords(0);
			if (haveUserInitialCoordinate) {
				std::vector<int> initialCoordsVec = vm["initial_focus_coord"].as<std::vector<int> >();
				memcpy(initialCoords.values, initialCoordsVec.data(), sizeof(int) * 3);
				initialCoords.y *= -1;
				initialCoords.z *= -1;
			}
			SDFViz application(vm["directory"].as<std::string>(), hideNonInterestCanonicalVoxels, hideLiveVoxels,
			                   hideInterestCanonicalRegions, hideUnknownVoxels, haveUserInitialCoordinate,
			                   initialCoords, vm["frame_index"].as<unsigned int>(), loadSlices, slicesOnly);
			application.Run();
		}
	} catch (const po::error& ex) {
		std::cerr << ex.what() << std::endl;
	}
}