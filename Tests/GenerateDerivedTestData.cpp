//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/20/20.
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
// === stdlib ===
#include <filesystem>
#include <map>
#include <iostream>

// === ORUtils ===
#include "../ORUtils/MemoryDeviceType.h"
// === ITMLib ===
#include "../ITMLib/Utils/Logging/Logging.h"
#include "../ITMLib/Utils/Metacoding/SerializableEnum.h"
#include "../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../ITMLib/Objects/Volume/VoxelBlockHash.h"
// === TestDataGeneration ===
#include "TestDataGeneration/ConstructSnoopyVolumes.h"
#include "TestDataGeneration/ConstructTeddyVolumes.h"
#include "TestDataGeneration/GenerateLevelSetAlignmentDeviceComparisonData.h"
#include "TestDataGeneration/GenerateVolumeWarpingConsistencyData.h"
#include "TestDataGeneration/GenerateLevelSetAlignmentIndexComparisonTestData.h"
#include "TestDataGeneration/GenerateVolumeFusionConsistencyData.h"
#include "TestDataGeneration/GenerateTestConfigurations.h"
#include "TestDataGeneration/GenerateSnoopyMeshes.h"
#include "TestDataGeneration/GenerateRaycastingTestData.h"
#include "TestDataGeneration/GenerateRigidAlignmentTestData.h"

using namespace ITMLib;


namespace fs = std::filesystem;




#define GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION GeneratedTestDataType, \
    (SNOOPY_UNMASKED_VOLUMES,                "SNOOPY_UNMASKED_VOLUMES", "snoopy_unmasked_volumes", "unmasked_volumes", "unmasked", "u", "su", "suv"), \
    (SNOOPY_MASKED_VOLUMES,                  "SNOOPY_MASKED_VOLUMES", "snoopy_masked_volumes", "masked_volumes", "masked", "sm", "smv", "mv"), \
    (TEDDY_VOLUMES,                          "TEDDY_VOLUMES", "teddy_volumes", "Teddy_Volumes", "tv"),\
    (LEVEL_SET_ALIGNMENT_DEVICE_COMPARISON,  "LEVEL_SET_ALIGNMENT_DEVICE_COMPARISON", "lsa_device_comparison", "lsa_d"), \
    (LEVEL_SET_ALIGNMENT_INDEX_COMPARISON,   "LEVEL_SET_ALIGNMENT_INDEX_COMPARISON", "lsa_index_comparison", "lsa_i"), \
    (PVA_WARPED_VOLUMES,                     "PVA_WARPED_VOLUMES", "pva_warped_volumes", "pva_wv"), \
    (VBH_WARPED_VOLUMES,                     "VBH_WARPED_VOLUMES", "vbh_warped_volumes", "vbh_wv"), \
    (PVA_FUSED_VOLUMES,                      "PVA_FUSED_VOLUMES", "pva_fused_volumes", "pva_fv"), \
    (VBH_FUSED_VOLUMES,                      "VBH_FUSED_VOLUMES", "vbh_fused_volumes", "vbh_fv"), \
    (CONFUGRATION,                           "CONFIGURATION", "configuration", "config", "c"), \
    (MESHES,                                 "MESHES", "meshes", "m"), \
    (RENDERING,                              "RENDERING", "rendering", "r"), \
    (RIGID_ALIGNMENT,                        "RIGID_ALIGNMENT", "rigid_alignment", "ra")

GENERATE_SERIALIZABLE_ENUM(GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION);

int main(int argc, char* argv[]) {
	ITMLib::logging::InitializeTestLogging();

	std::map<GeneratedTestDataType, std::function<void()>> generator_by_string(
			{
					{SNOOPY_UNMASKED_VOLUMES,               ConstructSnoopyUnmaskedVolumes00},
					{SNOOPY_MASKED_VOLUMES,                 ConstructSnoopyMaskedVolumes16and17},
					{TEDDY_VOLUMES,                         ConstructTeddyVolumes115_Partial},
					{LEVEL_SET_ALIGNMENT_DEVICE_COMPARISON, GenerateLevelSetAlignment_CPU_vs_CUDA_TestData},
					{LEVEL_SET_ALIGNMENT_INDEX_COMPARISON,  GenerateLevelSetAlignment_PVA_vs_VBH_TestData},
					{PVA_WARPED_VOLUMES,                    GenerateWarpedVolumeTestData<PlainVoxelArray>},
					{VBH_WARPED_VOLUMES,                    GenerateWarpedVolumeTestData<VoxelBlockHash>},
					{PVA_FUSED_VOLUMES,                     GenerateFusedVolumeTestData<PlainVoxelArray>},
					{VBH_FUSED_VOLUMES,                     GenerateFusedVolumeTestData<VoxelBlockHash>},
					{CONFUGRATION,                          GenerateConfigurationTestData},
					{MESHES,                                GenerateMeshingTestData<VoxelBlockHash, MEMORYDEVICE_CPU>},
					{RENDERING,                             GenerateRenderingTestData_VoxelBlockHash<MEMORYDEVICE_CPU>},
					{RIGID_ALIGNMENT,                       GenerateRigidAlignmentTestData<VoxelBlockHash, MEMORYDEVICE_CPU>}
			}
	);
	if (argc < 2) {
		// calls every generator iteratively
		for (const auto& iter : generator_by_string) {
			std::cout << "Generating data using the " << enumerator_to_string(iter.first) << " generator." << std::endl;
			(iter.second)();
		}
	} else {
		std::string generated_data_type_argument = argv[1];
		bool wrong_second_argument = argc >= 3 && strcmp(argv[2], "-c") != 0;
		if (generated_data_type_argument == "h" || generated_data_type_argument == "help" || generated_data_type_argument == "-h" ||
		    generated_data_type_argument == "-help" || generated_data_type_argument == "--help" || wrong_second_argument) {
			std::cout << "Generates derived data used for testing the library. " << std::endl;
			std::cout << "Usage:" << std::endl << "generate_derived_test::snoopy " << std::endl << "(runs all modes)  -- OR -- "
			          << std::endl << "generate_derived_test::snoopy <mode> [-c]" << std::endl <<
			          ", where <mode> can be one of: " << std::endl << std::endl;
			int i_pair = 0;
			for (auto& pair : generator_by_string) {
				if (i_pair < generator_by_string.size() - 1) {
					std::cout << enumerator_to_string(pair.first) << "( alternative tokens: " << enumerator_to_string_token_list(pair.first) << "), "
					          << std::endl;
				} else {
					std::cout << "or " << enumerator_to_string(pair.first) << "( alternative tokens: " << enumerator_to_string_token_list(pair.first)
					          << ").";
				}
				i_pair++;
			}
			std::cout << std::endl;

			std::cout << "For any of these, shorthands can be used, which are typically acronyms with some words omitted"
			             ", e.g. \"suv\" can be used instead of \"SNOOPY_UNMASKED_VOLUMES\" and \"pva_wv\" instead of \"PVA_WARPED_VOLUMES\". "
			             "Don't be afraid to experiment." << std::endl;
			std::cout
					<< "If -c (\"continue\") flag is passed, all python_generators including and after the specified one in the sequence are called in order."
					<< std::endl;
		} else if (argc < 3) {
			GeneratedTestDataType chosen = string_to_enumerator<GeneratedTestDataType>(generated_data_type_argument);
			std::cout << "current path: " << std::filesystem::current_path() << std::endl;
			std::cout << "Generating data using the " << enumerator_to_string(chosen) << " generator." << std::endl;
			generator_by_string[chosen]();
		} else {
			bool hit_start_generator = false;
			GeneratedTestDataType chosen = string_to_enumerator<GeneratedTestDataType>(generated_data_type_argument);
			for (const auto& iter : generator_by_string) {
				if (iter.first == chosen || hit_start_generator) {
					std::cout << "Generating data using the " << enumerator_to_string(iter.first) << " generator." << std::endl;
					(iter.second)();
					hit_start_generator = true;
				}
			}
		}
	}
	return 0;
}