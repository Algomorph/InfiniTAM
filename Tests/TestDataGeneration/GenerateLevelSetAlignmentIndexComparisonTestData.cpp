//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/24/20.
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

// === ORUtils ===
#include "../../ORUtils/MemoryDeviceType.h"

// === ITMLib ===
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"

// === test_utilities ===
#include "../TestUtilities/TestDataUtilities.h"
#include "../TestUtilities/LevelSetAlignment/LevelSetAlignmentTestUtilities.h"

// === local ===
#include "GenerateLevelSetAlignmentIndexComparisonTestData.h"

using namespace ITMLib;
using namespace test;

void GenerateLevelSetAlignment_PVA_vs_VBH_TestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating multi-iteration warp field data from snoopy masked partial volumes 16 & 17 (PVA & VBH)... ");
	LevelSetAlignmentSwitches switches_data_only(true, false, false, false, false);

	GenericMultiIterationAlignmentTest<MEMORYDEVICE_CPU>(switches_data_only, 10, SAVE_SUCCESSIVE_ITERATIONS, 0, 1.1f);
	LevelSetAlignmentSwitches switches_data_and_tikhonov(true, false, true, false, false);
	GenericMultiIterationAlignmentTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov, 5, SAVE_SUCCESSIVE_ITERATIONS, 0, 1.1f);
	LevelSetAlignmentSwitches switches_data_and_tikhonov_and_sobolev_smoothing(true, false, true, false, true);
	GenericMultiIterationAlignmentTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov_and_sobolev_smoothing, 5, SAVE_SUCCESSIVE_ITERATIONS, 0, 1.1f);
	GenericMultiIterationAlignmentTest<MEMORYDEVICE_CPU>(LevelSetAlignmentSwitches(true, false, true, false, true),
	                                                     3, LevelSetAlignmentTestMode::SAVE_FINAL_ITERATION_AND_FUSION, 0, 1.1f);
}