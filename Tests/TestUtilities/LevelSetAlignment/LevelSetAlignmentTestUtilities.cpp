//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
//  Copyright (c) 2019 Gregory Kramida
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
//local
#include "LevelSetAlignmentTestUtilities.tpp"

namespace test_utilities {


template
void PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CPU>(int iteration, LevelSetAlignmentSwitches tracker_switches, float absolute_tolerance);

template
void GenericWarpTest<MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                       LevelSetAlignmentTestMode mode, float absolute_tolerance);


#ifndef COMPILE_WITHOUT_CUDA

template
void PVA_to_VBH_WarpComparisonSubtest<MEMORYDEVICE_CUDA>(int iteration, LevelSetAlignmentSwitches tracker_switches, float absolute_tolerance);

template
void GenericWarpTest<MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                        LevelSetAlignmentTestMode mode, float absolute_tolerance);
#endif





} // namespace test_utilities