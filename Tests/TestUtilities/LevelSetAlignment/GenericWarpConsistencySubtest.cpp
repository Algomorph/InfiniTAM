//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 10/8/20.
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
#include "GenericWarpConsistencySubtest.tpp"

namespace test{

template
void
GenericMultiIterationAlignmentSubtest<PlainVoxelArray, MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                                                         LevelSetAlignmentTestMode mode,
                                                                         float absolute_tolerance, float tolerance_divergence_factor);
template
void
GenericMultiIterationAlignmentSubtest<VoxelBlockHash, MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                                                        LevelSetAlignmentTestMode mode,
                                                                        float absolute_tolerance, float tolerance_divergence_factor);

#ifndef COMPILE_WITHOUT_CUDA
template
void
GenericMultiIterationAlignmentSubtest<PlainVoxelArray, MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                                                          LevelSetAlignmentTestMode mode,
                                                                          float absolute_tolerance, float tolerance_divergence_factor);

template
void
GenericMultiIterationAlignmentSubtest<VoxelBlockHash, MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                                                         LevelSetAlignmentTestMode mode,
                                                                         float absolute_tolerance, float tolerance_divergence_factor);
#endif

} // namespace test