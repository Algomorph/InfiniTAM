//  ================================================================
//  Created by Gregory Kramida on 2/24/20.
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
#pragma once


// TODO: remove this crazy header as soon as CLion CUDA IDE support is released _CLION_CUDA
#if defined(__clang__) || defined(__JETBRAINS_IDE__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wignored-attributes"

//#define __host__
//#define __device__
//#define __global__
//#define __noinline__
//#define __forceinline__
//#define __shared__
//#define __constant__
//#define __managed__
//#define __restrict__

// This is slightly mental, but gets it to properly index device function calls like __popc and whatever.
#define __CUDACC__
#include "/usr/local/cuda-10.2/targets/x86_64-linux/include/device_functions.h"

// These headers are all implicitly present when you compile CUDA with clang. Clion doesn't know that, so
// we include them explicitly to make the indexer happy. Doing this when you actually build is, obviously,
// a terrible idea :D
#include "/usr/local/cuda-10.2/targets/x86_64-linux/include/vector_types.h"
#include "/usr/lib/llvm-6.0/lib/clang/6.0.0/include/__clang_cuda_builtin_vars.h"
#include "/usr/lib/llvm-6.0/lib/clang/6.0.0/include/__clang_cuda_intrinsics.h"
#include "/usr/lib/llvm-6.0/lib/clang/6.0.0/include/__clang_cuda_math_forward_declares.h"
#include "/usr/lib/llvm-6.0/lib/clang/6.0.0/include/__clang_cuda_complex_builtins.h"
#include "/usr/lib/llvm-6.0/lib/clang/6.0.0/include/__clang_cuda_cmath.h"
#include "../../../../../usr/local/cuda/targets/x86_64-linux/include/curand_kernel.h"


#endif