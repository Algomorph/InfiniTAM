//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/25/20.
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
// lambdas
#ifdef __CUDACC__
#define CPU_AND_GPU_LAMBDA(...) [=] __host__ __device__
#else
#define CPU_AND_GPU_LAMBDA(...) [__VA_ARGS__]
#endif

// code that should always be compiled for the GPU if the CUDA compiler is involved
#if defined(__CUDACC__)
#define _DEVICE_WHEN_AVAILABLE_ __device__
#else
#define _DEVICE_WHEN_AVAILABLE_
#endif

// code that should always be compiled for both the CPU and the GPU if the CUDA compiler is involved
#if defined(__CUDACC__)
#define _HOST_AND_DEVICE_ __host__ __device__
#else
#define _HOST_AND_DEVICE_
#endif