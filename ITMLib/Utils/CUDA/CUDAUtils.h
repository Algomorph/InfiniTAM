//============================ BEGIN COPYRIGHT SECTION =================================================================
// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once


template<class T>
inline __device__ void warpReduce(volatile T* sdata, int tid) {
	sdata[tid] += sdata[tid + 32];
	sdata[tid] += sdata[tid + 16];
	sdata[tid] += sdata[tid + 8];
	sdata[tid] += sdata[tid + 4];
	sdata[tid] += sdata[tid + 2];
	sdata[tid] += sdata[tid + 1];
}

inline __device__ void warpReduce3(volatile float* sdata, int tid) {
	sdata[3*tid+0] += sdata[3*(tid + 32)+0];
	sdata[3*tid+1] += sdata[3*(tid + 32)+1];
	sdata[3*tid+2] += sdata[3*(tid + 32)+2];
	sdata[3*tid+0] += sdata[3*(tid + 16)+0];
	sdata[3*tid+1] += sdata[3*(tid + 16)+1];
	sdata[3*tid+2] += sdata[3*(tid + 16)+2];
	sdata[3*tid+0] += sdata[3*(tid + 8)+0];
	sdata[3*tid+1] += sdata[3*(tid + 8)+1];
	sdata[3*tid+2] += sdata[3*(tid + 8)+2];
	sdata[3*tid+0] += sdata[3*(tid + 4)+0];
	sdata[3*tid+1] += sdata[3*(tid + 4)+1];
	sdata[3*tid+2] += sdata[3*(tid + 4)+2];
	sdata[3*tid+0] += sdata[3*(tid + 2)+0];
	sdata[3*tid+1] += sdata[3*(tid + 2)+1];
	sdata[3*tid+2] += sdata[3*(tid + 2)+2];
	sdata[3*tid+0] += sdata[3*(tid + 1)+0];
	sdata[3*tid+1] += sdata[3*(tid + 1)+1];
	sdata[3*tid+2] += sdata[3*(tid + 1)+2];
}


__device__ static inline void atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMin(double* address, double val)
{
	unsigned long long int* address_as_i = (unsigned long long int*)address;
	unsigned long long int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
		                  __double_as_longlong(::fmin(val, __longlong_as_double(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(double* address, double val)
{
	unsigned long long int* address_as_i = (unsigned long long int*)address;
	unsigned long long int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
		                  __double_as_longlong(::fmax(val, __longlong_as_double(assumed))));
	} while (assumed != old);
}
#if !defined(__CUDA_ARCH__) || __CUDA_ARCH__ >= 600
#else
__device__ double atomicAdd(double* address, double val)
{
	unsigned long long int* address_as_ull = (unsigned long long int*)address;
	unsigned long long int old = *address_as_ull, assumed;
	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed,
		                __double_as_longlong(val + __longlong_as_double(assumed)));
	} while (assumed != old);
	return __longlong_as_double(old);
}
#endif

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = val;
}

template<typename T>
__global__ void memsetKernelLarge_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * (blockIdx.x + blockIdx.y * gridDim.x);
	if (offset >= nwords) return;
	devPtr[offset] = val;
}

template<typename T>
inline void memsetKernel(T *devPtr, const T val, size_t nwords)
{
	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)nwords / (float)blockSize.x));
	if (gridSize.x <= 65535) {
		memsetKernel_device<T> <<<gridSize,blockSize>>>(devPtr, val, nwords);
		ORcudaKernelCheck;
	} else {
		gridSize.x = (int)ceil(sqrt((float)gridSize.x));
		gridSize.y = (int)ceil((float)nwords / (float)(blockSize.x * gridSize.x));
		memsetKernelLarge_device<T> <<<gridSize,blockSize>>>(devPtr, val, nwords);
		ORcudaKernelCheck;
	}
}

template<typename T>
__global__ void fillArrayKernel_device(T *devPtr, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = offset;
}

template<typename T>
inline void fillArrayKernel(T *devPtr, size_t nwords)
{
	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)nwords / (float)blockSize.x));
	fillArrayKernel_device<T> <<<gridSize,blockSize>>>(devPtr, nwords);
	ORcudaKernelCheck;
}

//================================ END COPYRIGHT SECTION ===============================================================
//  ================================================================
//  Created by Gregory Kramida on 12/13/19.
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

__device__ static inline char atomicAdd(char* address, char val) {
	// offset, in bytes, of the char* address within the 32-bit address of the space that overlaps it
	size_t long_address_modulo = (size_t) address & 3;
	// the 32-bit address that overlaps the same memory
	auto* base_address = (unsigned int*) ((char*) address - long_address_modulo);
	// A 0x3210 selector in __byte_perm will simply select all four bytes in the first argument in the same order.
	// The "4" signifies the position where the first byte of the second argument will end up in the output.
	unsigned int selectors[] = {0x3214, 0x3240, 0x3410, 0x4210};
	// for selecting bytes within a 32-bit chunk that correspond to the char* address (relative to base_address)
	unsigned int selector = selectors[long_address_modulo];
	unsigned int long_old, long_assumed, long_val, replacement;

	long_old = *base_address;

	do {
		long_assumed = long_old;
		// replace bits in long_old that pertain to the char address with those from val
		long_val = __byte_perm(long_old, 0, long_address_modulo) + val;
		replacement = __byte_perm(long_old, long_val, selector);
		long_old = atomicCAS(base_address, long_assumed, replacement);
	} while (long_old != long_assumed);
	return __byte_perm(long_old, 0, long_address_modulo);
}


__device__ static inline char atomicAdd2(char* address, char val) {
	size_t long_address_modulo = (size_t) address & 3;
	auto* base_address = (unsigned int*) ((char*) address - long_address_modulo);
	unsigned int long_val = (unsigned int) val << (8 * long_address_modulo);
	unsigned int long_old = atomicAdd(base_address, long_val);

	if (long_address_modulo == 3) {
		// the first 8 bits of long_val represent the char value,
		// hence the first 8 bits of long_old represent its previous value.
		return (char) (long_old >> 24);
	} else {
		// bits that represent the char value within long_val
		unsigned int mask = 0x000000ff << (8 * long_address_modulo);
		unsigned int masked_old = long_old & mask;
		// isolate the bits that represent the char value within long_old, add the long_val to that,
		// then re-isolate by excluding bits that represent the char value
		unsigned int overflow = (masked_old + long_val) & ~mask;
		if (overflow) {
			atomicSub(base_address, overflow);
		}
		return (char) (masked_old >> 8 * long_address_modulo);
	}
}


__device__ static inline short atomicAdd(short* address, short val) {
	unsigned int* base_address = (unsigned int*) ((char*) address - ((size_t) address &
	                                                                 2));    //tera's revised version (showtopic=201975)
	unsigned int long_val = ((size_t) address & 2) ? ((unsigned int) val << 16) : (unsigned short) val;
	unsigned int long_old = atomicAdd(base_address, long_val);

	if ((size_t) address & 2) {
		return (short) (long_old >> 16);
	} else {
		unsigned int overflow = ((long_old & 0xffff) + long_val) & 0xffff0000;
		if (overflow)
			atomicSub(base_address, overflow);
		return (short) (long_old & 0xffff);
	}
}

__device__ static inline char atomicCAS(char* address, char expected, char desired) {
	size_t long_address_modulo = (size_t) address & 3;
	auto* base_address = (unsigned int*) ((char*) address - long_address_modulo);
	unsigned int selectors[] = {0x3214, 0x3240, 0x3410, 0x4210};

	unsigned int sel = selectors[long_address_modulo];
	unsigned int long_old, long_assumed, long_val, replacement;
	char old;

	long_val = (unsigned int) desired;
	long_old = *base_address;
	do {
		long_assumed = long_old;
		replacement = __byte_perm(long_old, long_val, sel);
		long_old = atomicCAS(base_address, long_assumed, replacement);
		old = (char) ((long_old >> (long_address_modulo * 8)) & 0x000000ff);
	} while (expected == old && long_assumed != long_old);

	return old;
}
