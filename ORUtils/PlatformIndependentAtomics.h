//  ================================================================
//  Created by Gregory Kramida on 10/10/19.
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
#pragma once
#if !defined(__CUDACC__)
#include <atomic>

template<typename T>
inline
T atomicMax_CPU(std::atomic<T>& variable, T value) {
	auto current = variable.load();
	while (current < value && !variable.compare_exchange_weak(current, value, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}

template<typename T>
inline
T atomicMin_CPU(std::atomic<T>& variable, T value) {
	auto current = variable.load();
	while (current > value && !variable.compare_exchange_weak(current, value, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}


template<typename T>
inline
T atomicAdd_CPU(std::atomic<T>& variable, T addend) {
	auto current = variable.load();
	while (!variable.compare_exchange_weak(current, current + addend, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}

template<typename T>
inline
T atomicSub_CPU(std::atomic<T>& variable, T subtracted) {
	auto current = variable.load();
	while (!variable.compare_exchange_weak(current, current - subtracted, std::memory_order_relaxed, std::memory_order_relaxed));
	return current;
}

template<>
inline
int atomicAdd_CPU<int>(std::atomic<int>& variable, int addend){
	return variable.fetch_add(addend, std::memory_order_relaxed);
}

template<>
inline
int atomicSub_CPU<int>(std::atomic<int>& variable, int subtracted){
	return variable.fetch_sub(subtracted, std::memory_order_relaxed);
}

template <typename T>
inline void initializeAtomic_CPU(std::atomic<T>& var, T value){
	var.store(value);
}

template<>
inline
unsigned int atomicAdd_CPU<unsigned int>(std::atomic<unsigned int>& variable, unsigned int addend){
	return variable.fetch_add(addend, std::memory_order_relaxed);
}
#endif

#if defined(__CUDACC__)
template <typename T>
inline T getDataCPU(T* var){
	T var_val;
	ORcudaSafeCall(cudaMemcpy(&var_val, var, sizeof(T), cudaMemcpyDeviceToHost));
	return var_val;
}
#endif

#if defined(__CUDACC__)
// for CUDA device code
#define DECLARE_ATOMIC(type, name) type* name = nullptr
#define DECLARE_ATOMIC_INT(name)  int* name = nullptr
#define DECLARE_ATOMIC_UINT(name) unsigned int* name = nullptr
#define DECLARE_ATOMIC_FLOAT(name) float* name = nullptr
#define DECLARE_ATOMIC_DOUBLE(name) double* name = nullptr
#define CLEAN_UP_ATOMIC(name) ORcudaSafeCall (cudaFree(name))
#define GET_ATOMIC_VALUE(name) (* name)
#define GET_ATOMIC_VALUE_CPU(name) getDataCPU( name )
#define ATOMIC_ARGUMENT(type) type *
#define ATOMIC_FLOAT_ARGUMENT_TYPE float*
#define ATOMIC_INT_ARGUMENT_TYPE int*

#define INITIALIZE_ATOMIC(type, var, value) \
{ \
type val = value; \
ORcudaSafeCall(cudaMalloc((void**)&var, sizeof( type ))); \
ORcudaSafeCall(cudaMemcpy(var, &val, sizeof( type ), cudaMemcpyHostToDevice)); \
}
#else
#define DECLARE_ATOMIC(type, name) std::atomic< type > name = {0}
#define DECLARE_ATOMIC_INT(name)  std::atomic<int> name = {0}
#define DECLARE_ATOMIC_UINT(name)  std::atomic<unsigned int> name = {0}
#define DECLARE_ATOMIC_FLOAT(name) std::atomic<float> name = {0}
#define DECLARE_ATOMIC_DOUBLE(name) std::atomic<double> name = {0}
#define CLEAN_UP_ATOMIC(name) ;
#define GET_ATOMIC_VALUE(name) (name .load())
#define GET_ATOMIC_VALUE_CPU(name) GET_ATOMIC_VALUE(name)
#define ATOMIC_ARGUMENT(type) std::atomic< type >&
#define ATOMIC_FLOAT_ARGUMENT_TYPE std::atomic<float>&
#define ATOMIC_INT_ARGUMENT_TYPE std::atomic<int>&

#define INITIALIZE_ATOMIC(type, var, value) \
initializeAtomic_CPU< type > (var, value)

#endif

#if defined(__CUDACC__)
#define ATOMIC_ADD(name, value) atomicAdd( name, value )
#define ATOMIC_SUB(name, value) atomicSub( name, value )
#define ATOMIC_MAX(name, value) atomicMax( name, value )
#define ATOMIC_MIN(name, value) atomicMin( name, value )
#define _DEVICE_WHEN_AVAILABLE_ __device__
#else
#define ATOMIC_ADD(name, value) atomicAdd_CPU( name, value )
#define ATOMIC_SUB(name, value) atomicSub_CPU( name, value )
#define ATOMIC_MAX(name, value) atomicMax_CPU( name, value )
#define ATOMIC_MIN(name, value) atomicMin_CPU( name, value )
#define _DEVICE_WHEN_AVAILABLE_
#endif


