//  ================================================================
//  Created by Gregory Kramida on 4/11/20.
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

// Credits: Inspired by ORUtils/MemoryBlock.h in original InfiniTAM (https://github.com/victorprad/InfiniTAM/) and ORUtils (https://github.com/carlren/ORUtils)

#pragma once

#include "MemoryDeviceType.h"
#include "PlatformIndependence.h"

#ifndef __METALC__

#ifndef COMPILE_WITHOUT_CUDA

#include "CUDADefines.h"

#endif

#ifdef COMPILE_WITH_METAL
#include "MetalContext.h"
#endif

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <iterator>

namespace ORUtils {
/** \brief
Represents memory blocks, templated on the data type
*/
template<typename T, typename A = std::allocator<T> >
class MemoryBlock {

public:

// region =================== TYPEDEFS =================================================================================

	typedef A allocator_type;
	typedef typename A::reference reference;
	typedef typename A::const_reference const_reference;
	typedef typename A::difference_type difference_type;
	typedef typename A::value_type value_type;
	typedef typename A::size_type size_type;
// endregion ===========================================================================================================
// region =================== ITERATORS ================================================================================

	class iterator {
	public:
		typedef iterator self_type;
		typedef T value_type;
		typedef T& reference;
		typedef T* pointer;
		typedef std::input_iterator_tag iterator_category;
		typedef int difference_type;

		iterator(pointer ptr, MemoryDeviceType access_mode) : ptr_(ptr), access_mode(access_mode) {}

		self_type operator++() {
			self_type i = *this;
			ptr_++;
			return i;
		}

		self_type operator++(int junk) {
			ptr_++;
			return *this;
		}

		value_type operator*() {
			switch (access_mode) {
				case MEMORYDEVICE_CPU: {
					return *ptr_;
				}
#ifndef COMPILE_WITHOUT_CUDA
				case MEMORYDEVICE_CUDA: {
					T result;
					ORcudaSafeCall(cudaMemcpy(&result, this->ptr_, sizeof(T), cudaMemcpyDeviceToHost));
					return result;
				}
#endif
				default:
					throw std::runtime_error("Invalid access mode.");
			}
		}

		pointer operator->() {
			switch (access_mode) {
				case MEMORYDEVICE_CPU: {
					return ptr_;
				}
#ifndef COMPILE_WITHOUT_CUDA
				case MEMORYDEVICE_CUDA: {
					ORcudaSafeCall(cudaMemcpy(&buffer_value, this->ptr_, sizeof(T), cudaMemcpyDeviceToHost));
					return &buffer_value;
				}
#endif
				default:
					throw std::runtime_error("Invalid access mode.");
			}
		}

		bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_ && this->access_mode == rhs.access_mode; }

		bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_ || this->access_mode != rhs.access_mode; }

	private:
		T buffer_value;
		pointer ptr_;
		const MemoryDeviceType access_mode;
	};

// endregion ===========================================================================================================

	MemoryBlock()
			: element_count(0),
			  is_allocated_for_CPU(false),
			  is_allocated_for_CUDA(false),
			  is_metal_compatible(false),
			  data_cpu(nullptr),
			  data_cuda(nullptr),
			  access_mode(MEMORYDEVICE_NONE) {}


#ifdef COMPILE_WITH_METAL
	inline const void *GetMetalBuffer() const { return data_metal_buffer; }
#endif

	/**
	 * Initialize an empty memory block of the given size,
	 * on CPU only or GPU only or on both. CPU might also use the
	 * Metal compatible allocator (i.e. with 16384 alignment).
	*/
	MemoryBlock(size_type element_count, bool allocate_CPU, bool allocate_CUDA, bool make_metal_compatible = true)
			: is_allocated_for_CPU(false), is_allocated_for_CUDA(false),
#ifdef COMPILE_WITH_METAL
			is_metal_compatible(make_metal_compatible),
#else
              is_metal_compatible(false),
#endif
#ifdef COMPILE_WITHOUT_CUDA
			access_mode(allocate_CPU ? MEMORYDEVICE_CPU : MEMORYDEVICE_NONE)
#else
              access_mode(allocate_CPU ? MEMORYDEVICE_CPU : allocate_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_NONE) {
#endif
		Allocate(element_count, allocate_CPU, allocate_CUDA, make_metal_compatible);
		Clear();
	}

	/**
	 * Initialize an empty memory block of the given size, either
	 * on CPU only or on GPU only. CPU allocation will be Metal-compatible if Metal
	 * is enabled during compilation.
	*/
	MemoryBlock(size_type element_count, MemoryDeviceType memory_type) :
			is_allocated_for_CPU(false), is_allocated_for_CUDA(false),
#ifdef COMPILE_WITH_METAL
			is_metal_compatible(true),
#else
			is_metal_compatible(false),
#endif
#ifdef COMPILE_WITHOUT_CUDA
			access_mode(memory_type == MEMORYDEVICE_NONE ? MEMORYDEVICE_NONE : MEMORYDEVICE_CPU)
#else
			access_mode(memory_type) {
#endif

		switch (memory_type) {
			case MEMORYDEVICE_METAL:
			case MEMORYDEVICE_CPU:
				Allocate(element_count, true, false, true);
				break;
			case MEMORYDEVICE_CUDA: {
				Allocate(element_count, false, true, true);
				break;
			}
			default:
				break;
		}

		Clear();
	}


	virtual ~MemoryBlock() {
		this->Free();
	}

	MemoryBlock(MemoryBlock&& other) noexcept:
			element_count(other.element_count), // for code clarity
			is_allocated_for_CPU(other.is_allocated_for_CPU),
			is_allocated_for_CUDA(other.is_allocated_for_CUDA),
			is_metal_compatible(other.is_metal_compatible),
			data_cpu(other.data_cpu),
			data_cuda(other.data_cuda),
#ifdef COMPILE_WITH_METAL
			data_metal_buffer(other.data_metal_buffer),
#endif
			access_mode(other.access_mode) {
		other.element_count = 0;
		other.is_allocated_for_CPU = false;
		other.is_allocated_for_CUDA = false;
		other.is_metal_compatible = false;
		other.data_cpu = nullptr;
		other.data_cuda = nullptr;
		other.access_mode = MEMORYDEVICE_NONE;
	}

	MemoryBlock(const MemoryBlock& other) :
			element_count(0),
			is_allocated_for_CPU(false),
			is_allocated_for_CUDA(false),
			is_metal_compatible(false),
			access_mode(other.access_mode) {
		Allocate(other.element_count, other.is_allocated_for_CPU, other.is_allocated_for_CUDA, other.is_metal_compatible);
		if (is_allocated_for_CPU) {
			this->SetFrom(other, MemoryCopyDirection::CPU_TO_CPU);
		}
		if (is_allocated_for_CUDA) {
			this->SetFrom(other, MemoryCopyDirection::CUDA_TO_CUDA);
		}
	}

	MemoryBlock& operator=(MemoryBlock other) {
		swap(*this, other);
		return *this;
	}

	/**
	 * Count of elements allocated in the data array.
	 * */
	size_type size() const { return this->element_count; };

	/** Get the data pointer for CPU or GPU. */
	inline DEVICEPTR(T)* GetData(MemoryDeviceType memory_type = MEMORYDEVICE_NONE) {
		switch (memory_type) {
			case MEMORYDEVICE_CPU:
				assert(this->is_allocated_for_CPU);
				return data_cpu;
			case MEMORYDEVICE_CUDA:
				assert(this->is_allocated_for_CUDA);
				return data_cuda;
			case MEMORYDEVICE_NONE:
				switch (access_mode) {
					case MEMORYDEVICE_CPU:
						return data_cpu;
					case MEMORYDEVICE_CUDA:
						return data_cuda;
				}
		}
		assert(false);
		return nullptr;
	}

	/** Get the const data pointer for CPU or GPU. */
	inline const DEVICEPTR(T)* GetData(MemoryDeviceType memory_type = MEMORYDEVICE_NONE) const {
		switch (memory_type) {
			case MEMORYDEVICE_CPU:
				assert(this->is_allocated_for_CPU);
				return data_cpu;
			case MEMORYDEVICE_CUDA:
				assert(this->is_allocated_for_CUDA);
				return data_cuda;
			case MEMORYDEVICE_NONE:
				switch (access_mode) {
					case MEMORYDEVICE_CPU:
						return data_cpu;
					case MEMORYDEVICE_CUDA:
						return data_cuda;
				}
		}
		assert(false);
		return nullptr;
	}

	MemoryDeviceType GetAccessMode() const { return this->access_mode; }

	//TODO: make a Clear method that accepts a T default value (use cuda kernel to propagate the value throughout array)
	/**
	 * Set all data to the given @p defaultValue.
	 * */
	void Clear(unsigned char defaultValue = 0) {
		if (is_allocated_for_CPU) memset(data_cpu, defaultValue, element_count * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
		if (is_allocated_for_CUDA) ORcudaSafeCall(cudaMemset(data_cuda, defaultValue, element_count * sizeof(T)));
#endif
	}

	/**
	 * Resize a memory block, losing all old data.
	 * Essentially any previously allocated data is
	 * released, new memory is allocated.
	 * */
	void Resize(size_t newDataSize, bool forceReallocation = true) {
		if (newDataSize == element_count) return;

		if (newDataSize > element_count || forceReallocation) {
			bool allocate_CPU = this->is_allocated_for_CPU;
			bool allocate_CUDA = this->is_allocated_for_CUDA;
			bool metalCompatible = this->is_metal_compatible;

			this->Free();
			this->Allocate(newDataSize, allocate_CPU, allocate_CUDA, metalCompatible);
		}

		this->element_count = newDataSize;
	}

	/** Transfer data from CPU to GPU, if possible. */
	void UpdateDeviceFromHost() const {
#ifndef COMPILE_WITHOUT_CUDA
		if (is_allocated_for_CUDA && is_allocated_for_CPU)
			ORcudaSafeCall(cudaMemcpy(data_cuda, data_cpu, element_count * sizeof(T), cudaMemcpyHostToDevice));
#endif
	}

	/** Transfer data from GPU to CPU, if possible. */
	void UpdateHostFromDevice() const {
#ifndef COMPILE_WITHOUT_CUDA
		if (is_allocated_for_CUDA && is_allocated_for_CPU)
			ORcudaSafeCall(cudaMemcpy(data_cpu, data_cuda, element_count * sizeof(T), cudaMemcpyDeviceToHost));
#endif
	}

	/** Copy data */
	void SetFrom(const MemoryBlock<T>& source) {
		SetFrom(source, DetermineMemoryCopyDirection(this->access_mode, source.access_mode));
	}

	/** Copy data */
	void SetFrom(const MemoryBlock<T>& source, MemoryCopyDirection memory_copy_direction) {
		Resize(source.element_count);
		switch (memory_copy_direction) {
			case CPU_TO_CPU:
				memcpy(this->data_cpu, source.data_cpu, source.element_count * sizeof(T));
				break;
#ifndef COMPILE_WITHOUT_CUDA
			case CPU_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source.data_cpu, source.element_count * sizeof(T),
				                               cudaMemcpyHostToDevice));
				break;
			case CUDA_TO_CPU:
				ORcudaSafeCall(cudaMemcpy(this->data_cpu, source.data_cuda, source.element_count * sizeof(T),
				                          cudaMemcpyDeviceToHost));
				break;
			case CUDA_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source.data_cuda, source.element_count * sizeof(T),
				                               cudaMemcpyDeviceToDevice));
				break;
#endif
			default:
				break;
		}
	}

	/** Get an individual element of the memory block from either the CPU or GPU. */
	T GetElement(int n, MemoryDeviceType memory_type) const {
		switch (memory_type) {
			case MEMORYDEVICE_CPU: {
				return this->data_cpu[n];
			}
#ifndef COMPILE_WITHOUT_CUDA
			case MEMORYDEVICE_CUDA: {
				T result;
				ORcudaSafeCall(cudaMemcpy(&result, this->data_cuda + n, sizeof(T), cudaMemcpyDeviceToHost));
				return result;
			}
#endif
			default:
				throw std::runtime_error("Invalid memory type");
		}
	}


	T GetElement(size_type index) const {
		assert(index < element_count);
		switch (access_mode) {
			case MEMORYDEVICE_CPU:
				return data_cpu[index];
#ifndef COMPILE_WITHOUT_CUDA
			case MEMORYDEVICE_CUDA: {
				T result;
				ORcudaSafeCall(cudaMemcpy(&result, this->data_cuda + index, sizeof(T), cudaMemcpyDeviceToHost));
				return result;
			}
#endif
			case MEMORYDEVICE_NONE:
				throw std::runtime_error("No memory allocated for data on any memory device type.");
			case MEMORYDEVICE_METAL:
			default:
				throw std::runtime_error("Invalid memory type");

		}
	}

	T operator[](size_type index) const {
		return GetElement(index);
	}

	iterator begin() const {
		switch (access_mode) {
			case MEMORYDEVICE_CPU:
				return iterator(this->data_cpu, MEMORYDEVICE_CPU);
#ifndef COMPILE_WITHOUT_CUDA
			case MEMORYDEVICE_CUDA:
				return iterator(this->data_cuda, MEMORYDEVICE_CUDA);
#endif
			case MEMORYDEVICE_NONE:
				throw std::runtime_error("No memory allocated for data on any memory device type.");
			case MEMORYDEVICE_METAL:
			default:
				throw std::runtime_error("Invalid memory type");

		}
	}

	iterator end() const {
		switch (access_mode) {
			case MEMORYDEVICE_CPU:
				return iterator(this->data_cpu + element_count, MEMORYDEVICE_CPU);
#ifndef COMPILE_WITHOUT_CUDA
			case MEMORYDEVICE_CUDA:
				return iterator(this->data_cuda + element_count, MEMORYDEVICE_CUDA);
#endif
			case MEMORYDEVICE_NONE:
				throw std::runtime_error("No memory allocated for data on any memory device type.");
			case MEMORYDEVICE_METAL:
			default:
				throw std::runtime_error("Invalid memory type");

		}
	}


	void Swap(MemoryBlock<T>& rhs) {
		using std::swap;
		swap(this->element_count, rhs.element_count);
		swap(this->is_allocated_for_CPU, rhs.is_allocated_for_CPU);
		swap(this->is_allocated_for_CUDA, rhs.is_allocated_for_CUDA);
		swap(this->is_metal_compatible, rhs.is_metal_compatible);
		swap(this->data_cpu, rhs.data_cpu);
		swap(this->data_cuda, rhs.data_cuda);
#ifdef COMPILE_WITH_METAL
		swap(this->data_metal_buffer, rhs.data_metal_buffer);
#endif
		swap(this->access_mode, rhs.access_mode);
	}

	friend void swap(MemoryBlock<T>& lhs, MemoryBlock<T>& rhs) { // nothrow
		lhs.Swap(rhs);
	}

protected:
	size_type element_count;
	bool is_allocated_for_CPU, is_allocated_for_CUDA, is_metal_compatible;
	/** Pointer to memory on CPU host. */
	DEVICEPTR(T)* data_cpu;
	/** Pointer to memory on GPU, if available. */
	DEVICEPTR(T)* data_cuda;

#ifdef COMPILE_WITH_METAL
	void *data_metal_buffer;
#endif
	MemoryDeviceType access_mode;

private:
	enum AllocationMethod {
		ALLOCATION_ON_CPU_WITHOUT_CUDA_OR_METAL,
		ALLOCATION_ON_CPU_WITH_CUDA,
		ALLOCATION_ON_CPU_AND_GPU_WITH_METAL
	};

	/**
	 * Allocate element data of the specified size with the specified method(s). If the
	 * data has been allocated before, the data is freed.
	*/
	void Allocate(size_type element_count, bool allocate_CPU, bool allocate_CUDA, bool metal_compatible) {
		Free();

		this->element_count = element_count;

		if (allocate_CPU) {
			AllocationMethod allocation_method = ALLOCATION_ON_CPU_WITHOUT_CUDA_OR_METAL;

#ifndef COMPILE_WITHOUT_CUDA
			if (allocate_CUDA) allocation_method = ALLOCATION_ON_CPU_WITH_CUDA;
#endif
#ifdef COMPILE_WITH_METAL
			if (is_metal_compatible) allocation_method = ALLOCATION_ON_CPU_AND_GPU_WITH_METAL;
#endif
			switch (allocation_method) {
				case ALLOCATION_ON_CPU_WITHOUT_CUDA_OR_METAL:
					if (element_count == 0) data_cpu = nullptr;
					else data_cpu = new T[element_count];
					break;
				case ALLOCATION_ON_CPU_WITH_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
					if (element_count == 0) data_cpu = nullptr;
					else
						ORcudaSafeCall(cudaMallocHost((void**) &data_cpu, element_count * sizeof(T)));
#endif
					break;
				case ALLOCATION_ON_CPU_AND_GPU_WITH_METAL:
#ifdef COMPILE_WITH_METAL
					if (element_count == 0) data_cpu = nullptr;
					else allocateMetalData((void**)&data_cpu, (void**)&data_metal_buffer, (int)(element_count * sizeof(T)), true);
#endif
					break;
			}

			this->is_allocated_for_CPU = allocate_CPU;
			this->is_metal_compatible = metal_compatible;
		}

		if (allocate_CUDA) {
#ifndef COMPILE_WITHOUT_CUDA
			if (element_count == 0) data_cuda = nullptr;
			else
				ORcudaSafeCall(cudaMalloc((void**) &data_cuda, element_count * sizeof(T)));
			this->is_allocated_for_CUDA = allocate_CUDA;
#endif
		}
	}

	void Free() {
		if (is_allocated_for_CPU) {
			AllocationMethod allocation_method = ALLOCATION_ON_CPU_WITHOUT_CUDA_OR_METAL;

#ifndef COMPILE_WITHOUT_CUDA
			if (is_allocated_for_CUDA) allocation_method = ALLOCATION_ON_CPU_WITH_CUDA;
#endif
#ifdef COMPILE_WITH_METAL
			if (is_metal_compatible) allocation_method = ALLOCATION_ON_CPU_AND_GPU_WITH_METAL;
#endif
			switch (allocation_method) {
				case ALLOCATION_ON_CPU_WITHOUT_CUDA_OR_METAL:
					if (data_cpu != nullptr) delete[] data_cpu;
					break;
				case ALLOCATION_ON_CPU_WITH_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
					if (data_cpu != nullptr) ORcudaSafeCall(cudaFreeHost(data_cpu));
#endif
					break;
				case ALLOCATION_ON_CPU_AND_GPU_WITH_METAL:
#ifdef COMPILE_WITH_METAL
					if (data_cpu != nullptr) freeMetalData((void**)&data_cpu, (void**)&data_metal_buffer, (int)(element_count * sizeof(T)), true);
#endif
					break;
			}

			is_metal_compatible = false;
			is_allocated_for_CPU = false;
		}

		if (is_allocated_for_CUDA) {
#ifndef COMPILE_WITHOUT_CUDA
			if (data_cuda != nullptr) ORcudaSafeCall(cudaFree(data_cuda));
#endif
			is_allocated_for_CUDA = false;
		}
	}

};

}

#endif // __METALC__
