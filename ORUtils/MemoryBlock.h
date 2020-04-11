// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

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

#include <stdlib.h>
#include <string.h>
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
			switch (access_mode){
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

	/** Total number of allocated entries in the data array. */
	size_type size() const { return element_count; };

	/** Get the data pointer on CPU or GPU. */
	inline DEVICEPTR(T)* GetData(MemoryDeviceType memoryType) {
		switch (memoryType) {
			case MEMORYDEVICE_CPU:
				return data_cpu;
			case MEMORYDEVICE_CUDA:
				return data_cuda;
		}

		return 0;
	}

	/** Get the data pointer on CPU or GPU. */
	inline const DEVICEPTR(T)* GetData(MemoryDeviceType memoryType) const {
		switch (memoryType) {
			case MEMORYDEVICE_CPU:
				return data_cpu;
			case MEMORYDEVICE_CUDA:
				return data_cuda;
		}

		return 0;
	}

#ifdef COMPILE_WITH_METAL
	inline const void *GetMetalBuffer() const { return data_metalBuffer; }
#endif

	/** Initialize an empty memory block of the given size,
	on CPU only or GPU only or on both. CPU might also use the
	Metal compatible allocator (i.e. with 16384 alignment).
	*/
	MemoryBlock(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: isAllocated_CPU(false), isAllocated_CUDA(false), isMetalCompatible(false),
#ifdef COMPILE_WITHOUT_CUDA
			access_mode(allocate_CPU ? MEMORYDEVICE_CPU : MEMORYDEVICE_NONE)
#else
              access_mode(allocate_CPU ? MEMORYDEVICE_CPU : allocate_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_NONE) {
#endif
		Allocate(dataSize, allocate_CPU, allocate_CUDA, metalCompatible);
		Clear();
	}

	/** Initialize an empty memory block of the given size, either
	on CPU only or on GPU only. CPU will be Metal compatible if Metal
	is enabled.
	*/
	MemoryBlock(size_t dataSize, MemoryDeviceType memoryType) :
			isAllocated_CPU(false), isAllocated_CUDA(false), isMetalCompatible(false),
#ifdef COMPILE_WITHOUT_CUDA
			access_mode(memoryType == MEMORYDEVICE_NONE ? MEMORYDEVICE_NONE : MEMORYDEVICE_CPU)
#else
			access_mode(memoryType) {
#endif

		switch (memoryType) {
			case MEMORYDEVICE_CPU:
				Allocate(dataSize, true, false, true);
				break;
			case MEMORYDEVICE_CUDA: {
				Allocate(dataSize, false, true, true);
				break;
			}
		}

		Clear();
	}

	/** Set all image data to the given @p defaultValue. */
	void Clear(unsigned char defaultValue = 0) {
		if (isAllocated_CPU) memset(data_cpu, defaultValue, element_count * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
		if (isAllocated_CUDA) ORcudaSafeCall(cudaMemset(data_cuda, defaultValue, element_count * sizeof(T)));
#endif
	}

	/** Resize a memory block, losing all old data.
	Essentially any previously allocated data is
	released, new memory is allocated.
	*/
	void Resize(size_t newDataSize, bool forceReallocation = true) {
		if (newDataSize == element_count) return;

		if (newDataSize > element_count || forceReallocation) {
			bool allocate_CPU = this->isAllocated_CPU;
			bool allocate_CUDA = this->isAllocated_CUDA;
			bool metalCompatible = this->isMetalCompatible;

			this->Free();
			this->Allocate(newDataSize, allocate_CPU, allocate_CUDA, metalCompatible);
		}

		this->element_count = newDataSize;
	}

	/** Transfer data from CPU to GPU, if possible. */
	void UpdateDeviceFromHost() const {
#ifndef COMPILE_WITHOUT_CUDA
		if (isAllocated_CUDA && isAllocated_CPU)
			ORcudaSafeCall(cudaMemcpy(data_cuda, data_cpu, element_count * sizeof(T), cudaMemcpyHostToDevice));
#endif
	}

	/** Transfer data from GPU to CPU, if possible. */
	void UpdateHostFromDevice() const {
#ifndef COMPILE_WITHOUT_CUDA
		if (isAllocated_CUDA && isAllocated_CPU)
			ORcudaSafeCall(cudaMemcpy(data_cpu, data_cuda, element_count * sizeof(T), cudaMemcpyDeviceToHost));
#endif
	}

	/** Copy data */
	void SetFrom(const MemoryBlock<T>* source, MemoryCopyDirection memoryCopyDirection) {
		Resize(source->element_count);
		switch (memoryCopyDirection) {
			case CPU_TO_CPU:
				memcpy(this->data_cpu, source->data_cpu, source->element_count * sizeof(T));
				break;
#ifndef COMPILE_WITHOUT_CUDA
			case CPU_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cpu, source->element_count * sizeof(T),
				                               cudaMemcpyHostToDevice));
				break;
			case CUDA_TO_CPU:
				ORcudaSafeCall(cudaMemcpy(this->data_cpu, source->data_cuda, source->element_count * sizeof(T),
				                          cudaMemcpyDeviceToHost));
				break;
			case CUDA_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cuda, source->element_count * sizeof(T),
				                               cudaMemcpyDeviceToDevice));
				break;
#endif
			default:
				break;
		}
	}

	/** Get an individual element of the memory block from either the CPU or GPU. */
	T GetElement(int n, MemoryDeviceType memoryType) const {
		switch (memoryType) {
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

	T operator[](size_type index) const{
		return GetElement(index);
	}

	iterator begin() const{
		switch (access_mode){
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

	iterator end() const{
		switch (access_mode){
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


	virtual ~MemoryBlock() { this->Free(); }

	/** Allocate image data of the specified size. If the
	data has been allocated before, the data is freed.
	*/
	void Allocate(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible) {
		Free();

		this->element_count = dataSize;

		if (allocate_CPU) {
			int allocType = 0;

#ifndef COMPILE_WITHOUT_CUDA
			if (allocate_CUDA) allocType = 1;
#endif
#ifdef COMPILE_WITH_METAL
			if (metalCompatible) allocType = 2;
#endif
			switch (allocType) {
				case 0:
					if (dataSize == 0) data_cpu = NULL;
					else data_cpu = new T[dataSize];
					break;
				case 1:
#ifndef COMPILE_WITHOUT_CUDA
					if (dataSize == 0) data_cpu = NULL;
					else
						ORcudaSafeCall(cudaMallocHost((void**) &data_cpu, dataSize * sizeof(T)));
#endif
					break;
				case 2:
#ifdef COMPILE_WITH_METAL
					if (element_count == 0) data_cpu = NULL;
					else allocateMetalData((void**)&data_cpu, (void**)&data_metalBuffer, (int)(element_count * sizeof(T)), true);
#endif
					break;
			}

			this->isAllocated_CPU = allocate_CPU;
			this->isMetalCompatible = metalCompatible;
		}

		if (allocate_CUDA) {
#ifndef COMPILE_WITHOUT_CUDA
			if (dataSize == 0) data_cuda = NULL;
			else
				ORcudaSafeCall(cudaMalloc((void**) &data_cuda, dataSize * sizeof(T)));
			this->isAllocated_CUDA = allocate_CUDA;
#endif
		}
	}

	void Free() {
		if (isAllocated_CPU) {
			int allocType = 0;

#ifndef COMPILE_WITHOUT_CUDA
			if (isAllocated_CUDA) allocType = 1;
#endif
#ifdef COMPILE_WITH_METAL
			if (isMetalCompatible) allocType = 2;
#endif
			switch (allocType) {
				case 0:
					if (data_cpu != NULL) delete[] data_cpu;
					break;
				case 1:
#ifndef COMPILE_WITHOUT_CUDA
					if (data_cpu != NULL) ORcudaSafeCall(cudaFreeHost(data_cpu));
#endif
					break;
				case 2:
#ifdef COMPILE_WITH_METAL
					if (data_cpu != NULL) freeMetalData((void**)&data_cpu, (void**)&data_metalBuffer, (int)(element_count * sizeof(T)), true);
#endif
					break;
			}

			isMetalCompatible = false;
			isAllocated_CPU = false;
		}

		if (isAllocated_CUDA) {
#ifndef COMPILE_WITHOUT_CUDA
			if (data_cuda != NULL) ORcudaSafeCall(cudaFree(data_cuda));
#endif
			isAllocated_CUDA = false;
		}
	}

	void Swap(MemoryBlock<T>& rhs) {
		std::swap(this->element_count, rhs.element_count);
		std::swap(this->data_cpu, rhs.data_cpu);
		std::swap(this->data_cuda, rhs.data_cuda);
#ifdef COMPILE_WITH_METAL
		std::swap(this->data_metalBuffer, rhs.data_metalBuffer);
#endif
		std::swap(this->isAllocated_CPU, rhs.isAllocated_CPU);
		std::swap(this->isAllocated_CUDA, rhs.isAllocated_CUDA);
		std::swap(this->isMetalCompatible, rhs.isMetalCompatible);
	}

	// Suppress the default copy constructor and assignment operator
	MemoryBlock(const MemoryBlock&);
	MemoryBlock& operator=(const MemoryBlock&);

protected:
	size_type element_count;

	bool isAllocated_CPU, isAllocated_CUDA, isMetalCompatible;
	/** Pointer to memory on CPU host. */
	DEVICEPTR(T)* data_cpu;

	/** Pointer to memory on GPU, if available. */
	DEVICEPTR(T)* data_cuda;

#ifdef COMPILE_WITH_METAL
	void *data_metalBuffer;
#endif
	MemoryDeviceType access_mode;

};

}

#endif
