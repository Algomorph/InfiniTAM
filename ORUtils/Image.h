// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MemoryBlock.h"
#include "Vector.h"

#ifndef __METALC__

namespace ORUtils {
/** \brief
Represents images, templated on the pixel type
*/
template<typename T>
class Image : private MemoryBlock<T> {
public:

	/** Expose public MemoryBlock<T> member functions. */
	using MemoryBlock<T>::size;
	using MemoryBlock<T>::Clear;
	using MemoryBlock<T>::GetData;
	using MemoryBlock<T>::GetElement;
	using MemoryBlock<T>::GetAccessMode;
	using MemoryBlock<T>::SetFrom;

#ifdef COMPILE_WITH_METAL
	using MemoryBlock<T>::GetMetalBuffer();
#endif
	using MemoryBlock<T>::UpdateDeviceFromHost;
	using MemoryBlock<T>::UpdateHostFromDevice;

	bool IsAllocated_CPU() const
	{
		return this->is_allocated_for_CPU;
	};


	/** Size of the image in pixels. */
	Vector2<int> dimensions;

	/** Initialize an empty image of the given size, either
	on CPU only or on both CPU and GPU.
	*/
	Image(Vector2<int> noDims, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(noDims.x * noDims.y, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->dimensions = noDims;
	}

	Image(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->dimensions = Vector2<int>(0, 0);
	}

	Image(Vector2<int> noDims, MemoryDeviceType memoryType)
			: MemoryBlock<T>(noDims.x * noDims.y, memoryType) {
		this->dimensions = noDims;
	}

	/** Resize an image, losing all old image data.
	Essentially any previously allocated data is
	released, new memory is allocated.
	*/
	void ChangeDims(Vector2<int> newDims, bool forceReallocation = true) {
		MemoryBlock<T>::Resize(newDims.x * newDims.y, forceReallocation);
		dimensions = newDims;
	}

	void SetFrom(const Image<T>& source, MemoryCopyDirection memory_copy_direction) {
		ChangeDims(source.dimensions);
		MemoryBlock<T>::SetFrom(source, memory_copy_direction);
	}

	void Swap(Image<T>& rhs) {
		MemoryBlock<T>::Swap(rhs);
		std::swap(this->dimensions, rhs.dimensions);
	}

	template <typename TMask>
	void ApplyMask(const Image<TMask>& maskImage, T blankElement);

	friend bool operator==(const Image<T>& rhs, const Image<T>& lhs) {
		if (rhs.dimensions != lhs.dimensions || rhs.is_allocated_for_CPU != lhs.is_allocated_for_CPU ||
		    rhs.is_allocated_for_CUDA != lhs.is_allocated_for_CUDA) {
			return false;
		}
		if (rhs.is_allocated_for_CPU) {
			for (int iElement = 0; iElement < rhs.element_count; iElement++) {
				T rhsElement = rhs.GetElement(iElement, MEMORYDEVICE_CPU);
				T lhsElement = lhs.GetElement(iElement, MEMORYDEVICE_CPU);
				if (rhsElement != lhsElement) {
					return false;
				}
			}
		} else {
			for (int iElement = 0; iElement < rhs.element_count; iElement++) {
				T rhsElement = rhs.GetElement(iElement, MEMORYDEVICE_CUDA);
				T lhsElement = lhs.GetElement(iElement, MEMORYDEVICE_CUDA);
				if (rhsElement != lhsElement) {
					return false;
				}
			}
		}
		return true;
	}

	friend bool operator!=(const Image<T>& rhs, const Image<T>& lhs) {
		return !(rhs == lhs);
	}

	// Suppress the default copy constructor and assignment operator
	Image(const Image&);
	Image& operator=(const Image&);

private:
	/** Expose protected MemoryBlock<T> member variables. */
	using MemoryBlock<T>::element_count;
}; // class Image<T>


template<typename T>
template <typename TMask>
void Image<T>::ApplyMask(const Image<TMask>& maskImage, T blankElement) {
	if (!this->is_allocated_for_CPU || !maskImage.IsAllocated_CPU()) {
		DIEWITHEXCEPTION("Cannot apply mask, either mask or source image or both are not allocated for CPU.");
	}
	if (this->dimensions != maskImage.dimensions) {
		DIEWITHEXCEPTION("Source and mask image dimensions must match.");
	}
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(maskImage, blankElement)
#endif
	for (int iElement = 0; iElement < maskImage.size(); iElement++) {
		if(!maskImage.GetElement(iElement,MEMORYDEVICE_CPU)){
			this->data_cpu[iElement] = blankElement;
		}
	}
}

}//namespace ORUtils


#endif
