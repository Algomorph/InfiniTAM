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

	/** Expose public MemoryBlock<T> member variables. */
	using MemoryBlock<T>::element_count;


	/** Expose public MemoryBlock<T> member functions. */
	using MemoryBlock<T>::Clear;
	using MemoryBlock<T>::GetData;
	using MemoryBlock<T>::GetElement;
#ifdef COMPILE_WITH_METAL
	using MemoryBlock<T>::GetMetalBuffer();
#endif
	using MemoryBlock<T>::UpdateDeviceFromHost;
	using MemoryBlock<T>::UpdateHostFromDevice;

	bool IsAllocated_CPU() const
	{
		return this->isAllocated_CPU;
	};


	/** Size of the image in pixels. */
	Vector2<int> noDims;

	/** Initialize an empty image of the given size, either
	on CPU only or on both CPU and GPU.
	*/
	Image(Vector2<int> noDims, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(noDims.x * noDims.y, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->noDims = noDims;
	}

	Image(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->noDims = Vector2<int>(0, 0);
	}

	Image(Vector2<int> noDims, MemoryDeviceType memoryType)
			: MemoryBlock<T>(noDims.x * noDims.y, memoryType) {
		this->noDims = noDims;
	}

	/** Resize an image, losing all old image data.
	Essentially any previously allocated data is
	released, new memory is allocated.
	*/
	void ChangeDims(Vector2<int> newDims, bool forceReallocation = true) {
		MemoryBlock<T>::Resize(newDims.x * newDims.y, forceReallocation);
		noDims = newDims;
	}

	void SetFrom(const Image<T>* source, MemoryCopyDirection memoryCopyDirection) {
		ChangeDims(source->noDims);
		MemoryBlock<T>::SetFrom(source, memoryCopyDirection);
	}

	void Swap(Image<T>& rhs) {
		MemoryBlock<T>::Swap(rhs);
		std::swap(this->noDims, rhs.noDims);
	}

	template <typename TMask>
	void ApplyMask(const Image<TMask>& maskImage, T blankElement);

	friend bool operator==(const Image<T>& rhs, const Image<T>& lhs) {
		if (rhs.noDims != lhs.noDims || rhs.isAllocated_CPU != lhs.isAllocated_CPU ||
		    rhs.isAllocated_CUDA != lhs.isAllocated_CUDA) {
			return false;
		}
		if (rhs.isAllocated_CPU) {
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
};


template<typename T>
template <typename TMask>
void Image<T>::ApplyMask(const Image<TMask>& maskImage, T blankElement) {
	if (!this->isAllocated_CPU || !maskImage.IsAllocated_CPU()) {
		DIEWITHEXCEPTION("Cannot apply mask, either mask or source image or both are not allocated for CPU.");
	}
	if (this->noDims != maskImage.noDims) {
		DIEWITHEXCEPTION("Source and mask image dimensions must match.");
	}
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(maskImage, blankElement)
#endif
	for (int iElement = 0; iElement < maskImage.element_count; iElement++) {
		if(!maskImage.GetElement(iElement,MEMORYDEVICE_CPU)){
			this->data_cpu[iElement] = blankElement;
		}
	}
}
}//namespace ORUtils


#endif
