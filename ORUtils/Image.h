//  ================================================================
//  Created by Gregory Kramida on 5/24/20.
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

// Credits: Inspired by ORUtils/Image.h in original InfiniTAM (https://github.com/victorprad/InfiniTAM/) and ORUtils (https://github.com/carlren/ORUtils)

#pragma once

#include "MemoryBlock.h"
#include "Vector.h"

#ifndef __METALC__

namespace ORUtils {
/**
 * \brief 2D raster images.
 * \tparam T pixel type.
 * */
template<typename T>
class Image : private MemoryBlock<T> {
private: // member variables
	using MemoryBlock<T>::element_count;
public: // member functions
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

	bool IsAllocated_CPU() const {
		return this->is_allocated_for_CPU;
	};

	/** Image width & height, in pixels. */
	Vector2<int> dimensions;

	/**
	 * Initialize an empty image of the given size, either on CPU only, CUDA device only, or both CPU and CUDA.
	 * */
	Image(Vector2<int> dimensions, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(dimensions.x * dimensions.y, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->dimensions = dimensions;
	}

	Image(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible) {
		this->dimensions = Vector2<int>(0, 0);
	}

	Image(Vector2<int> dimensions, MemoryDeviceType memory_device_type)
			: MemoryBlock<T>(dimensions.x * dimensions.y, memory_device_type) {
		this->dimensions = dimensions;
	}

	Image(const Image& other, MemoryDeviceType memory_device_type) : Image(other.dimensions, memory_device_type) {
		this->SetFrom(other);
	}

	Image(const Image& other) : MemoryBlock<T>(other), dimensions(other.dimensions) {}

	Image(Image&& other) : MemoryBlock<T>(other), dimensions(other.dimensions) {
		other.dimensions = Vector2<int>();
	}

	/**
	 * Resize the image, losing all old image data if force_reallocation is set to true.
	 */
	void ChangeDims(Vector2<int> dimensions, bool force_reallocation = true) {
		MemoryBlock<T>::Resize(dimensions.x * dimensions.y, force_reallocation);
		this->dimensions = dimensions;
	}

	void SetFrom(const Image<T>& source, MemoryCopyDirection memory_copy_direction) {
		ChangeDims(source.dimensions);
		MemoryBlock<T>::SetFrom(source, memory_copy_direction);
	}

	void Swap(Image<T>& rhs) {
		using std::swap;
		MemoryBlock<T>::Swap(rhs);
		swap(this->dimensions, rhs.dimensions);
	}

	friend void swap(Image<T>& lhs, Image<T>& rhs) { // nothrow
		lhs.Swap(rhs);
	}

	template<typename TMask>
	void ApplyMask(const Image<TMask>& mask_image, T blank_element);

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
		}
		if (rhs.is_allocated_for_CUDA) {
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

	Image& operator=(Image other) {
		swap(*this, other);
		return *this;
	}
};


template<typename T>
template<typename TMask>
void Image<T>::ApplyMask(const Image<TMask>& mask_image, T blank_element) {
	if (!this->is_allocated_for_CPU || !mask_image.IsAllocated_CPU()) {
		DIEWITHEXCEPTION("Cannot apply mask, either mask or source image or both are not allocated for CPU.");
	}
	if (this->dimensions != mask_image.dimensions) {
		DIEWITHEXCEPTION("Source and mask image dimensions must match.");
	}
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(mask_image, blank_element)
#endif
	for (int iElement = 0; iElement < mask_image.size(); iElement++) {
		if (!mask_image.GetElement(iElement, MEMORYDEVICE_CPU)) {
			this->data_cpu[iElement] = blank_element;
		}
	}
}

} // namespace ORUtils


#endif
