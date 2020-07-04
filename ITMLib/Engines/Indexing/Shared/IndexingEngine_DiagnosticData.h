//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/15/20.
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

#include "../../../Utils/ImageTypes.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Utils/Geometry/Segment.h"
#include "../../../../ORUtils/MemoryBlockPersistence.h"

namespace ITMLib {

namespace internal{
	static constexpr int max_pixel_block_count = 16;
	typedef ORUtils::VectorX<Vector3s, max_pixel_block_count> PixelBlockAllocationRecord;
}


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct IndexingDiagnosticData;

template<MemoryDeviceType TMemoryDeviceType>
struct IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType> {
public: // instance variables

	// ** indexing engine diagnostics **
	BoolImage surface1_point_mask;
	BoolImage surface2_point_mask;
	Float3Image surface1_point_cloud;
	Float3Image surface2_point_cloud;
	Float3Image march_endpoint1_point_cloud;
	Float3Image march_endpoint2_point_cloud;

	ORUtils::Image<internal::PixelBlockAllocationRecord> pixel_block_allocations;
	IntImage pixel_block_allocation_counts;

	ORUtils::MemoryBlock<Vector2i> depth_image_dimensions;



	DECLARE_ATOMIC(int, pixel_block_allocation_record_count);

	struct DataDevice {
		DataDevice() = default;

		bool* surface1_point_mask_device;
		bool* surface2_point_mask_device;
		Vector3f* surface1_point_cloud_device;
		Vector3f* surface2_point_cloud_device;
		Vector3f* march_endpoint1_point_cloud_device;
		Vector3f* march_endpoint2_point_cloud_device;

		internal::PixelBlockAllocationRecord* pixel_block_allocations_device;
		int* pixel_block_allocation_counts_device;

		Vector2i* depth_image_dimensions_device;

		DataDevice(IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType>& parent) :
				surface1_point_mask_device(parent.surface1_point_mask.GetData(TMemoryDeviceType)),
				surface2_point_mask_device(parent.surface2_point_mask.GetData(TMemoryDeviceType)),
				surface1_point_cloud_device(parent.surface1_point_cloud.GetData(TMemoryDeviceType)),
				surface2_point_cloud_device(parent.surface2_point_cloud.GetData(TMemoryDeviceType)),
				march_endpoint1_point_cloud_device(parent.march_endpoint1_point_cloud.GetData(TMemoryDeviceType)),
				march_endpoint2_point_cloud_device(parent.march_endpoint2_point_cloud.GetData(TMemoryDeviceType)),

				pixel_block_allocations_device(parent.pixel_block_allocations.GetData(TMemoryDeviceType)),
				pixel_block_allocation_counts_device(parent.pixel_block_allocation_counts.GetData(TMemoryDeviceType)),

				depth_image_dimensions_device(parent.depth_image_dimensions.GetData(TMemoryDeviceType)) {}


		_DEVICE_WHEN_AVAILABLE_
		inline void SetPixelData(const int x, const int y, bool has_surface1_point, bool has_surface2_point,
		                         const Vector4f& surface1_point, const Vector4f& surface2_point,
		                         const Segment& march_segment_blocks,
		                         const internal::PixelBlockAllocationRecord& pixel_blocks,
		                         int pixel_block_count) {
			const int pixel_index = x + y * depth_image_dimensions_device->width;
			surface1_point_mask_device[pixel_index] = has_surface1_point;
			surface2_point_mask_device[pixel_index] = has_surface2_point;
			surface1_point_cloud_device[pixel_index] = surface1_point.toVector3();
			surface2_point_cloud_device[pixel_index] = surface2_point.toVector3();
			march_endpoint1_point_cloud_device[pixel_index] = march_segment_blocks.origin;
			march_endpoint2_point_cloud_device[pixel_index] = march_segment_blocks.destination();

			pixel_block_allocations_device[pixel_index] = pixel_blocks;
			pixel_block_allocation_counts_device[pixel_index] = pixel_block_count;
		}
		_DEVICE_WHEN_AVAILABLE_
		inline void GetBlockRecordForPixel(const int x, const int y, int& pixel_block_count, internal::PixelBlockAllocationRecord& pixel_blocks){
			const int pixel_index = x + y * depth_image_dimensions_device->width;
			pixel_block_count =  pixel_block_allocation_counts_device[pixel_index];
			pixel_blocks = pixel_block_allocations_device[pixel_index];
		}
	};

	ORUtils::MemoryBlock<DataDevice> data_device;


public: // instance functions

	IndexingDiagnosticData(const Vector2i& depth_image_dimensions) :
			surface1_point_mask(depth_image_dimensions, TMemoryDeviceType),
			surface2_point_mask(depth_image_dimensions, TMemoryDeviceType),
			surface1_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			surface2_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			march_endpoint1_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			march_endpoint2_point_cloud(depth_image_dimensions, TMemoryDeviceType),

			pixel_block_allocations(depth_image_dimensions, TMemoryDeviceType),
			pixel_block_allocation_counts(depth_image_dimensions, TMemoryDeviceType),

			depth_image_dimensions(1, true, true),
			data_device(1, true, true) {
		INITIALIZE_ATOMIC(int, pixel_block_allocation_record_count, 0);
		*this->depth_image_dimensions.GetData(MEMORYDEVICE_CPU) = depth_image_dimensions;
		this->depth_image_dimensions.UpdateDeviceFromHost();
		*this->data_device.GetData(MEMORYDEVICE_CPU) = DataDevice(*this);
		this->data_device.UpdateDeviceFromHost();
	};

	IndexingDiagnosticData() : IndexingDiagnosticData(Vector2i(0, 0)) {};

	~IndexingDiagnosticData(){
		CLEAN_UP_ATOMIC(pixel_block_allocation_record_count);
	}

	void SaveToDisk(std::string output_folder_path) {
		ORUtils::OStreamWrapper file(output_folder_path + "/voxel_block_hash_diagnostic_data.dat", true);
		const int num_bool_layers = 2;
		file.OStream().write(reinterpret_cast<const char*>(&num_bool_layers), sizeof(int));
		ORUtils::MemoryBlockPersistence::SaveImage(file, surface1_point_mask);
		ORUtils::MemoryBlockPersistence::SaveImage(file, surface2_point_mask);

		const int num_float_layers = 4;
		file.OStream().write(reinterpret_cast<const char*>(&num_float_layers), sizeof(int));

		ORUtils::MemoryBlockPersistence::SaveImage(file, surface1_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImage(file, surface2_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImage(file, march_endpoint1_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImage(file, march_endpoint2_point_cloud);

		ORUtils::MemoryBlockPersistence::SaveImage(file, pixel_block_allocations);
		ORUtils::MemoryBlockPersistence::SaveImage(file, pixel_block_allocation_counts);
	}

	bool PrepareForFrame(const Vector2i& depth_image_dimensions) {
		if (*this->depth_image_dimensions.GetData(MEMORYDEVICE_CPU) != depth_image_dimensions) {
			*this->depth_image_dimensions.GetData(MEMORYDEVICE_CPU) = depth_image_dimensions;
			this->depth_image_dimensions.UpdateDeviceFromHost();

			this->surface1_point_mask.ChangeDims(depth_image_dimensions, false);
			this->surface2_point_mask.ChangeDims(depth_image_dimensions, false);
			this->surface1_point_cloud.ChangeDims(depth_image_dimensions, false);
			this->surface2_point_cloud.ChangeDims(depth_image_dimensions, false);
			this->march_endpoint1_point_cloud.ChangeDims(depth_image_dimensions, false);
			this->march_endpoint2_point_cloud.ChangeDims(depth_image_dimensions, false);

			this->pixel_block_allocations.ChangeDims(depth_image_dimensions, false);
			this->pixel_block_allocation_counts.ChangeDims(depth_image_dimensions, false);

			*this->data_device.GetData(MEMORYDEVICE_CPU) = DataDevice(*this);
			this->data_device.UpdateDeviceFromHost();
			return true;
		}
		this->pixel_block_allocation_counts.Clear();
		return false;
	}
};

} // namespace ITMLib