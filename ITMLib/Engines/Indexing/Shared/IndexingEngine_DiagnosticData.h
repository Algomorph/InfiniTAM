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
#include "../../../Utils/Geometry/Segment.h"
#include "../../../../ORUtils/MemoryBlockPersistence.h"

namespace ITMLib {

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct IndexingDiagnosticData;

template<MemoryDeviceType TMemoryDeviceType>
struct IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType> {
public: // member variables
	// ** indexing engine diagnostics **
	BoolImage surface1_point_mask;
	BoolImage surface2_point_mask;
	Float3Image surface1_point_cloud;
	Float3Image surface2_point_cloud;
	Float3Image march_endpoint1_point_cloud;
	Float3Image march_endpoint2_point_cloud;
	ORUtils::MemoryBlock<Vector2i> depth_image_dimensions;


	struct DataDevice {
		DataDevice() = default;

		DataDevice(IndexingDiagnosticData<VoxelBlockHash, TMemoryDeviceType>& parent) :
				surface1_point_mask_device(parent.surface1_point_mask.GetData(TMemoryDeviceType)),
				surface2_point_mask_device(parent.surface2_point_mask.GetData(TMemoryDeviceType)),
				surface1_point_cloud_device(parent.surface1_point_cloud.GetData(TMemoryDeviceType)),
				surface2_point_cloud_device(parent.surface2_point_cloud.GetData(TMemoryDeviceType)),
				march_endpoint1_point_cloud_device(parent.march_endpoint1_point_cloud.GetData(TMemoryDeviceType)),
				march_endpoint2_point_cloud_device(parent.march_endpoint2_point_cloud.GetData(TMemoryDeviceType)),
				depth_image_dimensions_device(parent.depth_image_dimensions.GetData(TMemoryDeviceType)) {}

		bool* surface1_point_mask_device;
		bool* surface2_point_mask_device;
		Vector3f* surface1_point_cloud_device;
		Vector3f* surface2_point_cloud_device;
		Vector3f* march_endpoint1_point_cloud_device;
		Vector3f* march_endpoint2_point_cloud_device;
		Vector2i* depth_image_dimensions_device;
		_DEVICE_WHEN_AVAILABLE_
		inline void SetPixelData(const int x, const int y, bool has_surface1_point, bool has_surface2_point,
		                         const Vector4f& surface1_point, const Vector4f& surface2_point,
		                         const Segment& march_segment_blocks) {
			const int element_ix = x + y * depth_image_dimensions_device->width;
			surface1_point_mask_device[element_ix] = has_surface1_point;
			surface2_point_mask_device[element_ix] = has_surface2_point;
			surface1_point_cloud_device[element_ix] = surface1_point.toVector3();
			surface2_point_cloud_device[element_ix] = surface2_point.toVector3();
			march_endpoint1_point_cloud_device[element_ix] = march_segment_blocks.origin;
			march_endpoint2_point_cloud_device[element_ix] = march_segment_blocks.destination();
		}
	};

	ORUtils::MemoryBlock<DataDevice> data_device;


public: // member functions

	IndexingDiagnosticData(const Vector2i& depth_image_dimensions) :
			surface1_point_mask(depth_image_dimensions, TMemoryDeviceType),
			surface2_point_mask(depth_image_dimensions, TMemoryDeviceType),
			surface1_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			surface2_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			march_endpoint1_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			march_endpoint2_point_cloud(depth_image_dimensions, TMemoryDeviceType),
			depth_image_dimensions(1, true, true),
			data_device(1, true, true) {
		*this->depth_image_dimensions.GetData(MEMORYDEVICE_CPU) = depth_image_dimensions;
		this->depth_image_dimensions.UpdateDeviceFromHost();
		*this->data_device.GetData(MEMORYDEVICE_CPU) = DataDevice(*this);
		this->data_device.UpdateDeviceFromHost();
	};

	IndexingDiagnosticData() : IndexingDiagnosticData(Vector2i(0, 0)) {};

	void SaveToDisk(std::string output_folder_path) {
		ORUtils::OStreamWrapper file(output_folder_path + "/voxel_block_hash_diagnostic_data.dat", true);
		Vector2i& image_dimensions = *this->depth_image_dimensions.GetData(MEMORYDEVICE_CPU);
		const int num_bool_layers = 2;
		file.OStream().write(reinterpret_cast<const char*>(&num_bool_layers), sizeof(int));
		file.OStream().write(reinterpret_cast<const char*>(&image_dimensions.height), sizeof(int));
		file.OStream().write(reinterpret_cast<const char*>(&image_dimensions.width), sizeof(int));
		ORUtils::MemoryBlockPersistence::SaveImageData(file, surface1_point_mask);
		ORUtils::MemoryBlockPersistence::SaveImageData(file, surface2_point_mask);

		const int num_float_layers = 4;
		file.OStream().write(reinterpret_cast<const char*>(&num_float_layers), sizeof(int));
		file.OStream().write(reinterpret_cast<const char*>(&image_dimensions.height), sizeof(int));
		file.OStream().write(reinterpret_cast<const char*>(&image_dimensions.width), sizeof(int));
		const int num_channels = 3;
		file.OStream().write(reinterpret_cast<const char*>(&num_channels), sizeof(int));

		//_DEBUG alloc
//		Float3Image copy(image_dimensions, MEMORYDEVICE_CPU);
//		copy.SetFrom(surface1_point_cloud, CUDA_TO_CPU);
//		Vector3f datum1 = copy.GetData(MEMORYDEVICE_CPU)[319 + 385 * 640];
//		copy.SetFrom(surface2_point_cloud, CUDA_TO_CPU);
//		Vector3f datum2 = copy.GetData(MEMORYDEVICE_CPU)[319 + 385 * 640];
//		printf("Surface points for px 319, 385 while saving: %f, %f, %f to %f, %f, %f\n",
//		       datum1[0], datum1[1], datum1[2],
//		       datum2[0], datum2[1], datum2[2]);
//
//		copy.SetFrom(march_endpoint1_point_cloud, CUDA_TO_CPU);
//		datum1 = copy.GetData(MEMORYDEVICE_CPU)[319 + 385 * 640];
//		copy.SetFrom(march_endpoint2_point_cloud, CUDA_TO_CPU);
//		datum2 = copy.GetData(MEMORYDEVICE_CPU)[319 + 385 * 640];
//		std::cout << "Size of Vector3f: " << sizeof(Vector3f) << std::endl;
//		printf("March segment for px 319, 385 while saving: %f, %f, %f to %f, %f, %f\n",
//		       datum1[0], datum1[1], datum1[2],
//		       datum2[0], datum2[1], datum2[2]);

		ORUtils::MemoryBlockPersistence::SaveImageData(file, surface1_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImageData(file, surface2_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImageData(file, march_endpoint1_point_cloud);
		ORUtils::MemoryBlockPersistence::SaveImageData(file, march_endpoint2_point_cloud);
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
			*this->data_device.GetData(MEMORYDEVICE_CPU) = DataDevice(*this);
			this->data_device.UpdateDeviceFromHost();
			return true;
		}
		return false;
	}
};

} // namespace ITMLib