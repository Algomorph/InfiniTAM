//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include <boost/test/test_tools.hpp>
#include "TestUtils.h"
#include "TestUtils.tpp"


#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Utils/FileIO/SceneLogger.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/SceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"

using namespace ITMLib;

template void GenerateTestScene_CPU<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>* scene);
template void GenerateTestScene_CPU<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>* scene);
#ifndef COMPILE_WITHOUT_CUDA
template void GenerateTestScene_CUDA<TSDFVoxel, VoxelBlockHash>(ITMVoxelVolume<TSDFVoxel, VoxelBlockHash>* scene);
template void GenerateTestScene_CUDA<TSDFVoxel, PlainVoxelArray>(ITMVoxelVolume<TSDFVoxel, PlainVoxelArray>* scene);
#endif

template void simulateVoxelAlteration<TSDFVoxel>(TSDFVoxel& voxel, float newSdfValue);
template void simulateRandomVoxelAlteration<TSDFVoxel>(TSDFVoxel& voxel);
template void simulateRandomVoxelAlteration<WarpVoxel>(WarpVoxel& voxel);


//have nothing to prep for PVA -- everything gets copied off the disk exactly
template<>
void PrepareVoxelVolumeForLoading(VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume) {}

template<>
void PrepareVoxelVolumeForLoading(VoxelVolume<WarpVoxel, PlainVoxelArray>* volume) {}

//for VBH, the scene has to be reset before loading
template<>
void PrepareVoxelVolumeForLoading(VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume) {
	volume->Reset();
}
template<>
void PrepareVoxelVolumeForLoading(VoxelVolume<WarpVoxel, VoxelBlockHash>* volume) {
	volume->Reset();
}

template<>
PlainVoxelArray::InitializationParameters GetFrame17PartialIndexParameters<PlainVoxelArray>() {
	return {Vector3i(80, 96, 144), Vector3i(-64, -24, 168)};
}

template<>
VoxelBlockHash::InitializationParameters GetFrame17PartialIndexParameters<VoxelBlockHash>() {
	return {0x800, 0x20000};
}

template<>
typename PlainVoxelArray::InitializationParameters GetStandard512IndexParameters<PlainVoxelArray>() {
	return {Vector3i(512), Vector3i(-256, -256, 0)};
}

template<>
typename VoxelBlockHash::InitializationParameters GetStandard512IndexParameters<VoxelBlockHash>() {
	return {0x40000, 0x20000};
}


template<>
typename PlainVoxelArray::InitializationParameters GetStandard128IndexParameters<PlainVoxelArray>() {
	return {Vector3i(128), Vector3i(-64, -64, 0)};
}

template<>
typename VoxelBlockHash::InitializationParameters GetStandard128IndexParameters<VoxelBlockHash>() {
	return {0x2000, 0x20000};
}

// FIXME: see TODO in header
//template ITMVoxelVolume<TSDFVoxel, PlainVoxelArray> loadVolume<TSDFVoxel, PlainVoxelArray>(
//                                                 const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 PlainVoxelArray::InitializationParameters initializationParameters,
//                                                 configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<TSDFVoxel, VoxelBlockHash> loadVolume<TSDFVoxel, VoxelBlockHash>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 VoxelBlockHash::InitializationParameters initializationParameters,
//                                                 configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<WarpVoxel, PlainVoxelArray> loadVolume<WarpVoxel, PlainVoxelArray>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                          PlainVoxelArray::InitializationParameters initializationParameters,
//                                                          configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<WarpVoxel, VoxelBlockHash> loadVolume<WarpVoxel, VoxelBlockHash>(
//                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                         VoxelBlockHash::InitializationParameters initializationParameters,
//                                                         configuration::SwappingMode swapping_mode);

template void loadVolume<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
                                                     const std::string& path, MemoryDeviceType memoryDeviceType,
                                                     PlainVoxelArray::InitializationParameters initializationParameters,
                                                     configuration::SwappingMode swappingMode);
template void loadVolume<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
                                                    const std::string& path, MemoryDeviceType memoryDeviceType,
                                                    VoxelBlockHash::InitializationParameters initializationParameters,
                                                    configuration::SwappingMode swappingMode);
template void loadVolume<WarpVoxel, PlainVoxelArray>(VoxelVolume<WarpVoxel, PlainVoxelArray>** volume,
                                                     const std::string& path, MemoryDeviceType memoryDeviceType,
                                                     PlainVoxelArray::InitializationParameters initializationParameters,
                                                     configuration::SwappingMode swappingMode);
template void loadVolume<WarpVoxel, VoxelBlockHash>(VoxelVolume<WarpVoxel, VoxelBlockHash>** volume,
                                                    const std::string& path, MemoryDeviceType memoryDeviceType,
                                                    VoxelBlockHash::InitializationParameters initializationParameters,
                                                    configuration::SwappingMode swappingMode);

void
updateView(ITMView** view, const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
           const std::string& calibration_path, MemoryDeviceType memoryDevice) {
	static ViewBuilder* viewBuilder_CPU = nullptr;
	static ViewBuilder* viewBuilder_CUDA = nullptr;
	ViewBuilder* viewBuilderToUse;
	switch (memoryDevice) {
		case MEMORYDEVICE_CPU:
			if (viewBuilder_CPU == nullptr)
				viewBuilder_CPU = ViewBuilderFactory::MakeViewBuilder(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CPU;
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			if (viewBuilder_CUDA == nullptr)
				viewBuilder_CUDA = ITMViewBuilderFactory::MakeViewBuilder(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CUDA;
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Attmpted to update CUDA view while build without CUDA support, aborting.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("unsupported memory device type!");
	}

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	auto* mask = new ITMUCharImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, color_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(depth, depth_path.c_str()));
	BOOST_REQUIRE(ReadImageFromFile(mask, mask_path.c_str()));
	rgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
	depth->ApplyMask(*mask, 0);
	viewBuilderToUse->UpdateView(view, rgb, depth, false, false, false, true);
	delete rgb;
	delete depth;
	delete mask;
}

template
void buildSdfVolumeFromImage<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
                                                         ITMView** view,
                                                         const std::string& depth_path, const std::string& color_path,
                                                         const std::string& mask_path,
                                                         const std::string& calibration_path,
                                                         MemoryDeviceType memoryDevice,
                                                         PlainVoxelArray::InitializationParameters initializationParameters,
                                                         configuration::SwappingMode swappingMode,
                                                         bool useBilateralFilter);

template
void buildSdfVolumeFromImage<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
                                                         const std::string& depth_path, const std::string& color_path,
                                                         const std::string& mask_path,
                                                         const std::string& calibration_path,
                                                         MemoryDeviceType memoryDevice,
                                                         PlainVoxelArray::InitializationParameters initializationParameters,
                                                         configuration::SwappingMode swappingMode,
                                                         bool useBilateralFilter);

template
void buildSdfVolumeFromImage<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
                                                        ITMView** view,
                                                        const std::string& depth_path, const std::string& color_path,
                                                        const std::string& mask_path,
                                                        const std::string& calibration_path,
                                                        MemoryDeviceType memoryDevice,
                                                        VoxelBlockHash::InitializationParameters initializationParameters,
                                                        configuration::SwappingMode swappingMode,
                                                        bool useBilateralFilter);

template
void buildSdfVolumeFromImage<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
                                                        const std::string& depth_path, const std::string& color_path,
                                                        const std::string& mask_path,
                                                        const std::string& calibration_path,
                                                        MemoryDeviceType memoryDevice,
                                                        VoxelBlockHash::InitializationParameters initializationParameters,
                                                        configuration::SwappingMode swappingMode,
                                                        bool useBilateralFilter);

template
void initializeVolume<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
                                                 VoxelBlockHash::InitializationParameters initializationParameters,
                                                 MemoryDeviceType memoryDevice,
                                                 configuration::SwappingMode swappingMode);
template
void initializeVolume<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
                                                  PlainVoxelArray::InitializationParameters initializationParameters,
                                                  MemoryDeviceType memoryDevice,
                                                  configuration::SwappingMode swappingMode);
template
void initializeVolume<WarpVoxel, VoxelBlockHash>(VoxelVolume<WarpVoxel, VoxelBlockHash>** volume,
                                                 VoxelBlockHash::InitializationParameters initializationParameters,
                                                 MemoryDeviceType memoryDevice,
                                                 configuration::SwappingMode swappingMode);
template
void initializeVolume<WarpVoxel, PlainVoxelArray>(VoxelVolume<WarpVoxel, PlainVoxelArray>** volume,
                                                  PlainVoxelArray::InitializationParameters initializationParameters,
                                                  MemoryDeviceType memoryDevice,
                                                  configuration::SwappingMode swappingMode);
