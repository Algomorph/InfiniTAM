//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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
#include "TestUtilities.h"
#include "TestUtilities.tpp"


#include "../../ITMLib/Utils/Configuration.h"
#include "../../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../../ITMLib/Engines/Telemetry/VolumeSequenceRecorder.h"
#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"

using namespace ITMLib;

namespace test_utilities {

template<>
std::string IndexString<VoxelBlockHash>() {
	return "VBH";
}

template<>
std::string IndexString<PlainVoxelArray>() {
	return "PVA";
}

template<>
std::string DeviceString<MEMORYDEVICE_CPU>() {
	return "CPU";
}

template<>
std::string DeviceString<MEMORYDEVICE_CUDA>() {
	return "CUDA";
}

template void GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU, TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume);
template void GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU, TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume);

template void SimulateVoxelAlteration<TSDFVoxel>(TSDFVoxel& voxel, float newSdfValue);
template void SimulateRandomVoxelAlteration<TSDFVoxel>(TSDFVoxel& voxel);
template void SimulateRandomVoxelAlteration<WarpVoxel>(WarpVoxel& voxel);


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
//template ITMVoxelVolume<TSDFVoxel, PlainVoxelArray> LoadVolume<TSDFVoxel, PlainVoxelArray>(
//                                                 const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 PlainVoxelArray::InitializationParameters_Fr16andFr17 initializationParameters,
//                                                 configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<TSDFVoxel, VoxelBlockHash> LoadVolume<TSDFVoxel, VoxelBlockHash>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                 VoxelBlockHash::InitializationParameters_Fr16andFr17 initializationParameters,
//                                                 configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<WarpVoxel, PlainVoxelArray> LoadVolume<WarpVoxel, PlainVoxelArray>(const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                          PlainVoxelArray::InitializationParameters_Fr16andFr17 initializationParameters,
//                                                          configuration::SwappingMode swapping_mode);
//template ITMVoxelVolume<WarpVoxel, VoxelBlockHash> LoadVolume<WarpVoxel, VoxelBlockHash>(
//                                                         const std::string& path, MemoryDeviceType memoryDeviceType,
//                                                         VoxelBlockHash::InitializationParameters_Fr16andFr17 initializationParameters,
//                                                         configuration::SwappingMode swapping_mode);

template void LoadVolume<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
                                                     const std::string& path, MemoryDeviceType memoryDeviceType,
                                                     PlainVoxelArray::InitializationParameters initializationParameters,
                                                     configuration::SwappingMode swappingMode);
template void LoadVolume<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
                                                    const std::string& path, MemoryDeviceType memoryDeviceType,
                                                    VoxelBlockHash::InitializationParameters initializationParameters,
                                                    configuration::SwappingMode swappingMode);
template void LoadVolume<WarpVoxel, PlainVoxelArray>(VoxelVolume<WarpVoxel, PlainVoxelArray>** volume,
                                                     const std::string& path, MemoryDeviceType memoryDeviceType,
                                                     PlainVoxelArray::InitializationParameters initializationParameters,
                                                     configuration::SwappingMode swappingMode);
template void LoadVolume<WarpVoxel, VoxelBlockHash>(VoxelVolume<WarpVoxel, VoxelBlockHash>** volume,
                                                    const std::string& path, MemoryDeviceType memoryDeviceType,
                                                    VoxelBlockHash::InitializationParameters initializationParameters,
                                                    configuration::SwappingMode swappingMode);

static ViewBuilder* viewBuilder_CPU = nullptr;
static ViewBuilder* viewBuilder_CUDA = nullptr;

void
UpdateView(View** view, const std::string& depth_path, const std::string& color_path,
           const std::string& calibration_path, MemoryDeviceType memoryDevice) {

	ViewBuilder* viewBuilderToUse;
	switch (memoryDevice) {
		case MEMORYDEVICE_CPU:
			if (viewBuilder_CPU == nullptr)
				viewBuilder_CPU = ViewBuilderFactory::Build(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CPU;
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			if (viewBuilder_CUDA == nullptr)
				viewBuilder_CUDA = ViewBuilderFactory::Build(calibration_path, memoryDevice);
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
	ReadImageFromFile(*rgb, color_path.c_str());
	ReadImageFromFile(*depth, depth_path.c_str());
	viewBuilderToUse->UpdateView(view, rgb, depth, false, false, false, true);
	delete rgb;
	delete depth;
	delete mask;
}

void
UpdateView(View** view, const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
           const std::string& calibration_path, MemoryDeviceType memoryDevice) {

	ViewBuilder* viewBuilderToUse;
	switch (memoryDevice) {
		case MEMORYDEVICE_CPU:
			if (viewBuilder_CPU == nullptr)
				viewBuilder_CPU = ViewBuilderFactory::Build(calibration_path, memoryDevice);
			viewBuilderToUse = viewBuilder_CPU;
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			if (viewBuilder_CUDA == nullptr)
				viewBuilder_CUDA = ViewBuilderFactory::Build(calibration_path, memoryDevice);
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
	ReadImageFromFile(*rgb, color_path.c_str());
	ReadImageFromFile(*depth, depth_path.c_str());
	ReadImageFromFile(*mask, mask_path.c_str());
	rgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
	depth->ApplyMask(*mask, 0);
	viewBuilderToUse->UpdateView(view, rgb, depth, false, false, false, true);
	delete rgb;
	delete depth;
	delete mask;
}


template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
		View** view,
		const std::string& depth_path, const std::string& color_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
		const std::string& depth_path, const std::string& color_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
		View** view,
		const std::string& depth_path, const std::string& color_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
		const std::string& depth_path, const std::string& color_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);


template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
		View** view,
		const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume,
		const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
		View** view,
		const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);

template
void BuildSdfVolumeFromImage_NearSurfaceAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume,
		const std::string& depth_path, const std::string& color_path, const std::string& mask_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);


template
void BuildSdfVolumeFromImage_SurfaceSpanAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume1,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume2,
		View** view,
		const std::string& depth1_path, const std::string& color1_path, const std::string& mask1_path,
		const std::string& depth2_path, const std::string& color2_path, const std::string& mask2_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);
template
void BuildSdfVolumeFromImage_SurfaceSpanAllocation<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume1,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume2,
		const std::string& depth1_path, const std::string& color1_path, const std::string& mask1_path,
		const std::string& depth2_path, const std::string& color2_path, const std::string& mask2_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		PlainVoxelArray::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);
template
void BuildSdfVolumeFromImage_SurfaceSpanAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume1,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume2,
		View** view,
		const std::string& depth1_path, const std::string& color1_path, const std::string& mask1_path,
		const std::string& depth2_path, const std::string& color2_path, const std::string& mask2_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);
template
void BuildSdfVolumeFromImage_SurfaceSpanAllocation<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume1,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume2,
		const std::string& depth1_path, const std::string& color1_path, const std::string& mask1_path,
		const std::string& depth2_path, const std::string& color2_path, const std::string& mask2_path,
		const std::string& calibration_path,
		MemoryDeviceType memory_device,
		VoxelBlockHash::InitializationParameters initialization_parameters,
		configuration::SwappingMode swapping_mode);


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

configuration::Configuration GenerateChangedUpConfiguration(){
	using namespace configuration;
	configuration::Configuration changed_up_configuration(
			VoxelVolumeParameters(0.005, 0.12, 4.12, 0.05, 200, true, 1.2f),
			SurfelVolumeParameters(0.4f, 0.5f, static_cast<float>(22 * M_PI / 180), 0.008f, 0.0003f, 3.4f, 26.0f, 5,
			                       1.1f, 4.5f, 21, 300, false, false),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHash::VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000)
					)
			),
			SlavchevaSurfaceTracker::Parameters(0.11f, 0.09f, 2.0f, 0.3f, 0.1f, 1e-6f),
			SlavchevaSurfaceTracker::Switches(false, true, false, true, false),
			TelemetrySettings(Vector3i(20, 23, 0), true, true, false, true, true, true),
			Paths("TestData/output1",
			      "TestData/calib_file1.txt",
			      "", "", "",
			      "TestData/frame_color_%%06i.png",
			      "TestData/frame_depth_%%06i.png",
			      "TestData/frame_mask_%%06i.png",
			      ""),
			AutomaticRunSettings(50, 16, true, true, true),
			NonRigidTrackingParameters(ITMLib::TRACKER_SLAVCHEVA_DIAGNOSTIC, 300, 0.0002f, 0.4f),
			true,
			false,
			MEMORYDEVICE_CPU,
			true,
			true,
			true,
			false,
			configuration::FAILUREMODE_RELOCALIZE,
			configuration::SWAPPINGMODE_ENABLED,
			configuration::LIBMODE_BASIC,
			configuration::INDEX_ARRAY,
			configuration::VERBOSITY_WARNING,
			"type=rgb,levels=rrbb"
	);
	return changed_up_configuration;
}

} // namespace test_utilities