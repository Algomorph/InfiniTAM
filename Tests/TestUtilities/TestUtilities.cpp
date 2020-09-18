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

// stdlib
#include <filesystem>
// boost
#include <boost/test/test_tools.hpp>

// test utilities
#include "TestUtilities.h"
#include "TestUtilities.tpp"

// OpenReco
#include "../../ITMLib/Utils/Quaternions/Quaternion.h"
#include "../../ITMLib/Utils/Configuration/AutomaticRunSettings.h"
#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../../ITMLib/Engines/Telemetry/TelemetrySettings.h"
#include "../../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../ITMLib/Engines/Main/MainEngineSettings.h"
#include "../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"
#include "../../ITMLib/Engines/VolumeFusion/VolumeFusionSettings.h"

using namespace ITMLib;
namespace fs = std::filesystem;

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

void ConstructGeneratedVolumeSubdirectoriesIfMissing() {
	fs::create_directories(test_utilities::GeneratedVolumeDirectory + IndexString<VoxelBlockHash>());
	fs::create_directories(test_utilities::GeneratedVolumeDirectory + IndexString<PlainVoxelArray>());
}

void ConstructGeneratedConfigurationDirectoryIfMissing() {
	fs::create_directories(test_utilities::GeneratedConfigurationDirectory);
}

void ConstructGeneratedMeshDirectoryIfMissing() {
	fs::create_directories(test_utilities::GeneratedMeshDirectory);
}

void ConstructGeneratedArraysDirectoryIfMissing() {
	fs::create_directories(test_utilities::GeneratedArraysDirectory);
}

void ConstructGeneratedVideosDirectoryIfMissing() {
	fs::create_directories(test_utilities::GeneratedVideosDirectory);
}

template void GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU, TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume);
template void GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU, TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume);
template void GenerateRandomDepthWeightSubVolume<MEMORYDEVICE_CPU, TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume, const Extent3Di& bounds, const Extent2Di&
weight_range);

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

	auto* rgb = new UChar4Image(true, false);
	auto* depth = new ShortImage(true, false);
	auto* mask = new UCharImage(true, false);
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

	auto* rgb = new UChar4Image(true, false);
	auto* depth = new ShortImage(true, false);
	auto* mask = new UCharImage(true, false);
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

configuration::Configuration GenerateChangedUpConfiguration() {
	using namespace configuration;
	configuration::Configuration changed_up_configuration(
			Vector3i(20, 23, 0),
			true,
			true,
			VoxelVolumeParameters(0.005, 0.12, 4.12, 0.05, 200, true, 1.2f),
			SurfelVolumeParameters(0.4f, 0.5f, static_cast<float>(22 * M_PI / 180), 0.008f, 0.0003f, 3.4f, 26.0f, 5,
			                       1.1f, 4.5f, 21, 300, false, false),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHashParameters(0x20000, 0x20000)
					)
			),
			LoggingSettings(
					VERBOSITY_WARNING,
					true,
					false,
					true,
					true,
					true,
					true,
					true,
					true,
					true,
					true,
					true,
					true),
			Paths(GENERATED_TEST_DATA_PREFIX "TestData/output",
			      STATIC_TEST_DATA_PREFIX "TestData/calibration/snoopy_calib.txt",
			      "", "", "",
			      STATIC_TEST_DATA_PREFIX "TestData/frames/frame_color_%%06i.png",
			      STATIC_TEST_DATA_PREFIX "TestData/frames/frame_depth_%%06i.png",
			      STATIC_TEST_DATA_PREFIX "TestData/frames/frame_mask_%%06i.png",
			      ""),
			true,
			MEMORYDEVICE_CPU,
			true,
			true,
			true,
			configuration::FAILUREMODE_RELOCALIZE,
			configuration::SWAPPINGMODE_ENABLED,
			"type=rgb,levels=rrbb"
	);
	changed_up_configuration.source_tree = changed_up_configuration.ToPTree();

	TelemetrySettings changed_up_telemetry_settings(
			true,
			true,
			true,
			true,
			true,
			true,
			0.0001,
			32,
			true,
			true);
	MainEngineSettings changed_up_main_engine_settings(true, LIBMODE_BASIC, INDEX_ARRAY, false);
	IndexingSettings changed_up_indexing_settings(DIAGNOSTIC);
	RenderingSettings changed_up_rendering_settings(true);
	AutomaticRunSettings changed_up_automatic_run_settings(50, 16, true, true, true, true);
	LevelSetAlignmentParameters changed_up_level_set_evolution_parameters(
			ExecutionMode::DIAGNOSTIC,
			LevelSetAlignmentWeights(0.11f, 0.09f, 2.0f, 0.3f, 0.1f, 1e-6f, 0.4f),
			LevelSetAlignmentSwitches(false, true, false, true, false),
			LevelSetAlignmentTerminationConditions(300, 5, 0.0002f)
	);
	VolumeFusionSettings changed_up_volume_fusion_settings(true, 0.008);
	DepthFusionSettings changed_up_depth_fusion_settings(true, 0.008);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_main_engine_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_telemetry_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_indexing_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_rendering_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_automatic_run_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_level_set_evolution_parameters);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_volume_fusion_settings);
	AddDeferrableToSourceTree(changed_up_configuration, changed_up_depth_fusion_settings);
	return changed_up_configuration;
}

std::vector<ORUtils::SE3Pose> GenerateCameraTrajectoryAroundPoint(const Vector3f& original_viewpoint, const Vector3f& target, int degree_increment) {
	using namespace quaternion;
	Vector3f x_axis_unit(1.0f, 0.0f, 0.0f);
	Vector3f rotation_axis = ORUtils::normalize(ORUtils::cross(target, x_axis_unit));

	Vector3f target_to_viewpoint = original_viewpoint - target;
	Quaternion<float> target_to_viewpoint_quaternion(0.f, target_to_viewpoint.x, target_to_viewpoint.y,
	                                                 target_to_viewpoint.z);
	float angle_radians = 0.0;

	const float angle_increment = static_cast<float>(degree_increment) * PI / 180.0f;

	std::vector<ORUtils::SE3Pose> poses;
	for (int i_sample = 0; i_sample < (360 / degree_increment); i_sample++) {

		float half_angle = angle_radians / 2.0f; // for code clarity, math not baked in
		float cos_theta = cos(half_angle);
		float sin_theta = sin(half_angle);
		Quaternion<float> half_rotation_a(cos_theta, rotation_axis.x * sin_theta, rotation_axis.y * sin_theta,
		                                  rotation_axis.z * sin_theta);
		Quaternion<float> half_rotation_b(cos_theta, -rotation_axis.x * sin_theta, -rotation_axis.y * sin_theta,
		                                  -rotation_axis.z * sin_theta);
		Quaternion<float> rotation_result = (half_rotation_a *= target_to_viewpoint_quaternion) *= half_rotation_b;
		Vector3f rotated_target_to_viewpoint(rotation_result.b(), rotation_result.c(), rotation_result.d());
		Vector3f viewpoint = target + rotated_target_to_viewpoint;

		Vector3f viewpoint_to_target_normalized = ORUtils::normalize(-rotated_target_to_viewpoint);
		/* InfiniTAM's default (unconventional) axis system is as follows:
		 *               ◢ ----------▶ +x
		 *             ╱ |
		 *           ╱   |
		 *         ╱     |
		 *    +z ◣       |
		 *               |
		 *               ▼ +y
		 * (Probably, this was done because it mimics the default +x/+y image pixel raster ordering.
		 *
		 * Sticking with the right-hand rule for rotations, euler rotation of a (0,0,1) unit vector to some
		 * arbitrary unit-vector v is calculated as follows:
		 *
		 * rx = atan2(v.y, v.z)
		 * ry = atan2(v.x, v.z)
		 * rz = atan2(v.x, v.y)
		 *
		 */
		Vector3f euler(atan2(viewpoint_to_target_normalized.y, viewpoint_to_target_normalized.z),
		               atan2(viewpoint_to_target_normalized.x, viewpoint_to_target_normalized.z), 0.0f);
		ORUtils::SE3Pose pose;
		pose.SetFrom(viewpoint, euler);
		poses.push_back(pose);
		angle_radians += angle_increment;
	}
	return poses;
}


} // namespace test_utilities