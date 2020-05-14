//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/3/20.
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
//stdlib
#include <filesystem>

namespace fs = std::filesystem;

//local
#include "TelemetryRecorder.h"
#include "../VolumeFileIO/VolumeFileIOEngine.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../GlobalTemplateDefines.h"
#include "../../Utils/Telemetry/TelemetryUtilities.h"
#include "../Meshing/MeshingEngineFactory.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::TelemetryRecorder()
		:TelemetryRecorderInterface<TVoxel, TWarp, TIndex>(),
		canonical_volume_memory_usage_file(parameters.record_volume_memory_usage ?
		ORUtils::OStreamWrapper((fs::path(configuration::get().paths.output_path)
		                         / fs::path("canonical_volume_memory_usage.dat")).string(), true, true)
		                                                                   : ORUtils::OStreamWrapper()),
		  camera_trajectory_file(parameters.record_camera_matrices ?
		                         ORUtils::OStreamWrapper((fs::path(configuration::get().paths.output_path)
		                                                  / fs::path("camera_matrices.dat")).string(), true, true)
		                                                                                        : ORUtils::OStreamWrapper()),
                                    surface1_point_cloud(Vector2i(0,0),TMemoryDeviceType),
                                    surface2_point_cloud(Vector2i(0,0),TMemoryDeviceType),
                                    march_endpoint1_point_cloud(Vector2i(0,0),TMemoryDeviceType),
                                    march_endpoint2_point_cloud(Vector2i(0,0),TMemoryDeviceType){}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordVolumeMemoryUsageInfo(
		const VoxelVolume<TVoxel, TIndex>& canonical_volume) {
	if (parameters.record_volume_memory_usage) {
		VolumeFileIOEngine<TVoxel, TIndex>::AppendFileWithUtilizedMemoryInformation(
				this->canonical_volume_memory_usage_file, canonical_volume);
	}
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordCameraPose(const Matrix4f& camera_pose) {
	if (parameters.record_camera_matrices) {
		for (int i_value = 0; i_value < 16; i_value++) {
			float value = camera_pose.getValues()[i_value];
			camera_trajectory_file.OStream().write(reinterpret_cast<const char*>(&value),sizeof(float));
		}
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordFrameMeshFromVolume(
		const VoxelVolume<TVoxel, TIndex>& volume, const std::string& filename, int frame_index) {
	if (parameters.record_frame_meshes) {
		std::string frame_output_path = telemetry::CreateAndGetOutputPathForFrame(frame_index);
		std::string mesh_file_path = (fs::path(frame_output_path) / fs::path(filename)).string();
		MeshingEngine<TVoxel, TIndex>* meshing_engine;
		if (parameters.use_CPU_for_mesh_recording) {
			meshing_engine = MeshingEngineFactory::Build<TVoxel, TIndex>(MEMORYDEVICE_CPU);
		} else {
			meshing_engine = MeshingEngineFactory::Build<TVoxel, TIndex>(configuration::get().device_type);
		}

		Mesh mesh = meshing_engine->MeshVolume(&volume);
		mesh.WritePLY(mesh_file_path, false, false);
		delete meshing_engine;
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordPreSurfaceTrackingData(
		const VoxelVolume<TVoxel, TIndex>& raw_live_volume, const Matrix4f camera_matrix, int frame_index) {
	RecordFrameMeshFromVolume(raw_live_volume, "live_raw.ply", frame_index);
	RecordCameraPose(camera_matrix);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordPostSurfaceTrackingData(
		const VoxelVolume<TVoxel, TIndex>& warped_live_volume, int frame_index) {
	RecordFrameMeshFromVolume(warped_live_volume, "live_warped.ply", frame_index);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordPostFusionData(
		const VoxelVolume<TVoxel, TIndex>& canonical_volume, int frame_index) {
	RecordVolumeMemoryUsageInfo(canonical_volume);
	RecordFrameMeshFromVolume(canonical_volume, "canonical.ply", frame_index);
}


