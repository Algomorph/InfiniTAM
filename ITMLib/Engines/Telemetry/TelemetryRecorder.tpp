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
#include "../../../ORUtils/VectorAndMatrixPersistence.h"
#include "TelemetryRecorder.h"
#include "../VolumeFileIO/VolumeFileIOEngine.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../GlobalTemplateDefines.h"
#include "../../Utils/Telemetry/TelemetryUtilities.h"
#include "../Meshing/MeshingEngineFactory.h"
#include "../Analytics/AnalyticsEngine.h"
#include "../../Utils/Logging/ConsolePrintColors.h"
#include "../Traversal/Interface/VolumeTraversal.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::TelemetryRecorder()
		:TelemetryRecorderInterface<TVoxel, TWarp, TIndex>(),
		 canonical_volume_memory_usage_file(parameters.record_volume_memory_usage ?
		                                    ORUtils::OStreamWrapper((fs::path(configuration::Get().paths.output_path)
		                                                             / fs::path("canonical_volume_memory_usage.dat")).string(), true)
		                                                                          : ORUtils::OStreamWrapper()),
		 camera_trajectory_file(parameters.record_camera_matrices ?
		                        ORUtils::OStreamWrapper((fs::path(configuration::Get().paths.output_path)
		                                                 / fs::path("camera_matrices.dat")).string(), true)
		                                                          : ORUtils::OStreamWrapper()),
		 surface_tracking_energy_file(parameters.record_surface_tracking_optimization_energies ?
		                              ORUtils::OStreamWrapper((fs::path(configuration::Get().paths.output_path)
		                                                       / fs::path("surface_tracking_energies.dat")).string(), true)
		                                                                                       : ORUtils::OStreamWrapper()),
		 surface_tracking_statistics_file(parameters.record_surface_tracking_additional_statistics ?
		                                  ORUtils::OStreamWrapper((fs::path(configuration::Get().paths.output_path)
		                                                           / fs::path("surface_tracking_statistics.dat")).string(), true)
		                                                                                           : ORUtils::OStreamWrapper()) {}

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
		ORUtils::SaveMatrix(camera_trajectory_file, camera_pose);
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
			meshing_engine = MeshingEngineFactory::Build<TVoxel, TIndex>(configuration::Get().device_type);
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

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordSurfaceTrackingEnergies(const ComponentEnergies<TMemoryDeviceType>& energies,
                                                                                           int iteration_index) {
	if (parameters.record_surface_tracking_optimization_energies) {
		surface_tracking_energy_file.OStream().write(reinterpret_cast<const char*>(&iteration_index), sizeof(int));
		float total_data_energy = energies.GetTotalDataEnergy();
		surface_tracking_energy_file.OStream().write(reinterpret_cast<const char*>(&total_data_energy), sizeof(float));
		float total_level_set_energy = energies.GetTotalLevelSetEnergy();
		surface_tracking_energy_file.OStream().write(reinterpret_cast<const char*>(&total_level_set_energy), sizeof(float));
		float total_Tikhonov_energy = energies.GetTotalTikhonovEnergy();
		surface_tracking_energy_file.OStream().write(reinterpret_cast<const char*>(&total_Tikhonov_energy), sizeof(float));
		float total_Killing_energy = energies.GetTotalKillingEnergy();
		surface_tracking_energy_file.OStream().write(reinterpret_cast<const char*>(&total_Killing_energy), sizeof(float));
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordSurfaceTrackingStatistics(
		const AdditionalGradientAggregates<TMemoryDeviceType>& aggregates, int iteration_index) {
	if (parameters.record_surface_tracking_additional_statistics) {
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&iteration_index), sizeof(int));
		float average_canonical_sdf = aggregates.GetAverageCanonicalSdf();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&average_canonical_sdf), sizeof(float));
		float average_live_sdf = aggregates.GetAverageLiveSdf();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&average_live_sdf), sizeof(float));
		float average_sdf_difference = aggregates.GetAverageSdfDifference();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&average_sdf_difference), sizeof(float));
		float average_warp_distance = aggregates.GetAverageWarpDistance();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&average_warp_distance), sizeof(float));
		unsigned int considered_voxel_count = aggregates.GetConsideredVoxelCount();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&considered_voxel_count), sizeof(unsigned int));
		unsigned int data_voxel_count = aggregates.GetDataVoxelCount();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&data_voxel_count), sizeof(unsigned int));
		unsigned int level_set_voxel_count = aggregates.GetLevelSetVoxelCount();
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&level_set_voxel_count), sizeof(unsigned int));

	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordSurfaceTrackingMeanUpdate(float mean_update) {
	if (parameters.record_surface_tracking_additional_statistics) {
		surface_tracking_statistics_file.OStream().write(reinterpret_cast<const char*>(&mean_update), sizeof(float));
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordWarpUpdateLengthHistogram(
		const Histogram& histogram, int iteration_index) {
	if (parameters.record_warp_update_length_histograms) {
		warp_update_length_histogram_file.OStream().write(reinterpret_cast<const char*>(&iteration_index), sizeof(int));
		warp_update_length_histogram_file.OStream() << histogram;
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::RecordAndLogWarpUpdateLengthHistogram(const VoxelVolume<TWarp, TIndex>& warp_field) {
	if (parameters.record_warp_update_length_histograms || configuration::Get().logging_settings.log_warp_update_length_histograms) {
		auto& analytics_engine = AnalyticsEngine<TWarp, TIndex, TMemoryDeviceType>::Instance();
		Histogram histogram;
		if(parameters.use_warp_update_length_histogram_manual_max){
			histogram = analytics_engine.ComputeWarpUpdateLengthHistogram_ManualMax(&warp_field, parameters.warp_update_length_histogram_bin_count, parameters.warp_update_length_histogram_max);
		}else{
			histogram = analytics_engine.ComputeWarpUpdateLengthHistogram_VolumeMax(&warp_field, parameters.warp_update_length_histogram_bin_count);
		}

	}
}




