//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/18/20.
//  Copyright (c) $YEAR-2020 Gregory Kramida
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
#include "AnalyticsTelemetry.h"

namespace ITMLib {

template void RecordVolumeMemoryUsageInfo<TSDFVoxel, VoxelBlockHash>(
		ORUtils::MemoryBlockOStreamWrapper& file,
		const VoxelVolume<TSDFVoxel, VoxelBlockHash>& volume);

template void RecordVolumeMemoryUsageInfo<TSDFVoxel, PlainVoxelArray>(
		ORUtils::MemoryBlockOStreamWrapper& file,
		const VoxelVolume<TSDFVoxel, PlainVoxelArray>& volume);


void InitializePerFrameAnalyticsTelemetry(ORUtils::MemoryBlockOStreamWrapper* canonical_volume_memory_usage_file) {
	if (configuration::get().telemetry_settings.record_volume_memory_usage) {
		canonical_volume_memory_usage_file = new ORUtils::MemoryBlockOStreamWrapper(
				(fs::path(configuration::get().paths.output_path) / fs::path("canonical_volume_memory_usage.dat")).string(),
				true
		);
	}
}

} // namespace ITMLib


