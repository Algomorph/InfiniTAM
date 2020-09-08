//  ================================================================
//  Created by Gregory Kramida on 3/6/20.
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
#include "AnalyticsLogging.h"
#include "../../Utils/Quaternions/QuaternionFromMatrix.h"

namespace ITMLib {


template<>
void GetMaxBlockCounts<PlainVoxelArray>(int& canonical_max_block_count, int& live_max_block_count, int& warp_max_block_count) {
	canonical_max_block_count = live_max_block_count = warp_max_block_count = 1;
}

template<>
void GetMaxBlockCounts<VoxelBlockHash>(int& canonical_max_block_count, int& live_max_block_count, int& warp_max_block_count) {
	canonical_max_block_count = configuration::ForVolumeRole<VoxelBlockHash>(configuration::VOLUME_CANONICAL).voxel_block_count;
	live_max_block_count = configuration::ForVolumeRole<VoxelBlockHash>(configuration::VOLUME_LIVE).voxel_block_count;
	warp_max_block_count = configuration::ForVolumeRole<VoxelBlockHash>(configuration::VOLUME_WARP).voxel_block_count;
}

template void LogVoxelHashBlockUsage<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* canonical_volume,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* live_volume,
		VoxelVolume<WarpVoxel, VoxelBlockHash>* warp_volume, IndexingMethod indexing_method);
template void LogVoxelHashBlockUsage<TSDFVoxel, WarpVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* canonical_volume,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* live_volume,
		VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_volume, IndexingMethod indexing_method);

template void LogTSDFVolumeStatistics<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume,
                                                                 const std::string&, IndexingMethod indexing_method);
template void LogTSDFVolumeStatistics<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume,
                                                                  const std::string&, IndexingMethod indexing_method);

void LogCameraTrajectoryQuaternion(const ORUtils::SE3Pose* p) {
	if (configuration::Get().logging_settings.log_trajectory_quaternions) {
		double t[3];
		double R[9];
		double q[4];
		for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				R[r * 3 + c] = p->GetM().m[c * 4 + r];
		QuaternionFromRotationMatrix(R, q);
		LOG4CPLUS_PER_FRAME(logging::GetLogger(),
		                    "Camera quaternion: " << t[0] << " " << t[1] << " " << t[2] << " "
		                                          << q[1] << " " << q[2] << " " << q[3] << " " << q[0]);
	}
};

} // namespace ITMLib

