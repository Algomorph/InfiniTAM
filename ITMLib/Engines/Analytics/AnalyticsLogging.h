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
#pragma once

//stdlib
#include <string>
#include <vector>
#include <unordered_set>

//log4cplus
#include <log4cplus/loggingmacros.h>

//local
#include "../../Utils/Math.h"
#include "../../Utils/Logging/LoggingConfigruation.h"
#include "AnalyticsEngineFactory.h"
#include "../../Utils/Collections/OperationsOnSTLContainers.h"
#include "../../Utils/CPPPrintHelpers.h"

namespace ITMLib {

template<typename TVoxel, typename TIndex>
void LogTSDFVolumeStatistics(VoxelVolume<TVoxel, TIndex>* volume, std::string volume_description) {
	if (configuration::get().telemetry_settings.log_volume_statistics) {
		AnalyticsEngineInterface<TVoxel, TIndex>& calculator =
				AnalyticsEngineFactory::Get<TVoxel, TIndex>(volume->index.memory_type);
		LOG4CPLUS_PER_FRAME(logging::get_logger(),
		                    green << "=== Stats for volume '" << volume_description << "' ===" << reset);
//#define GET_VOXEL_ALLOCATION_STATISTICS
#ifdef GET_VOXEL_ALLOCATION_STATISTICS
		unsigned int allocated_voxel_count = calculator.CountAllocatedVoxels(volume);
		unsigned int allocated_hash_block_count = calculator.CountAllocatedHashBlocks(volume);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    Allocated voxel count: " << allocated_voxel_count);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    Allocated block count: " << allocated_hash_block_count);
#endif

#define GET_UTILIZED_COUNTS
#ifdef GET_UTILIZED_COUNTS
		unsigned int utilized_voxel_count = calculator.CountUtilizedVoxels(volume);
		unsigned int utilized_hash_block_count = calculator.CountUtilizedHashBlocks(volume);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    Utilized voxel count: " << utilized_voxel_count);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    Utilized block count: " << utilized_hash_block_count);
#endif

//#define DEBUG_ALLOCATION
#ifdef DEBUG_ALLOCATION
		std::vector<Vector3s> utilized_positions = calculator.GetUtilizedHashBlockPositions(volume);
		std::unordered_set<Vector3s> utilized_position_set(utilized_positions.begin(), utilized_positions.end());
		LOG4CPLUS_DEBUG(logging::get_logger(),
		                "Allocated position count: " << utilized_positions.size()
		                                             << "; unique utilized position count: "
		                                             << utilized_position_set.size());

		std::vector<Vector3s> allocated_positions = calculator.GetAllocatedHashBlockPositions(volume);
		std::unordered_set<Vector3s> allocated_position_set(allocated_positions.begin(), allocated_positions.end());
		LOG4CPLUS_DEBUG(logging::get_logger(),
		                "Allocated position count: " << allocated_positions.size()
		                                             << "; unique allocated position count: "
		                                             << allocated_position_set.size());
		std::unordered_set<Vector3s> difference_set = allocated_position_set - utilized_position_set;
		if (!difference_set.empty()) {
			LOG4CPLUS_DEBUG(logging::get_logger(), "Difference items: ");
			std::stringstream stuff;
			stuff << "| ";
			for (const auto item : difference_set) {
				stuff << item << " | ";
			}
			stuff << std::endl;
			LOG4CPLUS_DEBUG(logging::get_logger(), stuff.str());
		}
		Vector3s block_of_interest(16, 4, 27);
		if (utilized_position_set.find(block_of_interest) == utilized_position_set.end()) {
			LOG4CPLUS_DEBUG(logging::get_logger(),
			                "Could not find block " << block_of_interest << " among utilized blocks.");
		}


#endif
#ifdef GET_VOXEL_CATEGORY_STATISTICS
		unsigned int non_truncated_voxel_count = calculator.CountNonTruncatedVoxels(volume);
		unsigned int plus_one_voxel_count = calculator.CountVoxelsWithSpecificSdfValue(volume, 1.0f);
		double sum_non_truncated_abs_sdf = calculator.SumNonTruncatedVoxelAbsSdf(volume);
		double sum_truncated_abs_sdf = calculator.SumTruncatedVoxelAbsSdf(volume);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    NonTruncated SDF sum: " << sum_non_truncated_abs_sdf);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    Truncated SDF sum: " << sum_truncated_abs_sdf);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    NonTruncated voxel count: " << non_truncated_voxel_count);
		LOG4CPLUS_PER_FRAME(logging::get_logger(), "    +1.0 voxel count: " << plus_one_voxel_count);
#endif
#define GET_DEPTH_WEIGHT_STATISTICS
#ifdef GET_DEPTH_WEIGHT_STATISTICS
		if (configuration::get().device_type == MEMORYDEVICE_CUDA &&
			configuration::get().indexing_method == configuration::INDEX_HASH) {
			//_DEBUG
			Extent2Di low_weight_range0(0, 50);
			unsigned int low_weight_range_count0 =
					calculator.CountVoxelsWithDepthWeightInRange(volume, low_weight_range0);
			unsigned int low_weight_range_hb_count0 =
					calculator.CountHashBlocksWithDepthWeightInRange(volume, low_weight_range0);
			LOG4CPLUS_PER_FRAME(logging::get_logger(), "    [w_depth in [0, 50)] % voxels: "
					<< 100.0 * static_cast<double>(low_weight_range_count0) / static_cast<double>(utilized_voxel_count)
					<< "; % hash blocks: "
					<< 100.0 * static_cast<double>(low_weight_range_hb_count0) /
					   static_cast<double>(utilized_hash_block_count)
			);
			Extent2Di low_weight_range1(0, 20);
			unsigned int low_weight_range_count1 =
					calculator.CountVoxelsWithDepthWeightInRange(volume, low_weight_range1);
			unsigned int low_weight_range_hb_count1 =
					calculator.CountHashBlocksWithDepthWeightInRange(volume, low_weight_range1);
			LOG4CPLUS_PER_FRAME(logging::get_logger(), "    [w_depth in [0, 20)] % voxels: "
					<< 100.0 * static_cast<double>(low_weight_range_count1) / static_cast<double>(utilized_voxel_count)
					<< "; % hash blocks: "
					<< 100.0 * static_cast<double>(low_weight_range_hb_count1) /
					   static_cast<double>(utilized_hash_block_count)
			);
			Extent2Di low_weight_range2(0, 10);
			unsigned int low_weight_range_count2 =
					calculator.CountVoxelsWithDepthWeightInRange(volume, low_weight_range2);
			unsigned int low_weight_range_hb_count2 =
					calculator.CountHashBlocksWithDepthWeightInRange(volume, low_weight_range2);
			LOG4CPLUS_PER_FRAME(logging::get_logger(), "    [w_depth in [0, 10)] % voxels: "
					<< 100.0 * static_cast<double>(low_weight_range_count2) / static_cast<double>(utilized_voxel_count)
					<< "; % hash blocks: "
					<< 100.0 * static_cast<double>(low_weight_range_hb_count2) /
					   static_cast<double>(utilized_hash_block_count)
			);
		}
#endif
	}
};

extern template void LogTSDFVolumeStatistics<TSDFVoxel, VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume,
                                                                        std::string volume_description);
extern template void
LogTSDFVolumeStatistics<TSDFVoxel, PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume,
                                                    std::string volume_description);


} // namespace ITMLib
