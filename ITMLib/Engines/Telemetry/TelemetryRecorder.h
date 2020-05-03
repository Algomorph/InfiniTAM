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
#pragma once

#include "../../Objects/Volume/VoxelVolume.h"

namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class TelemetryRecorderInterface {
public:
	virtual ~TelemetryRecorderInterface() = default;
	virtual void RecordPreTrackingData(const VoxelVolume <TVoxel, TIndex>& raw_live_volume, int frame_index) = 0;
	virtual void RecordPostTrackingData(const VoxelVolume <TVoxel, TIndex>& warped_live_volume, int frame_index) = 0;
	virtual void RecordPostFusionData(const VoxelVolume <TVoxel, TIndex>& canonical_volume, int frame_index) = 0;
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class TelemetryRecorder : public TelemetryRecorderInterface<TVoxel, TWarp, TIndex> {

private: // member variables
	ORUtils::OStreamWrapper canonical_volume_memory_usage_file;
public: // member functions
	static TelemetryRecorder& GetDefaultInstance() {
		static TelemetryRecorder instance;
		return instance;
	}

	TelemetryRecorder();
	void RecordPreTrackingData(const VoxelVolume <TVoxel, TIndex>& raw_live_volume, int frame_index) override;
	void RecordPostTrackingData(const VoxelVolume <TVoxel, TIndex>& warped_live_volume, int frame_index) override;
	void RecordPostFusionData(const VoxelVolume <TVoxel, TIndex>& canonical_volume, int frame_index) override;
private: // member functions
	void RecordVolumeMemoryUsageInfo(const VoxelVolume <TVoxel, TIndex>& canonical_volume);
	void
	RecordFrameMeshFromVolume(const VoxelVolume <TVoxel, TIndex>& volume, const std::string& filename, int frame_index);
};


} // namespace ITMLib