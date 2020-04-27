//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

//local
#include "../../Objects/Volume/VoxelVolume.h"
#include "VolumeSequenceRecorder.h"
#include "../../Utils/Visualization/VolumeSliceVisualizer2D.h"
#include "../../Utils/Geometry/CardinalAxesAndPlanes.h"

#ifdef WITH_VTK

#include "../../Utils/Visualization/VoxelValueGrapher.h"
#include "../../Utils/Visualization/SceneSliceVisualizer3D.h"
#include "../../Utils/Visualization/SceneTrackingEnergyPlotter.h"

#endif

#include "../../Utils/Configuration.h"


namespace ITMLib {

namespace telemetry {
// where to save the data within the output directory
const std::string warp_iteration_2D_slices_folder_name;
const std::string live_iteration_2D_slices_folder_name;
const std::string canonical_scene_rasterized_folder_name;
const std::string live_scene_rasterized_folder_name;
const std::string frame_folder_prefix;

std::string GetAndCreateOutputFolderForFrame(int frame_index);

}

//TODO: adapt to record warpField properly (+test)
//TODO: adapt to live-scene-pair structure; the live scene is now split into two that are being ping-ponged (+test)

class TelemetryRecorderInterface {
public:

};

template<typename TVoxel, typename TWarp, typename TIndex>
class TelemetryRecorder :
		public TelemetryRecorderInterface {
public: // static variables


public: // member variables
	const configuration::TelemetrySettings settings;
	const std::string output_path;
	const bool focus_spots_enabled;

private: // member variables
	// various loggers & visualizers
#ifdef WITH_OPENCV
	std::unique_ptr<VolumeSliceVisualizer2D<TVoxel, TWarp, TIndex>> volume_2D_slice_visualizer;
#endif
#ifdef WITH_VTK
	std::unique_ptr<VoxelValueGrapher<TVoxel,TWarp,TIndex>> focus_point_TSDF_grapher;
	std::unique_ptr<SceneSliceVisualizer3D<TVoxel, TWarp, TIndex>> scene3DSliceVisualizer;
	std::unique_ptr<ITMSceneTrackingEnergyPlotter> energy_plotter;
#endif
	VolumeSequenceRecorder<TVoxel, TWarp, TIndex>* scene3DLogger = nullptr;

	std::ofstream energy_statistics_file;
	int current_frame_index;
	std::string current_frame_output_path;

public: // member functions
	TelemetryRecorder();
	TelemetryRecorder(const configuration::TelemetrySettings& settings, const std::string& output_path,
	                  bool focus_spots_enabled = true);
	~TelemetryRecorder();


	/**
	 * \brief To be called before performing recording operations for each frame (or once before all per-iteration
	 * operations for that frame)
	 * \param frame_index index of the frame in question
	 */
	void InitializeFrameRecording(int frame_index,
	                              const VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                              const VoxelVolume<TVoxel, TIndex>* raw_live_volume);


	void SaveWarps();
	void FinalizeFrameRecording();
	void RecordAndPlotEnergies(double totalDataEnergy,
	                           double totalLevelSetEnergy,
	                           double totalKillingEnergy,
	                           double totalSmoothnessEnergy,
	                           double totalEnergy);
	bool IsRecordingWarp2DSlices();
	bool IsRecordingWarps();
	void SaveWarpSlices(int iteration);
	void LogHighlight(int hash, int locId, ITMHighlightIterationInfo info);

	TelemetryRecorder(TelemetryRecorder const&) = delete;
	void operator=(TelemetryRecorder const&) = delete;

private: // member functions



	std::string GetOutputDirectoryFor2DSceneSlicesWithWarps() const;
	std::string GetOutputDirectoryFor2DLiveSceneSliceProgression() const;
	std::string GetOutputDirectoryPrefixForLiveSceneAsSlices() const;
	std::string GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const;
	void MakeOrClearOutputDirectoriesFor2DSceneSlices() const;


};

} //namespace InfiniTAM