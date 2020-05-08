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

//stdlib
#include "iomanip"

#ifdef WITH_OPENCV
//OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#endif

//local
#include "TelemetryRecorderLegacy.h"
#include "../../Utils/Analytics/BenchmarkUtilities.h"
#include "../../Utils/CPPPrintHelpers.h"
#include "../../Utils/Configuration/Configuration.h"


using namespace ITMLib;
namespace bench = ITMLib::benchmarking;

//TODO: create/destroy windowed visualizers (i.e. plotter) when their corresponding setting values are turned on/off, and make them Close their corresponding windows -Greg (GitHub:Algomorph)


template<typename TVoxel, typename TWarp, typename TIndex>
TelemetryRecorder<TVoxel, TWarp, TIndex>::TelemetryRecorder()
		: TelemetryRecorder<TVoxel, TWarp, TIndex>(configuration::get().telemetry_settings,
		                                           configuration::get().paths.output_path,
		                                           configuration::get().logging_settings.verbosity_level >=
		                                           VERBOSITY_FOCUS_SPOTS) {};


template<typename TVoxel, typename TWarp, typename TIndex>
TelemetryRecorder<TVoxel, TWarp, TIndex>::TelemetryRecorder(const configuration::TelemetrySettings& settings,
                                                            const std::string& output_path,
                                                            bool focus_spots_enabled)
		:
#ifdef WITH_OPENCV
		volume_2D_slice_visualizer(),
#endif
		settings(settings),
		output_path(output_path),
		focus_spots_enabled(focus_spots_enabled) {}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::InitializeFrameRecording(
		int frame_index,
		const VoxelVolume<TVoxel, TIndex>* canonical_volume,
		const VoxelVolume<TVoxel, TIndex>* raw_live_volume
) {
	this->current_frame_index = frame_index;
	this->current_frame_output_path = telemetry::GetAndCreateOutputFolderForFrame(frame_index);


	//TODO: make all visualizer/logger classes re-usable, i.e. just change the path & build them in the constructor (don't use pointers) -Greg (GitHub:Algomorph)
	// region ================================== 1D/2D SLICE RECORDING =================================================
	if (this->focus_spots_enabled) {

		if (settings.record_live_focus_point_TSDF_graph) {
#ifdef WITH_VTK
			this->focus_point_TSDF_grapher.reset(new VoxelValueGrapher(configuration::get().focus_coordinates, AXIS_X, 16));
			focus_point_TSDF_grapher->Plot1DSceneSlice(canonical_volume, Vector4i(97, 181, 193, 255), 3.0);
			focus_point_TSDF_grapher->Plot1DSceneSlice(raw_live_volume, Vector4i(183, 115, 46, 255), 3.0);
#else
			std::cerr << "Warning: code built without VTK support, hence ignoring the attempt to record 1D volume slices"
				" with updates" << std::endl;
#endif
		}
#ifdef WITH_OPENCV
		volume_2D_slice_visualizer.reset(
				new VolumeSliceVisualizer2D<TVoxel, TWarp, TIndex>(configuration::get().focus_coordinates, 100,
				                                                   16.0,
				                                                   planeFor2Dand3DSlices));

		MakeOrClearOutputDirectoriesFor2DSceneSlices();
		if (recordingCanonicalSceneAs2DSlices) {
			volume_2D_slice_visualizer->SaveSceneSlicesAs2DImages_AllDirections(
					canonical_volume, GetOutputDirectoryPrefixForCanonicalSceneAsSlices());
		}
		if (recordingLiveSceneAs2DSlices) {
			volume_2D_slice_visualizer->SaveSceneSlicesAs2DImages_AllDirections(
					raw_live_volume, GetOutputDirectoryPrefixForLiveSceneAsSlices());
		}
#endif
		if (recordingScene2DSlicesWithUpdates) {
			std::cout << yellow << "Recording 2D scene slices with warps & warped live scene progression as images in "
			          << GetOutputDirectoryFor2DSceneSlicesWithWarps() << " and "
			          << GetOutputDirectoryFor2DLiveSceneSliceProgression() << " respectively..."
			          << reset << std::endl;
			InitializeWarp2DSliceRecording(canonical_volume, raw_live_volume);
		}

		if (recordingScene3DSlicesWithUpdates) {
#ifdef WITH_VTK
			if (!scene3DSliceVisualizer) {
				scene3DSliceVisualizer.reset(new SceneSliceVisualizer3D<TVoxel, TWarp, TIndex>
						                             (canonical_volume, raw_live_volume, warpField, configuration::get().focus_coordinates,
						                              planeFor2Dand3DSlices, _3dSliceInPlaneRadius,
						                              _3dSliceOutOfPlaneRadius));
			} else {
				scene3DSliceVisualizer->TriggerRebuildSlices();
			}
#else
			std::cerr << "Warning: code compiled without VTK support, "
						 "hence ignoring the attempt to record scene 3D slices with updates" << std::endl;
#endif
		}

	} else {
		if (recordingScene1DSlicesWithUpdates || recordingCanonicalSceneAs2DSlices || recordingLiveSceneAs2DSlices ||
		    recordingScene2DSlicesWithUpdates || recordingScene3DSlicesWithUpdates) {
			std::cout << red << "WARNING: Recording 1D/2D/3D slices or saving live/canonical frames as 2D slices "
			                    "requires focus coordinates to be set (and they were not)." << reset << std::endl;
		}
	}
	// endregion
	if (plottingEnergies) {
#ifdef WITH_VTK
		this->energy_plotter.reset(new ITMSceneTrackingEnergyPlotter());
#else
		std::cerr << "Warning: code built without VTK support, hence ignoring the attempt to plot energies on graphs"
		<< std::endl;
#endif
	}

	// region ========================= INITIALIZE WARP RECORDING ======================================================
	benchmarking::start_timer("TrackMotion_2_RecordingEnergy");

	const std::string energyStatFilePath = outputDirectory + "/energy.txt";
	energy_statistics_file = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energy_statistics_file << "data" << "," << "level_set" << "," << "smoothness" << ","
	                       << "killing" << "," << "total" << std::endl;
	benchmarking::stop_timer("TrackMotion_2_RecordingEnergy");

	if (recording3DSceneAndWarpProgression) {
		scene3DLogger->SaveScenesCompact();
		scene3DLogger->StartSavingWarpState();
		if (hasFocusCoordinates) {
			scene3DLogger->ClearHighlights();
		}
	}
	// endregion =======================================================================================================
}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::FinalizeFrameRecording() {
	if (recording3DSceneAndWarpProgression) {
		scene3DLogger->StopSavingWarpState();
		if (hasFocusCoordinates) {
			Vector3i sliceMinPoint(configuration::get().focus_coordinates[0] - focusSliceRadius,
			                       configuration::get().focus_coordinates[1] - focusSliceRadius,
			                       configuration::get().focus_coordinates[2] - focusSliceRadius);
			Vector3i sliceMaxPoint(configuration::get().focus_coordinates[0] + focusSliceRadius,
			                       configuration::get().focus_coordinates[1] + focusSliceRadius,
			                       configuration::get().focus_coordinates[2] + focusSliceRadius);

			std::cout << "Making slice around voxel " << green << configuration::get().focus_coordinates << reset << " with l_0 radius of "
			          << focusSliceRadius << "...";
			std::string sliceId;
			scene3DLogger->MakeSlice(sliceMinPoint, sliceMaxPoint, sliceId);
			std::cout << "Slice finished." << std::endl;
			scene3DLogger->SwitchActiveScene(sliceId);
		}
	}
	if (plottingEnergies) {
#ifdef WITH_VTK
		energy_plotter->SaveScreenshot(this->outputDirectory + "/energy_plot.png");
		energy_plotter.reset();
#else
		std::cerr << "Warning: code built without VTK support, hence ignoring the attempt to plot energies on graphs"
		<< std::endl;
#endif
	}
	if (hasFocusCoordinates) {
		if (recordingScene3DSlicesWithUpdates) {
#ifdef WITH_VTK
			scene3DSliceVisualizer->TriggerBuildFusedCanonical();
#else
			std::cerr << "Warning: code compiled without VTK support, "
						 "hence ignoring the attempt to record scene 3D slices with updates" << std::endl;
#endif
		}
	}
#ifdef WITH_VTK
	focus_point_TSDF_grapher.reset();
#endif
#ifdef WITH_OPENCV
	volume_2D_slice_visualizer.reset();
#endif
	energy_statistics_file.close();
}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::SaveWarps() {
	if (recording3DSceneAndWarpProgression) {
		this->scene3DLogger->SaveCurrentWarpState();
	}
}


template<typename TVoxel, typename TWarp, typename TIndex>
void VolumeSliceVisualizer2D<TVoxel, TWarp, TIndex>::SaveWarpSlices(int iteration) {
	if (hasFocusCoordinates) {
		if (recordingScene2DSlicesWithUpdates) {
#ifdef WITH_OPENCV
			volume_2D_slice_visualizer->RecordWarpUpdates();
#else
			std::cerr <<"Warning: code build without OpenCV support, hence ignoring the attempt to record 2D volume slices"
			<< std::endl;
#endif
		}
		if (recordingScene1DSlicesWithUpdates) {
#ifdef WITH_VTK
			scene1DSliceVisualizer->Plot1DSceneSlice(liveScene, Vector4i(0, 0, 0, 255), 1.0);
			scene1DSliceVisualizer->Draw1DWarpUpdateVector(canonicalScene, warpField, Vector4i(255, 0, 0, 255));
#else
			std::cerr <<"Warning: code build without VTK support, hence ignoring the attempt to record 1D volume slices"
						" with updates" << std::endl;
#endif
		}
		if (recordingScene3DSlicesWithUpdates) {
#ifdef WITH_VTK

			scene3DSliceVisualizer->TriggerDrawWarpUpdates();
			scene3DSliceVisualizer->TriggerUpdateLiveState();
#else
			std::cerr << "Warning: code compiled without VTK support, "
						 "hence ignoring the attempt to record scene 3D slices with updates" << std::endl;
#endif
		}
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::RecordAndPlotEnergies(double totalDataEnergy,
                                                                     double totalLevelSetEnergy,
                                                                     double totalKillingEnergy,
                                                                     double totalSmoothnessEnergy,
                                                                     double totalEnergy) {
	if (this->recordingEnergiesToFile) {
		energy_statistics_file << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
		                       << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;
	}
	if (this->plottingEnergies) {
#ifdef WITH_VTK
		this->energy_plotter->AddDataPoints(static_cast<float>(totalDataEnergy),
		                                    static_cast<float>(totalSmoothnessEnergy),
		                                    static_cast<float>(totalLevelSetEnergy),
		                                    static_cast<float>(totalKillingEnergy));
#else
		std::cerr << "Warning: code built without VTK support, hence ignoring the attempt to plot energies on graphs"
				  << std::endl;
#endif
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool TelemetryRecorder<TVoxel, TWarp, TIndex>::IsRecordingWarp2DSlices() {
	return this->recordingScene2DSlicesWithUpdates;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool TelemetryRecorder<TVoxel, TWarp, TIndex>::IsRecordingWarps() {
	return this->recording3DSceneAndWarpProgression;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::LogHighlight(int hash, int locId,
                                                            ITMHighlightIterationInfo info) {
	scene3DLogger->LogHighlight(hash, locId, 0, info);
}

template<typename TVoxel, typename TWarp, typename TIndex>
TelemetryRecorder<TVoxel, TWarp, TIndex>::~TelemetryRecorder() {
	delete this->scene3DLogger;
}

// endregion ===========================================================================================================
// region ================================ PATH GENERATION =============================================================


template<typename TVoxel, typename TWarp, typename TIndex>
std::string
TelemetryRecorder<TVoxel, TWarp, TIndex>::GetOutputDirectoryFor2DSceneSlicesWithWarps() const {
#ifdef WITH_OPENCV
	fs::path path(fs::path(this->outputDirectory) / (warp_iteration_2D_slices_folder_name + "_" +
	                                                 PlaneToString(this->volume_2D_slice_visualizer->GetPlane())));
	return path.string();
#else
	return "";
#endif
}

template<typename TVoxel, typename TWarp, typename TIndex>
std::string
TelemetryRecorder<TVoxel, TWarp, TIndex>::GetOutputDirectoryFor2DLiveSceneSliceProgression() const {
	fs::path path(fs::path(this->outputDirectory) / (live_iteration_2D_slices_folder_name + "_" +
	                                                 PlaneToString(this->volume_2D_slice_visualizer->GetPlane())));
	return path.string();
}


template<typename TVoxel, typename TWarp, typename TIndex>
std::string
TelemetryRecorder<TVoxel, TWarp, TIndex>::GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / canonical_scene_rasterized_folder_name);
	return path.string();
}

template<typename TVoxel, typename TWarp, typename TIndex>
std::string
TelemetryRecorder<TVoxel, TWarp, TIndex>::GetOutputDirectoryPrefixForLiveSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / live_scene_rasterized_folder_name);
	return path.string();
}


inline
static void ClearDirectory(const fs::path& path) {
	for (fs::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it) {
		fs::remove_all(it->path());
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void TelemetryRecorder<TVoxel, TWarp, TIndex>::MakeOrClearOutputDirectoriesFor2DSceneSlices() const {
	auto ClearIfExistsMakeIfDoesnt = [&](std::string pathString) {
		fs::path path = pathString;
		if (!fs::exists(path)) {
			fs::create_directories(path);
		} else {
			ClearDirectory(path);
		}
	};

	if (recordingLiveSceneAs2DSlices) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryPrefixForLiveSceneAsSlices());
	}
	if (recordingCanonicalSceneAs2DSlices) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryPrefixForCanonicalSceneAsSlices());
	}
	if (recordingScene2DSlicesWithUpdates) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryFor2DLiveSceneSliceProgression());
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryFor2DSceneSlicesWithWarps());
	}
}






// endregion ===========================================================================================================