//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/8/20.
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
#include "TelemetrySettings.h"


namespace ITMLib {

//_DEBUG
//DEFINE_DEFERRABLE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION);

TelemetrySettings::TelemetrySettings() = default;
TelemetrySettings::TelemetrySettings(TelemetrySettings& other, const std::string& parse_path) : TelemetrySettings(
		other) { this->parse_path = parse_path; }
TelemetrySettings::TelemetrySettings(bool record_volume_memory_usage, bool record_live_volume_as_2D_slices, bool record_canonical_volume_as_2D_slices,
                                     bool record_live_focus_point_TSDF_graph, bool record_live_focus_layer_TSDF_heatmap, Plane TSDF_heatmap_plane,
                                     bool record_focus_neighborhood_live_tsdf_sequence, int focus_neighborhood_size,
                                     bool record_focus_neighborhood_warp_sequence, bool record_surface_tracking_optimization_energies,
                                     bool record_frame_meshes, bool use_CPU_for_mesh_recording, bool record_camera_matrices, std::string origin,
                                     std::string parse_path) : record_volume_memory_usage(record_volume_memory_usage),
                                                               record_live_volume_as_2D_slices(record_live_volume_as_2D_slices),
                                                               record_canonical_volume_as_2D_slices(record_canonical_volume_as_2D_slices),
                                                               record_live_focus_point_TSDF_graph(record_live_focus_point_TSDF_graph),
                                                               record_live_focus_layer_TSDF_heatmap(record_live_focus_layer_TSDF_heatmap),
                                                               TSDF_heatmap_plane(TSDF_heatmap_plane), record_focus_neighborhood_live_tsdf_sequence(
				record_focus_neighborhood_live_tsdf_sequence), focus_neighborhood_size(focus_neighborhood_size),
                                                               record_focus_neighborhood_warp_sequence(record_focus_neighborhood_warp_sequence),
                                                               record_surface_tracking_optimization_energies(
		                                                               record_surface_tracking_optimization_energies),
                                                               record_frame_meshes(record_frame_meshes),
                                                               use_CPU_for_mesh_recording(use_CPU_for_mesh_recording),
                                                               record_camera_matrices(record_camera_matrices), origin(std::move(origin)),
                                                               parse_path(std::move(parse_path)) {}
TelemetrySettings::TelemetrySettings(const boost::program_options::variables_map& vm, std::string origin, std::string parse_path)
		: record_volume_memory_usage(vm["record_volume_memory_usage"].empty() ? false : vm["record_volume_memory_usage"].as<bool>()),
		  record_live_volume_as_2D_slices(vm["record_live_volume_as_2D_slices"].empty() ? false : vm["record_live_volume_as_2D_slices"].as<bool>()),
		  record_canonical_volume_as_2D_slices(
				  vm["record_canonical_volume_as_2D_slices"].empty() ? false : vm["record_canonical_volume_as_2D_slices"].as<bool>()),
		  record_live_focus_point_TSDF_graph(
				  vm["record_live_focus_point_TSDF_graph"].empty() ? false : vm["record_live_focus_point_TSDF_graph"].as<bool>()),
		  record_live_focus_layer_TSDF_heatmap(
				  vm["record_live_focus_layer_TSDF_heatmap"].empty() ? false : vm["record_live_focus_layer_TSDF_heatmap"].as<bool>()),
		  TSDF_heatmap_plane(vm["TSDF_heatmap_plane"].empty() ? PLANE_ZX : string_to_enumerator<Plane>(vm["TSDF_heatmap_plane"].as<std::string>())),
		  record_focus_neighborhood_live_tsdf_sequence(
				  vm["record_focus_neighborhood_live_tsdf_sequence"].empty() ? false : vm["record_focus_neighborhood_live_tsdf_sequence"].as<bool>()),
		  focus_neighborhood_size(vm["focus_neighborhood_size"].empty() ? 3 : vm["focus_neighborhood_size"].as<int>()),
		  record_focus_neighborhood_warp_sequence(
				  vm["record_focus_neighborhood_warp_sequence"].empty() ? false : vm["record_focus_neighborhood_warp_sequence"].as<bool>()),
		  record_surface_tracking_optimization_energies(vm["record_surface_tracking_optimization_energies"].empty() ? false
		                                                                                                            : vm["record_surface_tracking_optimization_energies"].as<bool>()),
		  record_frame_meshes(vm["record_frame_meshes"].empty() ? false : vm["record_frame_meshes"].as<bool>()),
		  use_CPU_for_mesh_recording(vm["use_CPU_for_mesh_recording"].empty() ? false : vm["use_CPU_for_mesh_recording"].as<bool>()),
		  record_camera_matrices(vm["record_camera_matrices"].empty() ? false : vm["record_camera_matrices"].as<bool>()), origin(std::move(origin)),
		  parse_path(std::move(parse_path)) {}
void TelemetrySettings::SetFromPTree(const boost::property_tree::ptree& tree) {
	TelemetrySettings temporary_instance = BuildFromPTree(tree);
	*this = temporary_instance;
}
void TelemetrySettings::UpdateFromVariablesMap(const boost::program_options::variables_map& vm) {
	if (!vm["record_volume_memory_usage"].empty())this->record_volume_memory_usage = vm["record_volume_memory_usage"].as<bool>();
	if (!vm["record_live_volume_as_2D_slices"].empty())this->record_live_volume_as_2D_slices = vm["record_live_volume_as_2D_slices"].as<bool>();
	if (!vm["record_canonical_volume_as_2D_slices"].empty())this->record_canonical_volume_as_2D_slices = vm["record_canonical_volume_as_2D_slices"].as<bool>();
	if (!vm["record_live_focus_point_TSDF_graph"].empty())this->record_live_focus_point_TSDF_graph = vm["record_live_focus_point_TSDF_graph"].as<bool>();
	if (!vm["record_live_focus_layer_TSDF_heatmap"].empty())this->record_live_focus_layer_TSDF_heatmap = vm["record_live_focus_layer_TSDF_heatmap"].as<bool>();
	if (!vm["TSDF_heatmap_plane"].empty())this->TSDF_heatmap_plane = string_to_enumerator<Plane>(vm["TSDF_heatmap_plane"].as<std::string>());
	if (!vm["record_focus_neighborhood_live_tsdf_sequence"].empty())this->record_focus_neighborhood_live_tsdf_sequence = vm["record_focus_neighborhood_live_tsdf_sequence"].as<bool>();
	if (!vm["focus_neighborhood_size"].empty())this->focus_neighborhood_size = vm["focus_neighborhood_size"].as<int>();
	if (!vm["record_focus_neighborhood_warp_sequence"].empty())this->record_focus_neighborhood_warp_sequence = vm["record_focus_neighborhood_warp_sequence"].as<bool>();
	if (!vm["record_surface_tracking_optimization_energies"].empty())this->record_surface_tracking_optimization_energies = vm["record_surface_tracking_optimization_energies"].as<bool>();
	if (!vm["record_frame_meshes"].empty())this->record_frame_meshes = vm["record_frame_meshes"].as<bool>();
	if (!vm["use_CPU_for_mesh_recording"].empty())this->use_CPU_for_mesh_recording = vm["use_CPU_for_mesh_recording"].as<bool>();
	if (!vm["record_camera_matrices"].empty())this->record_camera_matrices = vm["record_camera_matrices"].as<bool>();
}
TelemetrySettings
TelemetrySettings::BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin, const std::string& parse_path) {
	TelemetrySettings default_instance;
	boost::optional<bool> record_volume_memory_usage = tree.get_optional<bool>("record_volume_memory_usage");
	boost::optional<bool> record_live_volume_as_2D_slices = tree.get_optional<bool>("record_live_volume_as_2D_slices");
	boost::optional<bool> record_canonical_volume_as_2D_slices = tree.get_optional<bool>("record_canonical_volume_as_2D_slices");
	boost::optional<bool> record_live_focus_point_TSDF_graph = tree.get_optional<bool>("record_live_focus_point_TSDF_graph");
	boost::optional<bool> record_live_focus_layer_TSDF_heatmap = tree.get_optional<bool>("record_live_focus_layer_TSDF_heatmap");
	boost::optional<Plane> TSDF_heatmap_plane = ptree_to_optional_enumerator<Plane>(tree, "TSDF_heatmap_plane");
	boost::optional<bool> record_focus_neighborhood_live_tsdf_sequence = tree.get_optional<bool>("record_focus_neighborhood_live_tsdf_sequence");
	boost::optional<int> focus_neighborhood_size = tree.get_optional<int>("focus_neighborhood_size");
	boost::optional<bool> record_focus_neighborhood_warp_sequence = tree.get_optional<bool>("record_focus_neighborhood_warp_sequence");
	boost::optional<bool> record_surface_tracking_optimization_energies = tree.get_optional<bool>("record_surface_tracking_optimization_energies");
	boost::optional<bool> record_frame_meshes = tree.get_optional<bool>("record_frame_meshes");
	boost::optional<bool> use_CPU_for_mesh_recording = tree.get_optional<bool>("use_CPU_for_mesh_recording");
	boost::optional<bool> record_camera_matrices = tree.get_optional<bool>("record_camera_matrices");
	TelemetrySettings instance = {record_volume_memory_usage ? record_volume_memory_usage.get() : default_instance.record_volume_memory_usage,
	                              record_live_volume_as_2D_slices ? record_live_volume_as_2D_slices.get()
	                                                              : default_instance.record_live_volume_as_2D_slices,
	                              record_canonical_volume_as_2D_slices ? record_canonical_volume_as_2D_slices.get()
	                                                                   : default_instance.record_canonical_volume_as_2D_slices,
	                              record_live_focus_point_TSDF_graph ? record_live_focus_point_TSDF_graph.get()
	                                                                 : default_instance.record_live_focus_point_TSDF_graph,
	                              record_live_focus_layer_TSDF_heatmap ? record_live_focus_layer_TSDF_heatmap.get()
	                                                                   : default_instance.record_live_focus_layer_TSDF_heatmap,
	                              TSDF_heatmap_plane ? TSDF_heatmap_plane.get() : default_instance.TSDF_heatmap_plane,
	                              record_focus_neighborhood_live_tsdf_sequence ? record_focus_neighborhood_live_tsdf_sequence.get()
	                                                                           : default_instance.record_focus_neighborhood_live_tsdf_sequence,
	                              focus_neighborhood_size ? focus_neighborhood_size.get() : default_instance.focus_neighborhood_size,
	                              record_focus_neighborhood_warp_sequence ? record_focus_neighborhood_warp_sequence.get()
	                                                                      : default_instance.record_focus_neighborhood_warp_sequence,
	                              record_surface_tracking_optimization_energies ? record_surface_tracking_optimization_energies.get()
	                                                                            : default_instance.record_surface_tracking_optimization_energies,
	                              record_frame_meshes ? record_frame_meshes.get() : default_instance.record_frame_meshes,
	                              use_CPU_for_mesh_recording ? use_CPU_for_mesh_recording.get() : default_instance.use_CPU_for_mesh_recording,
	                              record_camera_matrices ? record_camera_matrices.get() : default_instance.record_camera_matrices, origin};
	instance.source_tree = tree;
	return instance;
}
boost::property_tree::ptree TelemetrySettings::ToPTree(const std::string& _origin) const {
	boost::property_tree::ptree tree;
	tree.add("record_volume_memory_usage", record_volume_memory_usage);
	tree.add("record_live_volume_as_2D_slices", record_live_volume_as_2D_slices);
	tree.add("record_canonical_volume_as_2D_slices", record_canonical_volume_as_2D_slices);
	tree.add("record_live_focus_point_TSDF_graph", record_live_focus_point_TSDF_graph);
	tree.add("record_live_focus_layer_TSDF_heatmap", record_live_focus_layer_TSDF_heatmap);
	tree.add("TSDF_heatmap_plane", enumerator_to_string(TSDF_heatmap_plane));
	tree.add("record_focus_neighborhood_live_tsdf_sequence", record_focus_neighborhood_live_tsdf_sequence);
	tree.add("focus_neighborhood_size", focus_neighborhood_size);
	tree.add("record_focus_neighborhood_warp_sequence", record_focus_neighborhood_warp_sequence);
	tree.add("record_surface_tracking_optimization_energies", record_surface_tracking_optimization_energies);
	tree.add("record_frame_meshes", record_frame_meshes);
	tree.add("use_CPU_for_mesh_recording", use_CPU_for_mesh_recording);
	tree.add("record_camera_matrices", record_camera_matrices);
	return tree;
}
bool operator==(const TelemetrySettings& instance1, const TelemetrySettings& instance2) {
	return instance1.record_volume_memory_usage == instance2.record_volume_memory_usage &&
	       instance1.record_live_volume_as_2D_slices == instance2.record_live_volume_as_2D_slices &&
	       instance1.record_canonical_volume_as_2D_slices == instance2.record_canonical_volume_as_2D_slices &&
	       instance1.record_live_focus_point_TSDF_graph == instance2.record_live_focus_point_TSDF_graph &&
	       instance1.record_live_focus_layer_TSDF_heatmap == instance2.record_live_focus_layer_TSDF_heatmap &&
	       instance1.TSDF_heatmap_plane == instance2.TSDF_heatmap_plane &&
	       instance1.record_focus_neighborhood_live_tsdf_sequence == instance2.record_focus_neighborhood_live_tsdf_sequence &&
	       instance1.focus_neighborhood_size == instance2.focus_neighborhood_size &&
	       instance1.record_focus_neighborhood_warp_sequence == instance2.record_focus_neighborhood_warp_sequence &&
	       instance1.record_surface_tracking_optimization_energies == instance2.record_surface_tracking_optimization_energies &&
	       instance1.record_frame_meshes == instance2.record_frame_meshes &&
	       instance1.use_CPU_for_mesh_recording == instance2.use_CPU_for_mesh_recording &&
	       instance1.record_camera_matrices == instance2.record_camera_matrices;
}
std::ostream& operator<<(std::ostream& out, const TelemetrySettings& instance) {
	boost::property_tree::ptree tree(instance.ToPTree());
	boost::property_tree::write_json_no_quotes(out, tree, true);
	return out;
}
void TelemetrySettings::AddToOptionsDescription(boost::program_options::options_description& od) {
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_volume_memory_usage", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record information required to debug memory" " usage, e.g. used block locations for the VoxelBlockHash index.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_live_volume_as_2D_slices", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record 2D slices (images, with pixel " "representing TSDF value) of the live volume once per frame before the beginning of the surface tracking optimization.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_canonical_volume_as_2D_slices", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record 2D slices (images, with pixel " "representing TSDF value) of the canonical volume once per frame before the beginning of the surface tracking optimization.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_live_focus_point_TSDF_graph", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record graphs of SDF value of a single " "voxel plotted against iteration number of the surface tracking optimization.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_live_focus_layer_TSDF_heatmap", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record TSDF heatmaps for the voxel layer in " "the warped live volume (in the plane specified by TSDF_heatmap_plane parameter) for each iteration of the surface" "tracking optimization.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("TSDF_heatmap_plane", od).c_str(),
	                 boost::program_options::value<std::string>()->default_value(enumerator_to_string(PLANE_ZX)),
	                 (std::string("\"Plane in which to record TSDF heatmaps \" \"(see record_live_focus_layer_TSDF_heatmap parameter help).\"") +
	                  enumerator_bracketed_list<Plane>()).c_str());
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_focus_neighborhood_live_tsdf_sequence", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record a sequence of TSDF " "volumes representing the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) " "in the warped live frame over the course of the entire surface tracking optimization. [WITH_VTK compilation required!]");
	od.add_options()(generate_cli_argument_identifiers_snake_case("focus_neighborhood_size", od).c_str(),
	                 boost::program_options::value<int>()->default_value(3),
	                 "For focus neighborhood recording: a cube of size " "2 x focus_neighborhood_size + 1 around the focus_coordinates will be recorded. [WITH_VTK compilation required!]");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_focus_neighborhood_warp_sequence", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record a sequence of warp " "vectors in the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) over " "the course of the entire surface tracking optimization. [WITH_VTK compilation required!]");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_surface_tracking_optimization_energies", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record optimization energies " "for each iteration of the surface tracking optimization in a separate file. Only works when non_rigid_tracking_parameters.functor_type " "parameter is set to \"slavcheva_diagnostic\"");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_frame_meshes", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to log three meshes at every frame: (a) from live volume " "after camera tracking and before surface tracking, (b) from live volume after surface tracking, and (c) from " "canonical volume after fusion.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("use_CPU_for_mesh_recording", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to ALWAYS use CPU & regular RAM when recording mesh telemetry. For CUDA runs, this will reduce GPU memory usage.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_camera_matrices", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record estimated camera trajectory matrices in world space.");
}

} // namespace ITMLib