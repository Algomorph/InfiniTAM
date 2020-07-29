//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/29/20.
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
#include "VoxelVolumeParameters.h"
namespace ITMLib {
//_DEBUG
//DEFINE_SERIALIZABLE_STRUCT(VOXEL_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

VoxelVolumeParameters::VoxelVolumeParameters() = default;
VoxelVolumeParameters::VoxelVolumeParameters(float voxel_size, float near_clipping_distance, float far_clipping_distance, float truncation_distance,
                                             int max_integration_weight, bool stop_integration_at_max_weight, float block_allocation_band_factor,
                                             std::string origin) : voxel_size(voxel_size), near_clipping_distance(near_clipping_distance),
                                                                   far_clipping_distance(far_clipping_distance),
                                                                   truncation_distance(truncation_distance),
                                                                   max_integration_weight(max_integration_weight),
                                                                   stop_integration_at_max_weight(stop_integration_at_max_weight),
                                                                   block_allocation_band_factor(block_allocation_band_factor),
                                                                   origin(std::move(origin)) {}
void VoxelVolumeParameters::SetFromPTree(const boost::property_tree::ptree& tree) {
	VoxelVolumeParameters temporary_instance = BuildFromPTree(tree);
	*this = temporary_instance;
}
VoxelVolumeParameters VoxelVolumeParameters::BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin) {
	VoxelVolumeParameters default_instance;
	boost::optional<float> voxel_size = tree.get_optional<float>("voxel_size");
	boost::optional<float> near_clipping_distance = tree.get_optional<float>("near_clipping_distance");
	boost::optional<float> far_clipping_distance = tree.get_optional<float>("far_clipping_distance");
	boost::optional<float> truncation_distance = tree.get_optional<float>("truncation_distance");
	boost::optional<int> max_integration_weight = tree.get_optional<int>("max_integration_weight");
	boost::optional<bool> stop_integration_at_max_weight = tree.get_optional<bool>("stop_integration_at_max_weight");
	boost::optional<float> block_allocation_band_factor = tree.get_optional<float>("block_allocation_band_factor");
	VoxelVolumeParameters instance = {voxel_size ? voxel_size.get() : default_instance.voxel_size,
	                                  near_clipping_distance ? near_clipping_distance.get() : default_instance.near_clipping_distance,
	                                  far_clipping_distance ? far_clipping_distance.get() : default_instance.far_clipping_distance,
	                                  truncation_distance ? truncation_distance.get() : default_instance.truncation_distance,
	                                  max_integration_weight ? max_integration_weight.get() : default_instance.max_integration_weight,
	                                  stop_integration_at_max_weight ? stop_integration_at_max_weight.get()
	                                                                 : default_instance.stop_integration_at_max_weight,
	                                  block_allocation_band_factor ? block_allocation_band_factor.get()
	                                                               : default_instance.block_allocation_band_factor, origin};
	instance.source_tree = tree;
	return instance;
}
boost::property_tree::ptree VoxelVolumeParameters::ToPTree(const std::string& _origin) const {
	boost::property_tree::ptree tree;
	tree.add("voxel_size", voxel_size);
	tree.add("near_clipping_distance", near_clipping_distance);
	tree.add("far_clipping_distance", far_clipping_distance);
	tree.add("truncation_distance", truncation_distance);
	tree.add("max_integration_weight", max_integration_weight);
	tree.add("stop_integration_at_max_weight", stop_integration_at_max_weight);
	tree.add("block_allocation_band_factor", block_allocation_band_factor);
	return tree;
}
bool operator==(const VoxelVolumeParameters& instance1, const VoxelVolumeParameters& instance2) {
	return instance1.voxel_size == instance2.voxel_size && instance1.near_clipping_distance == instance2.near_clipping_distance &&
	       instance1.far_clipping_distance == instance2.far_clipping_distance && instance1.truncation_distance == instance2.truncation_distance &&
	       instance1.max_integration_weight == instance2.max_integration_weight &&
	       instance1.stop_integration_at_max_weight == instance2.stop_integration_at_max_weight &&
	       instance1.block_allocation_band_factor == instance2.block_allocation_band_factor;
}
std::ostream& operator<<(std::ostream& out, const VoxelVolumeParameters& instance) {
	boost::property_tree::ptree tree(instance.ToPTree());
	boost::property_tree::write_json_no_quotes(out, tree, true);
	return out;
}
VoxelVolumeParameters::VoxelVolumeParameters(const boost::program_options::variables_map& vm, const std::string& parent_long_identifier,
                                             std::string origin) : voxel_size(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "voxel_size")].empty() ? 0.004f : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "voxel_size")].as<float>()), near_clipping_distance(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "near_clipping_distance")].empty() ? 0.2f : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "near_clipping_distance")].as<float>()), far_clipping_distance(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "far_clipping_distance")].empty() ? 3.0f : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "far_clipping_distance")].as<float>()), truncation_distance(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "truncation_distance")].empty() ? 0.04f : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "truncation_distance")].as<float>()), max_integration_weight(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "max_integration_weight")].empty() ? 100 : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "max_integration_weight")].as<int>()), stop_integration_at_max_weight(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "stop_integration_at_max_weight")].empty() ? false
		                                                                                                    : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "stop_integration_at_max_weight")].as<bool>()), block_allocation_band_factor(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "block_allocation_band_factor")].empty() ? 2.0f : vm[compile_sub_struct_parse_path(
				parent_long_identifier, "block_allocation_band_factor")].as<float>()), origin(std::move(origin)) {}
void VoxelVolumeParameters::UpdateFromVariablesMap(const boost::program_options::variables_map& vm, const std::string& parent_long_identifier) {
	std::string long_identifier;
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "voxel_size");
	if (!vm[long_identifier].empty())this->voxel_size = vm[long_identifier].as<float>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "near_clipping_distance");
	if (!vm[long_identifier].empty())this->near_clipping_distance = vm[long_identifier].as<float>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "far_clipping_distance");
	if (!vm[long_identifier].empty())this->far_clipping_distance = vm[long_identifier].as<float>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "truncation_distance");
	if (!vm[long_identifier].empty()){
		this->truncation_distance = vm[long_identifier].as<float>();
	}
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "max_integration_weight");
	if (!vm[long_identifier].empty()){
		this->max_integration_weight = vm[long_identifier].as<int>();
	}
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "stop_integration_at_max_weight");
	if (!vm[long_identifier].empty()){
		this->stop_integration_at_max_weight = vm[long_identifier].as<bool>();
	}
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "block_allocation_band_factor");
	if (!vm[long_identifier].empty()){
		this->block_allocation_band_factor = vm[long_identifier].as<float>();
	}
}
void VoxelVolumeParameters::AddToOptionsDescription(boost::program_options::options_description& od, const std::string& long_identifier,
                                                    const std::string& short_identifier) {
	std::string field_long_identifier, field_short_identifier;
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "voxel_size", field_long_identifier, field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<float>()->default_value(0.004f),
	                 "Size of a voxel, usually given in meters");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "near_clipping_distance", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<float>()->default_value(0.2f),
	                 "Consider only depth values between near_clipping_distance to far_clipping_distance.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "far_clipping_distance", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<float>()->default_value(3.0f),
	                 "Consider only depth values between near_clipping_distance to far_clipping_distance.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "truncation_distance", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<float>()->default_value(0.04f),
	                 "Encodes the width of the band of the truncated signed distance transform that is actually stored in the " "volume. This is again usually specified in meters. " "The resulting width in voxels istruncation_distance divided by voxel_size.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "max_integration_weight", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<int>()->default_value(100),
	                 "Up to max_integration_weight observations per voxel are averaged. " "Beyond that a sliding average is computed.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "stop_integration_at_max_weight", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<bool>()->default_value(false),
	                 "Whether to stop integration for a given voxel when its max_integration_weight is reached.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "block_allocation_band_factor", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<float>()->default_value(2.0f),
	                 "(Voxel block hash indexing only) factor of narrow band width that will be " "considered for depth-based allocation. For instance, a factor of 2 will make " "sure that blocks that are twice as far from the surface as the boundary of the " "narrow (non-truncated) TSDF band will be allocated");
}

} // namespace ITMLib
