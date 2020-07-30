//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/9/20.
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
//local
#include "AutomaticRunSettings.h"
namespace ITMLib {

//__DEBUG
//DEFINE_DEFERRABLE_SERIALIZABLE_STRUCT(AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION);

AutomaticRunSettings::AutomaticRunSettings() = default;
AutomaticRunSettings::AutomaticRunSettings(int index_of_frame_to_end_before, int index_of_frame_to_start_at,
                                           bool load_volume_and_camera_matrix_before_processing, bool save_volumes_and_camera_matrix_after_processing,
                                           bool save_meshes_after_processing, bool exit_after_automatic_processing, std::string origin)
		: index_of_frame_to_end_before(index_of_frame_to_end_before), index_of_frame_to_start_at(index_of_frame_to_start_at),
		  load_volume_and_camera_matrix_before_processing(load_volume_and_camera_matrix_before_processing),
		  save_volumes_and_camera_matrix_after_processing(save_volumes_and_camera_matrix_after_processing),
		  save_meshes_after_processing(save_meshes_after_processing), exit_after_automatic_processing(exit_after_automatic_processing),
		  origin(std::move(origin)) {}
void AutomaticRunSettings::SetFromPTree(const boost::property_tree::ptree& tree) {
	AutomaticRunSettings temporary_instance = BuildFromPTree(tree);
	*this = temporary_instance;
}
AutomaticRunSettings AutomaticRunSettings::BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin) {
	AutomaticRunSettings default_instance;
	boost::optional<int> index_of_frame_to_end_before = tree.get_optional<int>("index_of_frame_to_end_before");
	boost::optional<int> index_of_frame_to_start_at = tree.get_optional<int>("index_of_frame_to_start_at");
	boost::optional<bool> load_volume_and_camera_matrix_before_processing = tree.get_optional<bool>(
			"load_volume_and_camera_matrix_before_processing");
	boost::optional<bool> save_volumes_and_camera_matrix_after_processing = tree.get_optional<bool>(
			"save_volumes_and_camera_matrix_after_processing");
	boost::optional<bool> save_meshes_after_processing = tree.get_optional<bool>("save_meshes_after_processing");
	boost::optional<bool> exit_after_automatic_processing = tree.get_optional<bool>("exit_after_automatic_processing");
	AutomaticRunSettings instance = {
			index_of_frame_to_end_before ? index_of_frame_to_end_before.get() : default_instance.index_of_frame_to_end_before,
			index_of_frame_to_start_at ? index_of_frame_to_start_at.get() : default_instance.index_of_frame_to_start_at,
			load_volume_and_camera_matrix_before_processing ? load_volume_and_camera_matrix_before_processing.get()
			                                                : default_instance.load_volume_and_camera_matrix_before_processing,
			save_volumes_and_camera_matrix_after_processing ? save_volumes_and_camera_matrix_after_processing.get()
			                                                : default_instance.save_volumes_and_camera_matrix_after_processing,
			save_meshes_after_processing ? save_meshes_after_processing.get() : default_instance.save_meshes_after_processing,
			exit_after_automatic_processing ? exit_after_automatic_processing.get() : default_instance.exit_after_automatic_processing, origin};
	instance.source_tree = tree;
	return instance;
}
boost::property_tree::ptree AutomaticRunSettings::ToPTree(const std::string& _origin) const {
	boost::property_tree::ptree tree;
	tree.add("index_of_frame_to_end_before", index_of_frame_to_end_before);
	tree.add("index_of_frame_to_start_at", index_of_frame_to_start_at);
	tree.add("load_volume_and_camera_matrix_before_processing", load_volume_and_camera_matrix_before_processing);
	tree.add("save_volumes_and_camera_matrix_after_processing", save_volumes_and_camera_matrix_after_processing);
	tree.add("save_meshes_after_processing", save_meshes_after_processing);
	tree.add("exit_after_automatic_processing", exit_after_automatic_processing);
	return tree;
}
bool operator==(const AutomaticRunSettings& instance1, const AutomaticRunSettings& instance2) {
	return instance1.index_of_frame_to_end_before == instance2.index_of_frame_to_end_before &&
	       instance1.index_of_frame_to_start_at == instance2.index_of_frame_to_start_at &&
	       instance1.load_volume_and_camera_matrix_before_processing == instance2.load_volume_and_camera_matrix_before_processing &&
	       instance1.save_volumes_and_camera_matrix_after_processing == instance2.save_volumes_and_camera_matrix_after_processing &&
	       instance1.save_meshes_after_processing == instance2.save_meshes_after_processing &&
	       instance1.exit_after_automatic_processing == instance2.exit_after_automatic_processing;
}
std::ostream& operator<<(std::ostream& out, const AutomaticRunSettings& instance) {
	boost::property_tree::ptree tree(instance.ToPTree());
	boost::property_tree::write_json_no_quotes(out, tree, true);
	return out;
}
AutomaticRunSettings::AutomaticRunSettings(const boost::program_options::variables_map& vm, const std::string& parent_long_identifier,
                                           std::string origin) : index_of_frame_to_end_before(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "index_of_frame_to_end_before")].as<int>()), index_of_frame_to_start_at(
		vm[compile_sub_struct_parse_path(parent_long_identifier, "index_of_frame_to_start_at")].as<int>()),
                                                                 load_volume_and_camera_matrix_before_processing(
		                                                                 vm[compile_sub_struct_parse_path(parent_long_identifier,
		                                                                                                  "load_volume_and_camera_matrix_before_processing")].as<bool>()),
                                                                 save_volumes_and_camera_matrix_after_processing(
		                                                                 vm[compile_sub_struct_parse_path(parent_long_identifier,
		                                                                                                  "save_volumes_and_camera_matrix_after_processing")].as<bool>()),
                                                                 save_meshes_after_processing(vm[compile_sub_struct_parse_path(parent_long_identifier,
                                                                                                                               "save_meshes_after_processing")].as<bool>()),
                                                                 exit_after_automatic_processing(
		                                                                 vm[compile_sub_struct_parse_path(parent_long_identifier,
		                                                                                                  "exit_after_automatic_processing")].as<bool>()),
                                                                 origin(std::move(origin)) {}
void AutomaticRunSettings::UpdateFromVariablesMap(const boost::program_options::variables_map& vm, const std::string& parent_long_identifier) {
	std::string long_identifier;
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "index_of_frame_to_end_before");
	if (!vm[long_identifier].defaulted())this->index_of_frame_to_end_before = vm[long_identifier].as<int>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "index_of_frame_to_start_at");
	if (!vm[long_identifier].defaulted())this->index_of_frame_to_start_at = vm[long_identifier].as<int>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "load_volume_and_camera_matrix_before_processing");
	if (!vm[long_identifier].defaulted())this->load_volume_and_camera_matrix_before_processing = vm[long_identifier].as<bool>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "save_volumes_and_camera_matrix_after_processing");
	if (!vm[long_identifier].defaulted())this->save_volumes_and_camera_matrix_after_processing = vm[long_identifier].as<bool>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "save_meshes_after_processing");
	if (!vm[long_identifier].defaulted())this->save_meshes_after_processing = vm[long_identifier].as<bool>();
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, "exit_after_automatic_processing");
	if (!vm[long_identifier].defaulted())this->exit_after_automatic_processing = vm[long_identifier].as<bool>();
}
void AutomaticRunSettings::AddToOptionsDescription(boost::program_options::options_description& od, const std::string& long_identifier,
                                                   const std::string& short_identifier) {
	std::string field_long_identifier, field_short_identifier;
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "index_of_frame_to_end_before", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<int>()->default_value(0),
	                 "This number of frames will be processed automatically after the program is launched (launches automatic run).");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "index_of_frame_to_start_at", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<int>()->default_value(0),
	                 "Index of the first frame (or frame set) to read from disk (or, how many frames to skip). The remaining frames will be read in order.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "load_volume_and_camera_matrix_before_processing",
	                                             field_long_identifier, field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<bool>()->default_value(false),
	                 "When this is set to true, the program will attempt to load the volume for the index_of_frame_to_start_with from the corresponding subfolder within output_folder.");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "save_volumes_and_camera_matrix_after_processing",
	                                             field_long_identifier, field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<bool>()->default_value(false),
	                 "Whether to save volume(s) after automatic processing");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "save_meshes_after_processing", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<bool>()->default_value(false),
	                 "Whether to save result mesh(es) after automatic processing");
	generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, "exit_after_automatic_processing", field_long_identifier,
	                                             field_short_identifier);
	od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), boost::program_options::value<bool>()->default_value(false),
	                 "Whether to exit the program after the automatic run.");
}
void AutomaticRunSettings::BuildDeferredFromVariablesMap(boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm) {
	AutomaticRunSettings deferrable(vm, AutomaticRunSettings::default_parse_path);
	auto deferrable_subtree = deferrable.ToPTree("");
	target_tree.add_child(AutomaticRunSettings::default_parse_path, deferrable_subtree);
}
void AutomaticRunSettings::UpdateDeferredFromVariablesMap(boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm,
                                                          const std::string& origin) {
	AutomaticRunSettings deferrable = ExtractDeferrableSerializableStructFromPtreeIfPresent<AutomaticRunSettings>(target_tree, origin);
	deferrable.UpdateFromVariablesMap(vm, AutomaticRunSettings::default_parse_path);
	auto deferrable_subtree = deferrable.ToPTree(origin);
	if(target_tree.get_child_optional(AutomaticRunSettings::default_parse_path)){
		target_tree.erase(AutomaticRunSettings::default_parse_path);
	}
	target_tree.add_child(AutomaticRunSettings::default_parse_path, deferrable_subtree);
}
void AutomaticRunSettings::AddDeferredToOptionsDescription(boost::program_options::options_description& od) {
	auto short_identifier = generate_cli_argument_short_identifier_from_long_identifier(AutomaticRunSettings::default_parse_path);
	auto& long_identifier = AutomaticRunSettings::default_parse_path;
	AutomaticRunSettings::AddToOptionsDescription(od, long_identifier, short_identifier);
}

} // namespace ITMLib