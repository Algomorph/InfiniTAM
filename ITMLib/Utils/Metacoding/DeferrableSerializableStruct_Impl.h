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
#pragma once

#include "SerializableStruct_Impl.h"

#ifndef MACRO_END
#define MACRO_END() static_assert(true, "")
#define ITM_METACODING_OUTER_EXPAND(x) x
#endif

// ==== template functions / utilities ====

std::string generate_cli_argument_short_identifier_from_long_identifier(const std::string& long_identifier);

template<typename TDeferrableSerializableStruct>
static TDeferrableSerializableStruct
ExtractDeferrableSerializableStructFromPtreeIfPresent(const pt::ptree& tree, const std::string& origin) {
	return ExtractSerializableStructFromPtreeIfPresent<TDeferrableSerializableStruct>(tree, TDeferrableSerializableStruct::default_parse_path, origin);
}

// region ======== MACROS ==========

// *** DECLARATION-ONLY ***
#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL(struct_name, parse_path, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, parse_path, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, parse_path, field_count, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, parse_path, ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_DEFERRED_FUNCS(struct_name) \
    static void BuildDeferredFromVariablesMap (boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm); \
    static void UpdateDeferredFromVariablesMap(boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm, const std::string& origin = ""); \
    static void AddDeferredToOptionsDescription(boost::program_options::options_description& od);

#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, parse_path, loop, ...) \
    struct struct_name { \
        static constexpr const char* default_parse_path = parse_path; \
        ORIGIN_AND_SOURCE_TREE() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_INDEPENDENT_FUNCS(struct_name, loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_DEPENDENT_FUNCS(struct_name) \
        DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_DEFERRED_FUNCS(struct_name) \
    }

// *** DEFINITION-ONLY ***

#define DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL(outer_class, struct_name, parse_path, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_2( outer_class, struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_2(outer_class, struct_name, field_count, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_3(SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(outer_class), \
                             SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(struct_name), struct_name, instance.source_tree=tree, , ,\
                             ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, , \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_DEFERRED(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
				                                          friend_qualifier, static_qualifier, \
				                                          ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
				                                          loop, ...) \
	static_qualifier void outer_class inner_qualifier BuildDeferredFromVariablesMap(boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm){\
		struct_name deferrable (vm, struct_name :: default_parse_path);\
		auto deferrable_subtree = deferrable.ToPTree("");\
		target_tree.add_child(struct_name :: default_parse_path, deferrable_subtree);\
	}\
	static_qualifier void outer_class inner_qualifier UpdateDeferredFromVariablesMap(boost::property_tree::ptree& target_tree, const boost::program_options::variables_map& vm, const std::string& origin default_string_arg){\
		struct_name deferrable = ExtractDeferrableSerializableStructFromPtreeIfPresent< struct_name > (target_tree, origin);\
		deferrable.UpdateFromVariablesMap(vm, struct_name :: default_parse_path);\
		auto deferrable_subtree = deferrable.ToPTree(origin);\
		target_tree.add_child(struct_name :: default_parse_path, deferrable_subtree);\
	}\
	static_qualifier void outer_class inner_qualifier AddDeferredToOptionsDescription(boost::program_options::options_description& od){\
		auto short_identifier = generate_cli_argument_short_identifier_from_long_identifier(struct_name :: default_parse_path); \
		auto& long_identifier = struct_name :: default_parse_path; \
		struct_name :: AddToOptionsDescription(od, long_identifier, short_identifier); \
	}


#define DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_3(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                                   friend_qualifier, static_qualifier, \
                                                   ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                                   loop, ...) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_INDEPENDENT(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                                   friend_qualifier, static_qualifier, \
                                                   ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                                   loop, __VA_ARGS__) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_DEPENDENT(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                                 friend_qualifier, static_qualifier, \
                                                 ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                                 loop, __VA_ARGS__) \
	DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL_DEFERRED(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                                      friend_qualifier, static_qualifier, \
                                                      ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                                      loop, __VA_ARGS__) \

// *** FULL STRUCT GENERATION ***
#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL(struct_name, parse_path, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_2(struct_name, parse_path, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_2(struct_name, parse_path, field_count, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_3(struct_name, parse_path, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_3(struct_name, parse_path, loop, ...) \
    struct struct_name { \
        static constexpr const char* default_parse_path = parse_path;\
        PARSE_PATH() \
        ORIGIN_AND_SOURCE_TREE() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_INDEPENDENT ( , , struct_name, default_instance.source_tree=tree, friend, static, \
                                                        ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, ="", \
                                                        loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_DEPENDENT ( , , struct_name, default_instance.source_tree=tree, friend, static, \
                                                      ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, ="", \
                                                      loop, __VA_ARGS__) \
    };

// === endregion
