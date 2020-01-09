//  ================================================================
//  Created by Gregory Kramida on 1/2/20.
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
#include <unordered_map>

//boost
#include <boost/program_options/variables_map.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>
#include <boost/preprocessor/stringize.hpp>

// local
#include "SerializationSequenceMacros.h"
#include "../json_utils.h"
#include "../../../ORUtils/PlatformIndependence.h"

// region ==== LOW-LEVEL MACROS
#define ITM_SERIALIZATION_IMPL_SEMICOLON() ;
#define ITM_SERIALIZATION_IMPL_COMMA() ,
#define ITM_SERIALIZATION_IMPL_AND() &&
#define ITM_SERIALIZATION_IMPL_NOTHING()

#define ITM_SERIALIZATION_IMPL_NARG(...)                                                                               \
  ITM_SERIALIZATION_IMPL_NARG_(__VA_ARGS__, ITM_SERIALIZATION_IMPL_RSEQ_N())
#define ITM_SERIALIZATION_IMPL_NARG_(...) ITM_SERIALIZATION_IMPL_ARG_N(__VA_ARGS__)

#define ITM_SERIALIZATION_IMPL_PRIMITIVE_CAT(a, ...) a##__VA_ARGS__
#define ITM_SERIALIZATION_IMPL_CAT(a, ...) ITM_SERIALIZATION_IMPL_PRIMITIVE_CAT(a, __VA_ARGS__)

// endregion

// region ==== SERIALIZABLE STRUCT FUNCTION DEFINITIONS ==================

template<typename TSerializableStruct>
static boost::optional<TSerializableStruct>
ptree_to_optional_serializable_struct(const pt::ptree& tree, pt::ptree::key_type const& key,
                                 const std::string& origin) {
	auto subtree = tree.get_child_optional(key);
	if (subtree) {
		return boost::optional<TSerializableStruct>(TSerializableStruct::BuildFromPTree(subtree.get(), origin));
	} else {
		return boost::optional<TSerializableStruct>{};
	}
}

// endregion	
// region ================== SERIALIZABLE ENUM TEMPLATED FUNCTION DEFINITIONS ===

template<typename TEnum>
static TEnum string_to_enumerator(const std::string& string);

template<typename TEnum>
static std::string enumerator_to_string(const TEnum& enum_value);

template<typename TEnum>
static TEnum variables_map_to_enumerator(const boost::program_options::variables_map& vm, const std::string& argument){
	return string_to_enumerator<TEnum>(vm[argument].as<std::string>());
}

template<typename TEnum>
static boost::optional<TEnum> ptree_to_optional_enumerator(const pt::ptree& ptree, const pt::ptree::key_type& key) {
	auto child = ptree.get_child_optional(key);
	if (child) {
		return boost::optional<TEnum>(string_to_enumerator<TEnum>(ptree.get<std::string>(key)));
	} else {
		return boost::optional<TEnum>{};
	}
}

// endregion

// region ================== SERIALIZABLE PATH FUNCTION DECLARATIONS ==============

std::string preprocess_path(const std::string& path, const std::string& origin);

std::string postprocess_path(const std::string& path, const std::string& origin);

boost::optional<std::string> ptree_to_optional_path(const boost::property_tree::ptree& tree, const pt::ptree::key_type& key, const std::string& origin );

// endregion
// region ================== SERIALIZABLE VECTOR FUNCTION DEFINITIONS ===========
template <typename TVector>
TVector std_vector_to_serializable_vector(const std::vector<typename TVector::value_type>& std_vector){
	TVector vector;
	if(std_vector.size() != TVector::size()){
		DIEWITHEXCEPTION_REPORTLOCATION("Wrong number of elements in parsed vector.");
	}
	memcpy(vector.values, std_vector.data(), sizeof(typename TVector::value_type) * TVector::size());
	return vector;
}

template <typename TVector>
TVector variables_map_to_vector(const boost::program_options::variables_map& vm, const std::string& argument){
	std::vector<typename TVector::value_type> std_vector = vm[argument].as<std::vector<typename TVector::value_type>>();
	return std_vector_to_serializable_vector<TVector>(std_vector);
}

template <typename TVector>
boost::optional<TVector> ptree_to_optional_serializable_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	TVector vector;
	if (pt.count(key) == 0) {
		return boost::optional<TVector>{};
	}
	std::vector<typename TVector::value_type> std_vector;
	for (auto& item : pt.get_child(key)){
		std_vector.push_back(item.second.get_value<typename TVector::value_type>());
	}
	return std_vector_to_serializable_vector<TVector>(std_vector);;
}
template <typename TVector>
boost::property_tree::ptree serializable_vector_to_ptree(TVector vector){
	boost::property_tree::ptree tree;
	for (int i_element = 0; i_element < TVector::size(); i_element++){
		boost::property_tree::ptree child;
		child.put("", vector.values[i_element]);
		tree.push_back(std::make_pair("",child));
	}
	return tree;
}

// region ===== SERIALIZABLE STRUCT PER-FIELD MACROS ===========

// *** used to declare fields & defaults ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_DECL(_, type, field_name, default_value, serialization_type) type field_name = default_value;

// *** used for a generic constructor that contains all fields ***
#define SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD(_, type, field_name, ...) type field_name
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG(_, type, field_name, ...) field_name ( std::move(field_name ))

// *** variables_map --> value ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_PRIMITIVE(struct_name, type, field_name, default_value)                 \
  	field_name(vm[ #field_name ].empty() ? struct_name (). field_name : vm[#field_name].as<type>())
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_PATH(struct_name, type, field_name, default_value)                      \
    field_name(vm[ #field_name ].empty() ? struct_name (). field_name : preprocess_path(vm[ #field_name ].as<std::string>(), origin))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_ENUM(struct_name, type, field_name, default_value)                      \
  	field_name(vm[ #field_name ].empty() ? struct_name (). field_name : string_to_enumerator< type >(vm[ #field_name ].as<std::string>()))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_STRUCT(struct_name, type, field_name, default_value)                    \
  	field_name(vm)
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_VECTOR(struct_name, type, field_name, default_value)                   \
  	field_name(vm[ #field_name ].empty() ? struct_name (). field_name : variables_map_to_vector <type> (vm, #field_name))

#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT(struct_name, type, field_name, default_value, serialization_type)       \
	ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_, serialization_type)(struct_name, type, field_name, default_value)

// *** ptree --> value ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PRIMITIVE(type, field_name, default_value)                   \
	boost::optional< type > field_name = tree.get_optional< type > ( #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PATH(type, field_name, default_value)                        \
	boost::optional< type > field_name = ptree_to_optional_path( tree, #field_name, origin );	
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_ENUM(type, field_name, default_value)                        \
	boost::optional< type > field_name = ptree_to_optional_enumerator< tree >( tree, #field_name );	
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_STRUCT(type, field_name, default_value)                      \
	boost::optional< type > field_name = ptree_to_optional_serializable_struct< type >( tree, #field_name, origin );	
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_VECTOR(type, field_name, default_value)                      \
	boost::optional< type > field_name = ptree_to_optional_serializable_vector< type >(tree, #field_name);
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE(_, type, field_name, default_value, serialization_type)      \
	ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_, serialization_type)(type, field_name, default_value)

#define SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL(_, type, field_name, ...)                                         \
	field_name ? field_name.get() : default_instance. field_name

// *** value --> ptree ***
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_PRIMITIVE(type, field_name)                                         \
	tree.add( #field_name , field_name );
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_PATH(type, field_name)                                              \
	tree.add( #field_name , postprocess_path( field_name, origin ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_ENUM(type, field_name)                                              \
	tree.add( #field_name , enumerator_to_string( field_name ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_STRUCT(type, field_name)                                            \
	tree.add_child( #field_name , field_name .ToPtree(origin));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_VECTOR(type, field_name)                                            \
	tree.add_child( #field_name , serializable_vector_to_ptree ( field_name ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE(_, type, field_name, default_value, serialization_type)             \
	ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_, serialization_type)(type, field_name)

// *** compare fields ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON(_, type, field_name, ...)                                            \
	instance1. field_name == instance2. field_name
// endregion

// region ==== SERIALIZABLE STRUCT TOP-LEVEL MACROS =======================

// *** declaration-only ***
#define SERIALIZABLE_STRUCT_DECL_IMPL( struct_name, ...)                                                               \
	SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_2( struct_name, field_count, ...)                                                \
	SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, field_count,                                                          \
							 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_3( struct_name, field_count, loop, ...)                                          \
	struct struct_name {                                                                                               \
		loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)                      \
		const std::string origin = "";                                                                                 \
		struct_name () {};                                                                                             \
		struct_name(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),          \
			std::string origin = "");                                                                                  \
		explicit struct_name (const boost::program_options::variables_map& vm, std::string origin = "");               \
		static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, std::string origin = "");           \
		boost::property_tree::ptree ToPTree(std::string origin = "") const;                                            \
		friend bool operator==(const struct_name & instance1, const struct_name & instance2);                          \
		friend std::ostream& operator<<(std::ostream& out, const struct_name& instance);                               \
	}
// *** definition-only ***
#define SERIALIZABLE_STRUCT_DEFN_IMPL( struct_name, ...)                                                               \
	SERIALIZABLE_STRUCT_DEFN_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DEFN_IMPL_2( struct_name, field_count, ...)                                                \
	SERIALIZABLE_STRUCT_DEFN_IMPL_3(struct_name, field_count,                                                          \
							 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_STRUCT_DEFN_IMPL_3( struct_name, field_count, loop, ...)                                          \
	struct_name::struct_name(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__), \
		std::string origin):                                                                                           \
			loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),               \
			origin(origin)                                                                                             \
			{}                                                                                                         \
	struct_name::struct_name(const boost::program_options::variables_map& vm, std::string origin) :                    \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),      \
			origin(std::move(origin))                                                                                  \
		{}                                                                                                             \
	struct_name struct_name::BuildFromPTree(const boost::property_tree::ptree& tree, std::string origin){              \
		struct_name default_instance;                                                                                  \
		loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)        \
		return {                                                                                                       \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),          \
			origin                                                                                                     \
		};																									           \
	}																											       \
	boost::property_tree::ptree struct_name::ToPTree(std::string origin) const {                                       \
		boost::property_tree::ptree tree;                                                                              \
		loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)               \
		return tree;                                                                                                   \
	}                                                                                                                  \
	bool operator==(const struct_name & instance1, const struct_name & instance2){                                     \
		return                                                                                                         \
		loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _, ITM_SERIALIZATION_IMPL_AND, __VA_ARGS__);                   \
	}                                                                                                                  \
	std::ostream& operator<<(std::ostream& out, const struct_name& instance) {                                         \
		boost::property_tree::ptree tree(instance.ToPTree());                                                          \
		boost::property_tree::write_json_no_quotes(out, tree, true);                                                   \
		return out;                                                                                                    \
	}                                                                                                              


#define SERIALIZABLE_STRUCT_IMPL( struct_name, ...)                                                                    \
	SERIALIZABLE_STRUCT_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_2( struct_name, field_count, ...)                                                     \
	SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count,                                                               \
							 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_STRUCT_IMPL_3( struct_name, field_count, loop, ...)                                               \
	struct struct_name {                                                                                               \
		loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)                      \
		const std::string origin = "";                                                                                 \
		struct_name () = default;                                                                                      \
		struct_name(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__), const std::string& origin = ""):\
			loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),               \
			origin(origin)                                                                                             \
			{}                                                                                                         \
		explicit struct_name (const boost::program_options::variables_map& vm, const std::string& origin = "") :       \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),      \
			origin(origin)                                                                                             \
		{}                                                                                                             \
		static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin = ""){    \
			struct_name default_instance;                                                                              \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)    \
			return {                                                                                                   \
				loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__),      \
				origin                                                                                                 \
			};																									       \
		}																											   \
		boost::property_tree::ptree ToPTree(const std::string& origin = "") const {                                    \
			boost::property_tree::ptree tree;                                                                          \
			loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)           \
			return tree;                                                                                               \
		}                                                                                                              \
		friend bool operator==(const struct_name & instance1, const struct_name & instance2){                          \
			return                                                                                                     \
			loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _, ITM_SERIALIZATION_IMPL_AND, __VA_ARGS__);               \
		}                                                                                                              \
		friend std::ostream& operator<<(std::ostream& out, const struct_name& instance) {                              \
			boost::property_tree::ptree tree(instance.ToPTree());                                                      \
			boost::property_tree::write_json_no_quotes(out, tree, true);                                               \
			return out;                                                                                                \
		}                                                                                                              \
	}

// endregion


// region ================== SERIALIZABLE ENUM PER-ENUMERATOR MACROS =============

// this top one is per-token, not per-enumerator
#define SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS(qualified_enumerator, token) { token , qualified_enumerator }

#define SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(_, enumerator, ...) enumerator
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS(enum_name, enumerator, ... )                              		       \
	SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, token_count, ... )                             \
	SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator,												   \
	 										 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP2_, token_count), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator, loop, ... )                                    \
	loop(SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS, enum_name::enumerator, ITM_SERIALIZATION_IMPL_COMMA,__VA_ARGS__)

//#define SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE(A, B )                                    \
//	case A : token = #B; break;
#define SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE(enum_name, enumerator, first_token, ... )                                    \
	case enum_name::enumerator : token = first_token; break;
// endregion
// region ================== SERIALIZABLE ENUM TOP-LEVEL MACROS ==================

//** declaration
#define SERIALIZABLE_ENUM_DECL_IMPL( enum_name, ...)                                                                   \
	SERIALIZABLE_ENUM_DECL_IMPL_2(enum_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_ENUM_DECL_IMPL_2( enum_name, field_count, ...)                                                    \
	SERIALIZABLE_ENUM_DECL_IMPL_3(enum_name, field_count,                                                              \
							 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_ENUM_DECL_IMPL_3( enum_name, field_count, loop, ...)                                              \
	enum enum_name {                                                                                                   \
		loop(SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)                    \
	};                                                                                                                 
	

//** definition
#define SERIALIZABLE_ENUM_DEFN_IMPL( enum_name, ...)                                                                   \
	SERIALIZABLE_ENUM_DEFN_IMPL_2(enum_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_ENUM_DEFN_IMPL_2( enum_name, field_count, ...)                                                    \
	SERIALIZABLE_ENUM_DEFN_IMPL_3(enum_name, field_count,                                                              \
							 ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_ENUM_DEFN_IMPL_3( enum_name, field_count, loop, ...)                                              \
	template<>                                                                                                         \
	enum_name string_to_enumerator< enum_name >(const std::string& string) {                                           \
		static std::unordered_map<std::string, enum_name > enumerator_by_string = {                                    \
				loop(SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS, enum_name, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)\
		};                                                                                                             \
		if (enumerator_by_string.find(string) == enumerator_by_string.end()) {                                         \
			DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized string token for enum " #enum_name);                         \
		}                                                                                                              \
		return enumerator_by_string[string];                                                                           \
		                                                                                                               \
	}                                                                                                                  \
	template<>                                                                                                         \
	std::string enumerator_to_string< enum_name >( const enum_name & value)	{                                          \
		std::string token = "";                                                                                        \
		switch(value) {                                                                                                \
			loop(SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE, enum_name, ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__)       \
	    }                                                                                                              \
	    return token;                                                                                                  \
    }

#define SERIALIZABLE_ENUM_IMPL( enum_name, ...)                                                                        \
	SERIALIZABLE_ENUM_DECL_IMPL(enum_name, __VA_ARGS__)                                                                \
	SERIALIZABLE_ENUM_DEFN_IMPL(enum_name, __VA_ARGS__)
// endregion
