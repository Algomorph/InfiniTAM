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
#include <regex>
#include <type_traits>

//boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>
#include <boost/preprocessor/stringize.hpp>

// local
#include "SequenceLevel1Macros.h"
#include "PreprocessorNargs.h"
#include "MetacodingAuxiliaryUtilities.h"
#include "../FileIO/JSON_Utilities.h"
#include "../Logging/PrettyPrinters.h"
#include "../../../ORUtils/PlatformIndependence.h"


#ifndef ITM_METACODING_STRING_TO_ENUMERATOR_DECLARATION
#define ITM_METACODING_STRING_TO_ENUMERATOR_DECLARATION
template<typename TEnum>
TEnum string_to_enumerator(const std::string& string);
#endif

#ifndef ITM_METACODING_ENUMERATOR_TO_STRING_DECLARATION
#define ITM_METACODING_ENUMERATOR_TO_STRING_DECLARATION
template<typename TEnum>
std::string enumerator_to_string(const TEnum& enum_value);
#endif

// region ==== OPTIONS_DESCRIPTION HELPER FUNCTIONS ============================


std::string find_snake_case_lowercase_acronym(const std::string& snake_case_identifier);
void generate_cli_argument_identifiers_snake_case(const boost::program_options::options_description& options_description,
                                                  const std::string& parent_long_identifier, const std::string& parent_short_identifier,
                                                  const std::string& field_name, std::string& long_identifier, std::string& short_identifier);

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

template<typename TSerializableStruct>
static TSerializableStruct
ExtractSerializableStructFromPtreeIfPresent(const pt::ptree& tree, pt::ptree::key_type const& key, const std::string& origin) {
	auto subtree = tree.get_child_optional(key);
	if (subtree) {
		return TSerializableStruct::BuildFromPTree(subtree.get(), origin);
	} else {
		return TSerializableStruct();
	}
}

// endregion
// region ================== SERIALIZABLE PATH FUNCTION DECLARATIONS ===================================================

std::string preprocess_path(const std::string& path, const std::string& origin);

std::string postprocess_path(const std::string& path, const std::string& origin);

boost::optional<std::string>
ptree_to_optional_path(const boost::property_tree::ptree& tree, const pt::ptree::key_type& key,
                       const std::string& origin);

// endregion
// region ================== SERIALIZABLE DYNAMIC_VECTOR FUNCTION DEFINITIONS ==================================================

// ==== path resolution ====

std::string compile_sub_struct_parse_path(const std::string& current_parse_path, const std::string& sub_struct_instance_name);

// ==== dynamic vector (std:: vector) of primitives or of enum values ====
template<typename TElementType, bool TElementsAreEnum>
struct StdVectorConverter;

template<typename TElementType>
struct StdVectorConverter<TElementType, true>{
    inline 
    static std::vector<TElementType> from_variables_map(const boost::program_options::variables_map& vm, const std::string& argument){
        std::vector<std::string> string_std_vector = vm[argument].as<std::vector<std::string>>();
        std::vector<TElementType> enum_std_vector;
        for(auto& string_enum_representation : string_std_vector){
            enum_std_vector.push_back(string_to_enumerator<TElementType>(string_enum_representation));
        }
        return enum_std_vector;
    }

    inline
    static void add_to_options_description(boost::program_options::options_description& od, const char* name, 
        std::vector<TElementType>& default_value, const char* description){
        std::vector<std::string> string_std_vector;
        for(auto& value : default_value){
            string_std_vector.push_back(enumerator_to_string<TElementType>(value));
        }
        od.add_options()(
            name, 
            boost::program_options::value<std::vector<std::string>>()->
                multitoken()->default_value(string_std_vector), description
        );
    }

    inline
    static std::vector<TElementType> from_ptree(pt::ptree const& pt, pt::ptree::key_type const& key){
        std::vector<TElementType> std_vector;
        for (auto& item : pt.get_child(key)) {
            std_vector.push_back(string_to_enumerator<TElementType>(item.second.get_value<std::string>()));
        }
        return std_vector;
    }

    inline 
    static boost::property_tree::ptree to_ptree(std::vector<TElementType>& value){
        boost::property_tree::ptree tree;
        for (int i_element = 0; i_element < value.size(); i_element++) {
            boost::property_tree::ptree child;
            child.put("", enumerator_to_string<TElementType>(value[i_element]));
            tree.push_back(std::make_pair("", child));
        }
        return tree;
    }
};

template<typename TElementType>
struct StdVectorConverter<TElementType, false>{
    inline 
    static std::vector<TElementType> from_variables_map(const boost::program_options::variables_map& vm, const std::string& argument){
        std::vector<TElementType> std_vector = vm[argument].as<std::vector<TElementType>>();
        return std_vector;
    }

    inline
    static void add_to_options_description(boost::program_options::options_description& od, const char* name, 
        std::vector<TElementType>& default_value, const char* description){
        od.add_options()(
            name, 
            boost::program_options::value<std::vector<TElementType>>()->
                multitoken()->default_value(default_value), description
        );
    }  

    inline
    static std::vector<TElementType> from_ptree(pt::ptree const& pt, pt::ptree::key_type const& key){
        std::vector<TElementType> std_vector;
        for (auto& item : pt.get_child(key)) {
            std_vector.push_back(item.second.get_value<TElementType>());
        }
        return std_vector;
    }

    inline 
    static boost::property_tree::ptree to_ptree(std::vector<TElementType>& value){
        boost::property_tree::ptree tree;
        for (int i_element = 0; i_element < value.size(); i_element++) {
            boost::property_tree::ptree child;
            child.put("", value[i_element]);
            tree.push_back(std::make_pair("", child));
        }
        return tree;
    }
};

template<typename TStdVector>
TStdVector variables_map_to_std_vector(const boost::program_options::variables_map& vm, const std::string& argument) {
    return StdVectorConverter<typename TStdVector::value_type, std::is_enum< typename TStdVector::value_type >::value>::
        from_variables_map(vm, argument);
}

template<typename TStdVector>
void add_std_vector_to_options_description(boost::program_options::options_description& od, const char* name, 
    TStdVector& default_value, const char* description){
    return StdVectorConverter<typename TStdVector::value_type, std::is_enum< typename TStdVector::value_type >::value>::
        add_to_options_description(od, name, default_value, description);
}

template<typename TStdVector>
boost::optional<TStdVector> ptree_to_optional_std_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
    if (pt.count(key) == 0) {
        return boost::optional<TStdVector>{};
    }
    return StdVectorConverter<typename TStdVector::value_type, std::is_enum< typename TStdVector::value_type >::value>::
            from_ptree(pt, key);
}

template<typename TStdVector>
boost::property_tree::ptree std_vector_to_ptree(TStdVector& vector) {
    return StdVectorConverter<typename TStdVector::value_type, std::is_enum< typename TStdVector::value_type >::value>::
            to_ptree(vector);
}

// endregion 
// region ================== SERIALIZABLE STATIC_VECTOR FUNCTION DEFINITIONS ==================================================
// ==== static vector types (Vector2, Vector3, etc... ) of primitives ==============================================

template<typename TStaticVector>
std::vector<typename TStaticVector::value_type> static_vector_to_std_vector(TStaticVector vector) {
	std::vector<typename TStaticVector::value_type> std_vector;
	for (int i_element = 0; i_element < TStaticVector::size(); i_element++) {
		std_vector.push_back(vector.values[i_element]);
	}
	return std_vector;
}

template<typename TStaticVector>
TStaticVector std_vector_to_static_vector(const std::vector<typename TStaticVector::value_type>& std_vector) {
	TStaticVector vector;
	if (std_vector.size() != TStaticVector::size()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Wrong number of elements in parsed vector.");
	}
	memcpy(vector.values, std_vector.data(), sizeof(typename TStaticVector::value_type) * TStaticVector::size());
	return vector;
}



template<typename TStaticVector>
TStaticVector variables_map_to_static_vector(const boost::program_options::variables_map& vm, const std::string& argument) {
	std::vector<typename TStaticVector::value_type> std_vector = vm[argument].as<std::vector<typename TStaticVector::value_type>>();
	return std_vector_to_static_vector<TStaticVector>(std_vector);
}

template<typename TStaticVector>
boost::optional<TStaticVector> ptree_to_optional_static_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	if (pt.count(key) == 0) {
		return boost::optional<TStaticVector>{};
	}
	std::vector<typename TStaticVector::value_type> std_vector;
	for (auto& item : pt.get_child(key)) {
		std_vector.push_back(item.second.get_value<typename TStaticVector::value_type>());
	}
	return std_vector_to_static_vector<TStaticVector>(std_vector);
}

template<typename TStaticVector>
boost::property_tree::ptree static_vector_to_ptree(TStaticVector vector) {
	boost::property_tree::ptree tree;
	for (int i_element = 0; i_element < TStaticVector::size(); i_element++) {
		boost::property_tree::ptree child;
		child.put("", vector.values[i_element]);
		tree.push_back(std::make_pair("", child));
	}
	return tree;
}

// endregion
// region ===== SERIALIZABLE STRUCT PER-FIELD MACROS ===================================================================

// *** used to declare fields & defaults ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_DECL(_, type, field_name, default_value, serialization_type, ...) \
    type field_name = default_value;

// *** used for a generic constructor that contains all fields ***
#define SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD(_, type, field_name, ...) type field_name

#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_PRIMITIVE(type, field_name) field_name ( field_name )
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_PATH(type, field_name) field_name ( std::move(field_name) )
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_ENUM(type, field_name) field_name ( field_name )
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_STRUCT(type, field_name) field_name ( std::move(field_name) )
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_STATIC_VECTOR(type, field_name) field_name ( field_name )
#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_DYNAMIC_VECTOR(type, field_name) field_name ( field_name )

#define SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG(_, type, field_name, default_value, serialization_type, ...) \
        ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG_, serialization_type)(type, field_name)

// *** variables_map --> value ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_PRIMITIVE(type, field_name, default_value) \
    field_name(vm[compile_sub_struct_parse_path(parent_long_identifier, #field_name)].as<type>())
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_PATH(type, field_name, default_value) \
    field_name(preprocess_path(vm[ compile_sub_struct_parse_path(parent_long_identifier, #field_name)  ].as<std::string>(), origin))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_ENUM(type, field_name, default_value) \
    field_name(string_to_enumerator< type >(vm[ compile_sub_struct_parse_path(parent_long_identifier, #field_name)  ].as<std::string>()))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_STRUCT(type, field_name, default_value) \
    field_name(vm, compile_sub_struct_parse_path(parent_long_identifier, #field_name))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_STATIC_VECTOR(type, field_name, default_value) \
    field_name(variables_map_to_static_vector <type> (vm, compile_sub_struct_parse_path(parent_long_identifier, #field_name) ))
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_DYNAMIC_VECTOR(type, field_name, default_value) \
    field_name(variables_map_to_std_vector <type> (vm, compile_sub_struct_parse_path(parent_long_identifier, #field_name) ))

#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT(struct_name, type, field_name, default_value, serialization_type, ...) \
    ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_, serialization_type)(type, field_name, default_value)

#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_PRIMITIVE(type, field_name) \
    if (!vm[long_identifier].defaulted())  this->field_name = vm[long_identifier].as<type>()
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_PATH(type, field_name) \
    if (!vm[long_identifier].defaulted())  this->field_name = preprocess_path(vm[long_identifier].as<std::string>(), origin)
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_ENUM(type, field_name) \
    if (!vm[long_identifier].defaulted())  this->field_name = string_to_enumerator< type >(vm[long_identifier].as<std::string>())
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_STRUCT(type, field_name) \
    this->field_name.UpdateFromVariablesMap(vm, long_identifier )
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_STATIC_VECTOR(type, field_name) \
    if (!vm[long_identifier].defaulted())  this->field_name = variables_map_to_static_vector <type> (vm, long_identifier)
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_DYNAMIC_VECTOR(type, field_name) \
    if (!vm[long_identifier].defaulted())  this->field_name = variables_map_to_std_vector <type> (vm, long_identifier)

#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE(struct_name, type, field_name, default_value, serialization_type, ...) \
	long_identifier = compile_sub_struct_parse_path(parent_long_identifier, #field_name); \
    ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE_, serialization_type)(type, field_name)

// *** field & default value --> options_description ***

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_PRIMITIVE(type, field_name, default_value_in, description)\
    od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), \
    boost::program_options::value< type >()->default_value( default_value_in ), description);

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_PATH(type, field_name, default_value_in, description)\
    od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), \
    boost::program_options::value< type >()->default_value( default_value_in ), description);

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_ENUM(type, field_name, default_value_in, description)\
    od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), \
    boost::program_options::value< std::string >()->default_value( enumerator_to_string(default_value_in) ), \
    (std::string(description) + enumerator_bracketed_list< type>()).c_str());

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_STRUCT(type, field_name, default_value, description)\
    type :: AddToOptionsDescription(od, field_long_identifier, field_short_identifier);

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_STATIC_VECTOR(type, field_name, default_value_in, description)\
    od.add_options()((field_long_identifier + "," + field_short_identifier).c_str(), \
    boost::program_options::value< std::vector< type::value_type> >()-> \
    multitoken()->default_value(static_vector_to_std_vector(default_value_in)), description);

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_DYNAMIC_VECTOR(type, field_name, default_value_in, description)\
    add_std_vector_to_options_description(od, (field_long_identifier + "," + field_short_identifier).c_str(), default_value_in, description);


#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION(_, type, field_name, default_value, serialization_type, description) \
    generate_cli_argument_identifiers_snake_case(od, long_identifier, short_identifier, #field_name, field_long_identifier, field_short_identifier); \
    ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_, serialization_type)(type, field_name, default_value, description)

// *** ptree --> value ***

#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PRIMITIVE(type, field_name, default_value) \
    boost::optional< type > field_name = tree.get_optional< type > ( #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PATH(type, field_name, default_value) \
    boost::optional< type > field_name = ptree_to_optional_path( tree, #field_name, origin );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_ENUM(type, field_name, default_value) \
    boost::optional< type > field_name = ptree_to_optional_enumerator< type >( tree, #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_STRUCT(type, field_name, default_value) \
    boost::optional< type > field_name = ptree_to_optional_serializable_struct< type >( tree, #field_name, origin );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_STATIC_VECTOR(type, field_name, default_value) \
    boost::optional< type > field_name = ptree_to_optional_static_vector< type >(tree, #field_name);
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_DYNAMIC_VECTOR(type, field_name, default_value) \
    boost::optional< type > field_name = ptree_to_optional_std_vector< type >(tree, #field_name);

#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE(_, type, field_name, default_value, serialization_type, ...) \
    ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_, serialization_type)(type, field_name, default_value)

#define SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL(_, type, field_name, ...) \
    field_name ? field_name.get() : default_instance. field_name

// *** value --> ptree ***

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_PRIMITIVE(type, field_name) \
    tree.add( #field_name , field_name );
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_PATH(type, field_name) \
    tree.add( #field_name , postprocess_path( field_name, _origin ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_ENUM(type, field_name) \
    tree.add( #field_name , enumerator_to_string( field_name ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_STRUCT(type, field_name) \
    tree.add_child( #field_name , field_name .ToPTree(origin));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_STATIC_VECTOR(type, field_name) \
    tree.add_child( #field_name , static_vector_to_ptree ( field_name ));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_DYNAMIC_VECTOR(type, field_name) \
    tree.add_child( #field_name , std_vector_to_ptree ( field_name ));

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE(_, type, field_name, default_value, serialization_type, ...) \
    ITM_METACODING_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_, serialization_type)(type, field_name)

// *** compare fields ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON(_, type, field_name, ...) \
    instance1. field_name == instance2. field_name


// endregion
// region ==== SERIALIZABLE STRUCT TOP-LEVEL MACROS =======================
// region ******************************************************** DECLARATION-ONLY ******************************************************************
#define SERIALIZABLE_STRUCT_DECL_IMPL(struct_name, ...) \
    SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, field_count, ...) \
    SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, loop, ...) \
    struct struct_name { \
        SERIALIZABLE_STRUCT_DECL_IMPL_BODY(struct_name, loop, __VA_ARGS__) \
    }

#define ORIGIN_AND_SOURCE_TREE() \
    std::string origin = ""; \
    boost::property_tree::ptree source_tree;

#define SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, ...) \
    ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, \
                                           ITM_METACODING_IMPL_NOTHING, __VA_ARGS__))

#define SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_INDEPENDENT_FUNCS(struct_name, loop, ...) \
    struct_name (); \
    struct_name(ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, \
                                                   ITM_METACODING_IMPL_COMMA, __VA_ARGS__)), \
        std::string origin = ""); \
    void SetFromPTree(const boost::property_tree::ptree& tree); \
    static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin = ""); \
    boost::property_tree::ptree ToPTree(const std::string& _origin = "") const; \
    friend bool operator==(const struct_name & instance1, const struct_name & instance2); \
    friend std::ostream& operator<<(std::ostream& out, const struct_name& instance);

#define SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_DEPENDENT_FUNCS(struct_name) \
	explicit struct_name (const boost::program_options::variables_map& vm, const std::string& parent_long_identifier = "", std::string origin = ""); \
	void UpdateFromVariablesMap(const boost::program_options::variables_map& vm, const std::string& parent_long_identifier = ""); \
	static void AddToOptionsDescription(boost::program_options::options_description& od, const std::string& long_identifier = "", const std::string& short_identifier = "");

#define SERIALIZABLE_STRUCT_DECL_IMPL_BODY(struct_name, loop, ...) \
        ORIGIN_AND_SOURCE_TREE() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_INDEPENDENT_FUNCS(struct_name, loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_PATH_DEPENDENT_FUNCS(struct_name)

// endregion
// region ********************************************************* DEFINITION-ONLY ******************************************************************
#define SERIALIZABLE_STRUCT_DEFN_IMPL(outer_class, struct_name, ...) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_2( outer_class, struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(qualifier) \
    ITM_METACODING_IMPL_IIF(ITM_METACODING_IMPL_ISEMPTY(qualifier)) \
    (qualifier, qualifier::)


#define SERIALIZABLE_STRUCT_DEFN_IMPL_2(outer_class, struct_name, field_count, ...) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_3(SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(outer_class), \
                             SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(struct_name), struct_name, instance.source_tree=tree, , ,\
                             ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, , \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DEFN_IMPL_3(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
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
                                        loop, __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_DEPENDENT(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                        friend_qualifier, static_qualifier, \
                                        ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                        loop, ...) \
	outer_class inner_qualifier struct_name(const boost::program_options::variables_map& vm, \
                                            const std::string& parent_long_identifier default_string_arg, \
                                            std::string origin default_string_arg) : \
            ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name, \
                                               ITM_METACODING_IMPL_COMMA, __VA_ARGS__)) \
            ORIGIN_SEPARATOR() origin_initializer \
        {} \
	void outer_class inner_qualifier UpdateFromVariablesMap(const boost::program_options::variables_map& vm, \
                                                            const std::string& parent_long_identifier default_string_arg){ \
        std::string long_identifier; \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_UPDATE, struct_name, \
                                               ITM_METACODING_IMPL_SEMICOLON, __VA_ARGS__)); \
    } \
    static_qualifier void outer_class inner_qualifier AddToOptionsDescription( \
            boost::program_options::options_description& od, \
            const std::string& long_identifier default_string_arg, \
            const std::string& short_identifier default_string_arg) { \
        std::string field_long_identifier, field_short_identifier; \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION, _, \
                                           ITM_METACODING_IMPL_NOTHING, __VA_ARGS__)) \
    }


#define SERIALIZABLE_STRUCT_DEFN_IMPL_PATH_INDEPENDENT(outer_class, inner_qualifier, struct_name, source_tree_initializer, \
                                        friend_qualifier, static_qualifier, \
                                        ORIGIN_SEPARATOR, origin_initializer, origin_varname, default_string_arg, \
                                        loop, ...) \
    outer_class inner_qualifier struct_name () = default; \
    outer_class inner_qualifier struct_name( \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, ITM_METACODING_IMPL_COMMA, \
                                           __VA_ARGS__)), \
        std::string origin default_string_arg): \
            ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _, \
                                               ITM_METACODING_IMPL_COMMA, __VA_ARGS__)) \
            ORIGIN_SEPARATOR() origin_initializer \
            {} \
    void outer_class inner_qualifier SetFromPTree(const boost::property_tree::ptree& tree){ \
        struct_name temporary_instance = BuildFromPTree(tree);\
        *this = temporary_instance;\
    } \
    static_qualifier outer_class struct_name outer_class inner_qualifier BuildFromPTree(const boost::property_tree::ptree& tree, \
                                                                    const std::string& origin default_string_arg){ \
        struct_name default_instance; \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _, \
                                           ITM_METACODING_IMPL_NOTHING, __VA_ARGS__)) \
        struct_name instance = { \
            ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _, \
                                               ITM_METACODING_IMPL_COMMA, __VA_ARGS__)) \
            ORIGIN_SEPARATOR() origin_varname \
        }; \
        source_tree_initializer; \
        return instance; \
    } \
    boost::property_tree::ptree outer_class inner_qualifier ToPTree(const std::string& _origin default_string_arg) const { \
        boost::property_tree::ptree tree; \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _, \
                                           ITM_METACODING_IMPL_NOTHING, __VA_ARGS__)) \
        return tree; \
    } \
    friend_qualifier bool operator==(const outer_class struct_name & instance1, const outer_class struct_name & instance2){ \
        return \
        ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _, ITM_METACODING_IMPL_AND, \
                                           __VA_ARGS__)); \
    } \
    friend_qualifier std::ostream& operator<<(std::ostream& out, const outer_class struct_name& instance) { \
        boost::property_tree::ptree tree(instance.ToPTree()); \
        boost::property_tree::write_json_no_quotes(out, tree, true); \
        return out; \
    }

// endregion
// *** FULL STRUCT GENERATION ***
#define SERIALIZABLE_STRUCT_IMPL(struct_name, ...) \
    SERIALIZABLE_STRUCT_IMPL_2(struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_2(struct_name, field_count, ...) \
    SERIALIZABLE_STRUCT_IMPL_3(struct_name, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_3(struct_name, loop, ...) \
    struct struct_name { \
        ORIGIN_AND_SOURCE_TREE() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DEFN_IMPL_3 ( , , struct_name, instance.source_tree=tree, friend, static, \
        ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, ="", \
        loop, __VA_ARGS__) \
    };

// endregion



