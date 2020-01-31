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

//boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>
#include <boost/preprocessor/stringize.hpp>

// local
#include "SerializationSequenceMacros.h"
#include "PreprocessorNargs.h"
#include "../JsonUtilities.h"
#include "../../../ORUtils/PlatformIndependence.h"

// region ==== LOW-LEVEL MACROS
#define ITM_SERIALIZATION_IMPL_SEMICOLON() ;
#define ITM_SERIALIZATION_IMPL_COMMA() ,
#define ITM_SERIALIZATION_IMPL_AND() &&
#define ITM_SERIALIZATION_IMPL_NOTHING()

#define ITM_SERIALIZATION_IMPL_PRIMITIVE_CAT(a, ...) a##__VA_ARGS__
#define ITM_SERIALIZATION_IMPL_CAT(a, ...) ITM_SERIALIZATION_IMPL_PRIMITIVE_CAT(a, __VA_ARGS__)

#define ITM_SERIALIZATION_IMPL_ARG16(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, ...) _15
#define ITM_SERIALIZATION_IMPL_HAS_COMMA(...) ITM_SERIALIZATION_IMPL_EXPAND(ITM_SERIALIZATION_IMPL_ARG16(__VA_ARGS__, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0))
#define ITM_SERIALIZATION_IMPL_TRIGGER_PARENTHESIS_(...) ,

#define ITM_SERIALIZATION_IMPL_ISEMPTY(...)                             \
ITM_SERIALIZATION_IMPL_PRIMITIVE_ISEMPTY(                               \
          /* test if there is just one argument, eventually an empty    \
             one */                                                     \
          ITM_SERIALIZATION_IMPL_HAS_COMMA(__VA_ARGS__),                \
          /* test if _TRIGGER_PARENTHESIS_ together with the argument   \
             adds a comma */                                            \
          ITM_SERIALIZATION_IMPL_HAS_COMMA(_TRIGGER_PARENTHESIS_ __VA_ARGS__),\
          /* test if the argument together with a parenthesis           \
             adds a comma */                                            \
          ITM_SERIALIZATION_IMPL_HAS_COMMA(__VA_ARGS__ (/*empty*/)),    \
          /* test if placing it between _TRIGGER_PARENTHESIS_ and the   \
             parenthesis adds a comma */                                \
          ITM_SERIALIZATION_IMPL_HAS_COMMA(ITM_SERIALIZATION_IMPL_TRIGGER_PARENTHESIS_ __VA_ARGS__ (/*empty*/))      \
          )

#define ITM_SERIALIZATION_IMPL_PASTE5(_0, _1, _2, _3, _4) _0 ## _1 ## _2 ## _3 ## _4
#define ITM_SERIALIZATION_IMPL_PRIMITIVE_ISEMPTY(_0, _1, _2, _3) ITM_SERIALIZATION_IMPL_HAS_COMMA(ITM_SERIALIZATION_IMPL_PASTE5(ITM_SERIALIZATION_IMPL_IS_EMPTY_CASE_, _0, _1, _2, _3))
#define ITM_SERIALIZATION_IMPL_IS_EMPTY_CASE_0001 ,

// endregion
// region ==== OPTIONS_DESCRIPTION HELPER FUNCTIONS ============================
namespace std {
//definition required by boost::program_options to output default values of vector types
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
	for (auto item : vec) {
		os << item << " ";
	}
	return os;
}
}//namespace std

static std::string generate_cli_argument_identifiers_snake_case(std::string variable_name,
                                                                const boost::program_options::options_description& opt) {
	std::string delimiter = "_";

	size_t pos = 0;
	std::string token;
	std::string short_argument_identifier;
	while ((pos = variable_name.find(delimiter)) != std::string::npos) {
		token = variable_name.substr(0, pos);
		short_argument_identifier += token[0];
		variable_name.erase(0, pos + delimiter.length());
	}
    short_argument_identifier += variable_name[0];
	const int maximum_similar_shorthands_allowed = 5;
	std::string base_short_argument_identifier = short_argument_identifier;
	for (int try_count = 0; opt.find_nothrow(short_argument_identifier, false, false, false) != nullptr &&
	                        try_count < maximum_similar_shorthands_allowed; try_count++) {
		const std::regex expression_ending_with_digit("\\w+(\\d)");
		std::smatch match;
		if (std::regex_match(short_argument_identifier, match, expression_ending_with_digit)) {
			int index = std::stoi(match[1].str());
			short_argument_identifier = base_short_argument_identifier + std::to_string(index + 1);
		} else {
			short_argument_identifier += "1";
		}
	}
	if (opt.find_nothrow(short_argument_identifier, false, false, false) != nullptr) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"There are too many argument identifiers with similar shorthands, please come up with an alternative argument name.");
	}
	return variable_name + "," + short_argument_identifier;
}

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
TEnum string_to_enumerator(const std::string& string);

template<typename TEnum>
std::string enumerator_to_string(const TEnum& enum_value);

template<typename TEnum>
std::string enumerator_bracketed_list();

template<typename TEnum>
TEnum variables_map_to_enumerator(const boost::program_options::variables_map& vm, const std::string& argument) {
	return string_to_enumerator<TEnum>(vm[argument].as<std::string>());
}

template<typename TEnum>
boost::optional<TEnum> ptree_to_optional_enumerator(const pt::ptree& ptree, const pt::ptree::key_type& key) {
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

boost::optional<std::string>
ptree_to_optional_path(const boost::property_tree::ptree& tree, const pt::ptree::key_type& key,
                       const std::string& origin);

// endregion
// region ================== SERIALIZABLE VECTOR FUNCTION DEFINITIONS ===========
template<typename TVector>
std::vector<typename TVector::value_type> serializable_vector_to_std_vector(TVector vector) {
	std::vector<typename TVector::value_type> std_vector;
	for (int i_element = 0; i_element < TVector::size(); i_element++) {
		std_vector.push_back(vector.values[i_element]);
	}
	return std_vector;
}

template<typename TVector>
TVector std_vector_to_serializable_vector(const std::vector<typename TVector::value_type>& std_vector) {
	TVector vector;
	if (std_vector.size() != TVector::size()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Wrong number of elements in parsed vector.");
	}
	memcpy(vector.values, std_vector.data(), sizeof(typename TVector::value_type) * TVector::size());
	return vector;
}

template<typename TVector>
TVector variables_map_to_vector(const boost::program_options::variables_map& vm, const std::string& argument) {
	std::vector<typename TVector::value_type> std_vector = vm[argument].as<std::vector<typename TVector::value_type>>();
	return std_vector_to_serializable_vector<TVector>(std_vector);
}

template<typename TVector>
boost::optional<TVector> ptree_to_optional_serializable_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	TVector vector;
	if (pt.count(key) == 0) {
		return boost::optional<TVector>{};
	}
	std::vector<typename TVector::value_type> std_vector;
	for (auto& item : pt.get_child(key)) {
		std_vector.push_back(item.second.get_value<typename TVector::value_type>());
	}
	return std_vector_to_serializable_vector<TVector>(std_vector);;
}

template<typename TVector>
boost::property_tree::ptree serializable_vector_to_ptree(TVector vector) {
	boost::property_tree::ptree tree;
	for (int i_element = 0; i_element < TVector::size(); i_element++) {
		boost::property_tree::ptree child;
		child.put("", vector.values[i_element]);
		tree.push_back(std::make_pair("", child));
	}
	return tree;
}

// endregion

// region ================== SERIALIZABLE ENUM PER-ENUMERATOR MACROS =============

// this top one is per-token, not per-enumerator
#define SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS(qualified_enumerator, token) { token , qualified_enumerator }

#define SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATOR(_, enumerator, ...) enumerator
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS(enum_name, enumerator, ...)                                           \
    SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, token_count, ...)                             \
    SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator,                                                   \
                                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP2_, token_count), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator, loop, ...)                                    \
    ITM_SERIALIZATION_IMPL_EXPAND2(loop(SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS, enum_name::enumerator, ITM_SERIALIZATION_IMPL_COMMA,__VA_ARGS__))

//#define SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE(A, B )                                    \
//	case A : token = #B; break;
#define SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE(enum_name, enumerator, first_token, ...)                            \
    case enum_name::enumerator : token = first_token; break;
// endregion
// region ================== SERIALIZABLE ENUM TOP-LEVEL MACROS ==================

#define SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(...)                                                                   \
    ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__))                 \
                (SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATOR, _, ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)

//** declaration
#define SERIALIZABLE_ENUM_DECL_IMPL(enum_name, ...)                                                                    \
    enum enum_name {                                                                                                   \
        SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(__VA_ARGS__)                                                           \
    };

//** definition
#define SERIALIZABLE_ENUM_DEFN_IMPL(INLINE, enum_name, ...)                                                            \
    SERIALIZABLE_ENUM_DEFN_IMPL_2(INLINE, enum_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_ENUM_DEFN_IMPL_2(INLINE, enum_name, field_count, ...)                                             \
    SERIALIZABLE_ENUM_DEFN_IMPL_3(INLINE, enum_name, field_count,                                                      \
                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_ENUM_DEFN_IMPL_3(INLINE, enum_name, field_count, loop, ...)                                       \
    template<>                                                                                                         \
    INLINE enum_name string_to_enumerator< enum_name >(const std::string& string) {                                    \
        static std::unordered_map<std::string, enum_name > enumerator_by_string = {                                    \
                ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS, enum_name,                  \
                	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__))                         \
        };                                                                                                             \
        if (enumerator_by_string.find(string) == enumerator_by_string.end()) {                                         \
            DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized string token for enum " #enum_name);                         \
        }                                                                                                              \
        return enumerator_by_string[string];                                                                           \
                                                                                                                       \
    }                                                                                                                  \
    template<>                                                                                                         \
    INLINE std::string enumerator_to_string< enum_name >( const enum_name & value)    {                                \
        std::string token;                                                                                             \
        switch(value) {                                                                                                \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE, enum_name,                   \
            	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
        }                                                                                                              \
        return token;                                                                                                  \
    }                                                                                                                  \
    template<>                                                                                                         \
    INLINE std::string enumerator_bracketed_list< enum_name >(){                                                       \
        return "[" BOOST_PP_STRINGIZE(BOOST_PP_CAT2(SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(__VA_ARGS__))) "]";        \
    }


#define SERIALIZABLE_ENUM_IMPL(enum_name, ...)                                                                         \
    SERIALIZABLE_ENUM_DECL_IMPL(enum_name, __VA_ARGS__)                                                                \
    SERIALIZABLE_ENUM_DEFN_IMPL(enum_name, __VA_ARGS__)
// endregion

// region ===== SERIALIZABLE STRUCT PER-FIELD MACROS ===========

// *** used to declare fields & defaults ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_DECL(_, type, field_name, default_value, serialization_type, ...)               \
    type field_name = default_value;

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
#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_VECTOR(struct_name, type, field_name, default_value)                    \
    field_name(vm[ #field_name ].empty() ? struct_name (). field_name : variables_map_to_vector <type> (vm, #field_name))

#define SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT(struct_name, type, field_name, default_value, serialization_type, ...) \
    ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_, serialization_type)(struct_name, type, field_name, default_value)

// *** value --> options_description ***
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_PRIMITIVE(type, field_name, default_value_in, description)\
    od.add_options()(generate_cli_argument_identifiers_snake_case(#field_name, od).c_str(),                            \
    boost::program_options::value< type >()->default_value( default_value_in ), #description);
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_PATH(type, field_name, default_value_in, description)\
    od.add_options()(generate_cli_argument_identifiers_snake_case(#field_name, od).c_str(),                            \
    boost::program_options::value< type >()->default_value( default_value_in ), #description);
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_ENUM(type, field_name, default_value_in, description)\
    od.add_options()(generate_cli_argument_identifiers_snake_case(#field_name, od).c_str(),                            \
    boost::program_options::value< std::string >()->default_value( enumerator_to_string(default_value_in) ),           \
    (std::string(#description) + enumerator_bracketed_list< type>()).c_str());
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_STRUCT(type, field_name, default_value, description)\
    type :: AddToOptionsDescription(od);
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_VECTOR(type, field_name, default_value_in, description)\
    od.add_options()(generate_cli_argument_identifiers_snake_case(#field_name, od).c_str(),                            \
    boost::program_options::value< std::vector< type::value_type> >()->                                                \
    multitoken()->default_value(serializable_vector_to_std_vector(default_value_in)), #description);

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION(_, type, field_name, default_value, serialization_type, description) \
    ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_, serialization_type)(type, field_name, default_value, description)

// *** ptree --> value ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PRIMITIVE(type, field_name, default_value)                   \
    boost::optional< type > field_name = tree.get_optional< type > ( #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_PATH(type, field_name, default_value)                        \
    boost::optional< type > field_name = ptree_to_optional_path( tree, #field_name, origin );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_ENUM(type, field_name, default_value)                        \
    boost::optional< type > field_name = ptree_to_optional_enumerator< type >( tree, #field_name );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_STRUCT(type, field_name, default_value)                      \
    boost::optional< type > field_name = ptree_to_optional_serializable_struct< type >( tree, #field_name, origin );
#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_VECTOR(type, field_name, default_value)                      \
    boost::optional< type > field_name = ptree_to_optional_serializable_vector< type >(tree, #field_name);

#define SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE(_, type, field_name, default_value, serialization_type, ...) \
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
    tree.add_child( #field_name , field_name .ToPTree(origin));
#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_VECTOR(type, field_name)                                            \
    tree.add_child( #field_name , serializable_vector_to_ptree ( field_name ));

#define SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE(_, type, field_name, default_value, serialization_type, ...)        \
    ITM_SERIALIZATION_IMPL_CAT(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_, serialization_type)(type, field_name)

// *** compare fields ***
#define SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON(_, type, field_name, ...)                                            \
    instance1. field_name == instance2. field_name


// endregion

// region ==== SERIALIZABLE STRUCT TOP-LEVEL MACROS =======================

// *** declaration-only ***
#define SERIALIZABLE_STRUCT_DECL_IMPL(struct_name, ...)                                                                \
    SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, field_count, ...)                                                 \
    SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, field_count,                                                          \
                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, field_count, loop, ...)                                           \
    struct struct_name {                                                                                               \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _,                                     \
        	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                               \
        std::string origin = "";                                                                                       \
        struct_name () {};                                                                                             \
        struct_name(ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _,                        \
        	                                           ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                    \
            std::string origin = "");                                                                                  \
        explicit struct_name (const boost::program_options::variables_map& vm, std::string origin = "");               \
        static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, std::string origin = "");           \
        boost::property_tree::ptree ToPTree(std::string origin = "") const;                                            \
        friend bool operator==(const struct_name & instance1, const struct_name & instance2);                          \
        friend std::ostream& operator<<(std::ostream& out, const struct_name& instance);                               \
        static void AddToOptionsDescription(boost::program_options::options_description& od);                          \
    }
// *** definition-only ***
#define SERIALIZABLE_STRUCT_DEFN_IMPL(outer_class, struct_name, ...)                                                   \
    SERIALIZABLE_STRUCT_DEFN_IMPL_2( outer_class, struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_DEFN_HANDLE_OUTER_CLASS(outer_class) \
    ITM_SERIALIZATION_IMPL_IIF(ITM_SERIALIZATION_IMPL_ISEMPTY(outer_class)) \
    (outer_class, outer_class::)


#define SERIALIZABLE_STRUCT_DEFN_IMPL_2(outer_class, struct_name, field_count, ...)                                    \
    SERIALIZABLE_STRUCT_DEFN_IMPL_3(SERIALIZABLE_STRUCT_DEFN_HANDLE_OUTER_CLASS(outer_class) , struct_name, field_count,\
                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_STRUCT_DEFN_IMPL_3(outer_class, struct_name, field_count, loop, ...)                              \
    outer_class struct_name::struct_name(                                                                              \
    	ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _, ITM_SERIALIZATION_IMPL_COMMA,      \
    		                               __VA_ARGS__)),                                                              \
        std::string origin):                                                                                           \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _,                             \
            	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                            \
            origin(origin)                                                                                             \
            {}                                                                                                         \
    outer_class struct_name::struct_name(const boost::program_options::variables_map& vm, std::string origin) :        \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name,                    \
            	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                            \
            origin(std::move(origin))                                                                                  \
        {}                                                                                                             \
    outer_class struct_name outer_class struct_name::BuildFromPTree(const boost::property_tree::ptree& tree,           \
    	                                                            std::string origin){                               \
        struct_name default_instance;                                                                                  \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _,                       \
        	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                               \
        return {                                                                                                       \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _,                        \
            	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                            \
            origin                                                                                                     \
        };                                                                                                             \
    }                                                                                                                  \
    boost::property_tree::ptree outer_class struct_name::ToPTree(std::string origin) const {                           \
        boost::property_tree::ptree tree;                                                                              \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _,                              \
        	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                               \
        return tree;                                                                                                   \
    }                                                                                                                  \
    bool operator==(const outer_class struct_name & instance1, const outer_class struct_name & instance2){             \
        return                                                                                                         \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _, ITM_SERIALIZATION_IMPL_AND,   \
        	                               __VA_ARGS__));                                                              \
    }                                                                                                                  \
    std::ostream& operator<<(std::ostream& out, const outer_class struct_name& instance) {                             \
        boost::property_tree::ptree tree(instance.ToPTree());                                                          \
        boost::property_tree::write_json_no_quotes(out, tree, true);                                                   \
        return out;                                                                                                    \
    }                                                                                                                  \
    void outer_class struct_name::AddToOptionsDescription(boost::program_options::options_description& od) {           \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION, _,               \
        	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                               \
    }


#define SERIALIZABLE_STRUCT_IMPL(struct_name, ...)                                                                     \
    SERIALIZABLE_STRUCT_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_STRUCT_IMPL_2(struct_name, field_count, ...)                                                      \
    SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count,                                                               \
                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)

//TODO: to avoid DRY violations, make individual function definitions into defines used both here and in SERIALIZABLE_STRUCT_DEFN_IMPL_3
#define SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count, loop, ...)                                                \
    struct struct_name {                                                                                               \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, ITM_SERIALIZATION_IMPL_NOTHING,     \
        		                           __VA_ARGS__))                                                               \
        std::string origin = "";                                                                                       \
        struct_name () = default;                                                                                      \
        struct_name(ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _,                        \
                                          ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)), const std::string& origin = ""):\
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _, ITM_SERIALIZATION_IMPL_COMMA,\
                                               __VA_ARGS__)),                                                          \
            origin(origin)                                                                                             \
            {}                                                                                                         \
        explicit struct_name (const boost::program_options::variables_map& vm, const std::string& origin = "") :       \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name,                    \
                                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                            \
            origin(origin)                                                                                             \
        {}                                                                                                             \
        static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin = ""){    \
            struct_name default_instance;                                                                              \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _,                   \
                                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
            return {                                                                                                   \
                ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _,                    \
                		                           ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                        \
                origin                                                                                                 \
            };                                                                                                         \
        }                                                                                                              \
        boost::property_tree::ptree ToPTree(const std::string& origin = "") const {                                    \
            boost::property_tree::ptree tree;                                                                          \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _,                          \
                                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
            return tree;                                                                                               \
        }                                                                                                              \
        friend bool operator==(const struct_name & instance1, const struct_name & instance2){                          \
            return                                                                                                     \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _,                           \
            		                                           ITM_SERIALIZATION_IMPL_AND, __VA_ARGS__));              \
        }                                                                                                              \
        friend std::ostream& operator<<(std::ostream& out, const struct_name& instance) {                              \
            boost::property_tree::ptree tree(instance.ToPTree());                                                      \
            boost::property_tree::write_json_no_quotes(out, tree, true);                                               \
            return out;                                                                                                \
        }                                                                                                              \
        static void AddToOptionsDescription(boost::program_options::options_description& od) {                         \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION, _,           \
            		                           ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
        }                                                                                                              \
    }

// Pathless serializable struct
//TODO: avoid DRY violations, reuse above code
#define PATHLESS_SERIALIZABLE_STRUCT_IMPL(struct_name, ...)                                                            \
    PATHLESS_SERIALIZABLE_STRUCT_IMPL_2(struct_name, ITM_SERIALIZATION_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define PATHLESS_SERIALIZABLE_STRUCT_IMPL_2(struct_name, field_count, ...)                                             \
    PATHLESS_SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count,                                                      \
                             ITM_SERIALIZATION_IMPL_CAT(ITM_SERIALIZATION_IMPL_LOOP_, field_count), __VA_ARGS__)


#define PATHLESS_SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count, loop, ...)                                       \
    struct struct_name {                                                                                               \
        ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_DECL, _, ITM_SERIALIZATION_IMPL_NOTHING,     \
        		                                                                    __VA_ARGS__))                      \
        struct_name () = default;                                                                                      \
        struct_name(ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_TYPED_FIELD, _,                        \
        	                                           ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__)),                    \
                    const std::string& origin = ""):                                                                   \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_INIT_FIELD_ARG, _,                             \
            	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__))                             \
            {}                                                                                                         \
        explicit struct_name (const boost::program_options::variables_map& vm, const std::string& origin = "") :       \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT, struct_name,                    \
            	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__))                             \
        {}                                                                                                             \
        static struct_name BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin = ""){    \
            struct_name default_instance;                                                                              \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE, _,                   \
            	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
            return {                                                                                                   \
                ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_FROM_OPTIONAL, _,                    \
                	                               ITM_SERIALIZATION_IMPL_COMMA, __VA_ARGS__))                         \
            };                                                                                                         \
        }                                                                                                              \
        boost::property_tree::ptree ToPTree(const std::string& origin = "") const {                                    \
            boost::property_tree::ptree tree;                                                                          \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE, _,                          \
            	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__))                           \
            return tree;                                                                                               \
        }                                                                                                              \
        friend bool operator==(const struct_name & instance1, const struct_name & instance2){                          \
            return                                                                                                     \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_FIELD_COMPARISON, _,                           \
            	                               ITM_SERIALIZATION_IMPL_AND, __VA_ARGS__));                              \
        }                                                                                                              \
        friend std::ostream& operator<<(std::ostream& out, const struct_name& instance) {                              \
            boost::property_tree::ptree tree(instance.ToPTree());                                                      \
            boost::property_tree::write_json_no_quotes(out, tree, true);                                               \
            return out;                                                                                                \
        }                                                                                                              \
        static void AddToOptionsDescription(boost::program_options::options_description& od) {                         \
            ITM_SERIALIZATION_IMPL_EXPAND(loop(SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION, _,           \
            	                               ITM_SERIALIZATION_IMPL_NOTHING, __VA_ARGS__));                          \
        }                                                                                                              \
    }


// endregion



