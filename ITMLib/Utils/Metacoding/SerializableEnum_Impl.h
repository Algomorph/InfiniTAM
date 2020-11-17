//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/6/20.
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

#include "SequenceLevel1Macros.h"
#include "PreprocessorNargs.h"
#include "MetacodingAuxiliaryUtilities.h"
#include "../../../ORUtils/PlatformIndependence.h"

// region ================== SERIALIZABLE ENUM TEMPLATED FUNCTION DEFINITIONS ==========================================

template<typename TEnum>
TEnum string_to_enumerator(const std::string& string);

template<typename TEnum>
std::string enumerator_to_string(const TEnum& enum_value);

template<typename TEnum>
std::string enumerator_bracketed_list();

template<typename TEnum>
std::vector<std::string> enumerator_to_string_token_list(const TEnum& enum_value);

template<typename TEnum>
TEnum variables_map_to_enumerator(const boost::program_options::variables_map& vm, const std::string& argument) {
	return string_to_enumerator<TEnum>(vm[argument].as<std::string>());
}

template<typename TEnum>
boost::optional<TEnum> ptree_to_optional_enumerator(const boost::property_tree::ptree& ptree, const boost::property_tree::ptree::key_type& key) {
	auto child = ptree.get_child_optional(key);
	if (child) {
		return boost::optional<TEnum>(string_to_enumerator<TEnum>(ptree.get<std::string>(key)));
	} else {
		return boost::optional<TEnum>{};
	}
}

// endregion

// region ================== SERIALIZABLE ENUM PER-TOKEN AND PER-ENUMERATOR MACROS ======================================

// these top ones are per-token, not per-enumerator
#define SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS(qualified_enumerator, token) { token , qualified_enumerator }
#define SERIALIZABLE_ENUM_IMPL_LIST_STRING_TOKEN(_, token, ...) token

#define SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATOR(_, enumerator, ...) enumerator
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS(enum_name, enumerator, ...) \
    SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_2(enum_name, enumerator, token_count, ...) \
    SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator, \
                                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP2_, token_count), __VA_ARGS__)
#define SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS_3(enum_name, enumerator, loop, ...) \
    ITM_METACODING_IMPL_EXPAND2(loop(SERIALIZABLE_ENUM_IMPL_GEN_TOKEN_MAPPINGS, enum_name::enumerator, ITM_METACODING_IMPL_COMMA,__VA_ARGS__))

#define SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE(enum_name, enumerator, first_token, ...) \
    case enum_name::enumerator : token = first_token; break;

#define SERIALIZABLE_ENUM_IMPL_GENERATE_TOKEN_LIST(...) \
    ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP2_, ITM_METACODING_IMPL_NARG(__VA_ARGS__)) \
                (SERIALIZABLE_ENUM_IMPL_LIST_STRING_TOKEN, _, ITM_METACODING_IMPL_COMMA, __VA_ARGS__)

#define SERIALIZABLE_ENUM_IMPL_TOKEN_LIST_SWITCH_CASE(enum_name, enumerator, ...) \
    case enum_name::enumerator : return { SERIALIZABLE_ENUM_IMPL_GENERATE_TOKEN_LIST(__VA_ARGS__) };

// endregion
// region ================== SERIALIZABLE ENUM TOP-LEVEL MACROS ========================================================

#define SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(...) \
    ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, ITM_METACODING_IMPL_NARG(__VA_ARGS__)) \
                (SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATOR, _, ITM_METACODING_IMPL_COMMA, __VA_ARGS__)

//** declaration
#define SERIALIZABLE_ENUM_DECL_IMPL(enum_name, ...) \
    enum enum_name { \
        SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(__VA_ARGS__) \
    };

//** definition
#define SERIALIZABLE_ENUM_DEFN_IMPL(INLINE, enum_name, ...) \
    SERIALIZABLE_ENUM_DEFN_IMPL_2(INLINE, enum_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define SERIALIZABLE_ENUM_DEFN_IMPL_2(INLINE, enum_name, field_count, ...) \
    SERIALIZABLE_ENUM_DEFN_IMPL_3(INLINE, enum_name, field_count, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)


#define SERIALIZABLE_ENUM_DEFN_IMPL_3(INLINE, enum_name, field_count, loop, ...) \
    template<> \
    INLINE enum_name string_to_enumerator< enum_name >(const std::string& string) { \
        static std::unordered_map<std::string, enum_name > enumerator_by_string = { \
                ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_ENUM_IMPL_STRING_MAPPINGS, enum_name, \
                	                               ITM_METACODING_IMPL_COMMA, __VA_ARGS__)) \
        }; \
        if (enumerator_by_string.find(string) == enumerator_by_string.end()) { \
            DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized string token for enum " #enum_name); \
        } \
        return enumerator_by_string[string]; \
    } \
    template<> \
    INLINE std::string enumerator_to_string< enum_name >( const enum_name & value)    { \
        std::string token; \
        switch(value) { \
            ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_ENUM_IMPL_STRING_SWITCH_CASE, enum_name, \
            	                               ITM_METACODING_IMPL_NOTHING, __VA_ARGS__)) \
        } \
        return token; \
    } \
    template<> \
    INLINE std::string enumerator_bracketed_list< enum_name >(){ \
        return "[" BOOST_PP_STRINGIZE(BOOST_PP_CAT2(SERIALIZABLE_ENUM_IMPL_LIST_ENUMERATORS(__VA_ARGS__))) "]"; \
    } \
    template<> \
    INLINE std::vector<std::string> enumerator_to_string_token_list< enum_name >(const enum_name & value){ \
        switch(value) { \
            ITM_METACODING_IMPL_EXPAND(loop(SERIALIZABLE_ENUM_IMPL_TOKEN_LIST_SWITCH_CASE, enum_name, \
            	                               ITM_METACODING_IMPL_NOTHING, __VA_ARGS__)) \
        } \
        return {};                                                                             \
    }


#define SERIALIZABLE_ENUM_IMPL(enum_name, ...) \
    SERIALIZABLE_ENUM_DECL_IMPL(enum_name, __VA_ARGS__) \
    SERIALIZABLE_ENUM_DEFN_IMPL(,enum_name, __VA_ARGS__)
// endregion