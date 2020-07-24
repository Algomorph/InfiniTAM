//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/24/20.
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
//local
#include "SerializableStruct_Impl.h"

#ifndef MACRO_END
#define MACRO_END() static_assert(true, "")
#define ITM_METACODING_OUTER_EXPAND(x) x
#endif

// *** DECLARATION-ONLY ***

#define PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL(struct_name, ...) \
    PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, field_count, ...) \
    PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

#define PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, loop, ...) \
    struct struct_name { \
        PARSE_PATH() \
        PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_BODY(struct_name, loop, __VA_ARGS__) \
    }

#define PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL_BODY(struct_name, loop, ...) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_FUNCS(struct_name, loop, __VA_ARGS__)

// *** DEFINITION-ONLY ***

#define PATHLESS_SERIALIZABLE_STRUCT_DEFN_IMPL(outer_class, struct_name, ...) \
    PATHLESS_SERIALIZABLE_STRUCT_DEFN_IMPL_2( outer_class, struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)


#define PATHLESS_SERIALIZABLE_STRUCT_DEFN_IMPL_2(outer_class, struct_name, field_count, ...) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_3(SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(outer_class), \
                             SERIALIZABLE_STRUCT_DEFN_HANDLE_QUALIFIER(struct_name), struct_name, , , ,\
                             ITM_METACODING_IMPL_NOTHING, , , , \
                             ITM_METACODING_IMPL_COMMA, parse_path(std::move(parse_path)), ,\
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)

// *** FULL STRUCT GENERATION ***
#define PATHLESS_SERIALIZABLE_STRUCT_IMPL(struct_name, ...) \
    PATHLESS_SERIALIZABLE_STRUCT_IMPL_2(struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define PATHLESS_SERIALIZABLE_STRUCT_IMPL_2(struct_name, field_count, ...) \
    PATHLESS_SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)


#define PATHLESS_SERIALIZABLE_STRUCT_IMPL_3(struct_name, field_count, loop, ...) \
    struct struct_name { \
        PARSE_PATH() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DEFN_IMPL_3( , , struct_name, , friend, static, ITM_METACODING_IMPL_NOTHING, , , ="",\
        ITM_METACODING_IMPL_COMMA, parse_path(std::move(parse_path)), ="", loop, __VA_ARGS__) \
    };

