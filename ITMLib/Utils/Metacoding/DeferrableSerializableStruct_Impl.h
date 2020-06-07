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

// *** declaration-only ***
#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL(struct_name, parse_path,  ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, parse_path, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_2(struct_name, parse_path, field_count, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, parse_path, ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)


#define DEFERRABLE_SERIALIZABLE_STRUCT_DECL_IMPL_3(struct_name, parse_path, loop, ...) \
    struct struct_name { \
        static constexpr const char* default_parse_path = parse_path; \
        SERIALIZABLE_STRUCT_DECL_IMPL_BODY(struct_name, loop, __VA_ARGS__) \
    }

// *** definition-only ***

#define DEFERRABLE_SERIALIZABLE_STRUCT_DEFN_IMPL(outer_class, struct_name, parse_path, ...) \
    SERIALIZABLE_STRUCT_DEFN_IMPL_2( outer_class, struct_name, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)



#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL(struct_name, parse_path, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_2(struct_name, parse_path, ITM_METACODING_IMPL_NARG(__VA_ARGS__), __VA_ARGS__)

#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_2(struct_name, parse_path, field_count, ...) \
    DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_3(struct_name, parse_path, \
                             ITM_METACODING_IMPL_CAT(ITM_METACODING_IMPL_LOOP_, field_count), __VA_ARGS__)


#define DEFERRABLE_SERIALIZABLE_STRUCT_IMPL_3(struct_name, parse_path, loop, ...) \
    struct struct_name { \
        static constexpr const char* = parse_path;\
        ORIGIN_AND_SOURCE_TREE() \
        SERIALIZABLE_STRUCT_DECL_IMPL_MEMBER_VARS(loop, __VA_ARGS__) \
        SERIALIZABLE_STRUCT_DEFN_IMPL_3 ( , , struct_name, default_instance.source_tree=tree, friend, static, \
        ITM_METACODING_IMPL_COMMA, origin(std::move(origin)), origin, ="", loop, __VA_ARGS__) \
    };
