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
#include "PathlessSerializableStruct_Impl.h"

#ifdef _MSC_VER
#define ITM_METACODING_OUTER_EXPAND(x) x
#define GENERATE_PATHLESS_SERIALIZABLE_STRUCT(...) ITM_METACODING_OUTER_EXPAND(PATHLESS_SERIALIZABLE_STRUCT_IMPL( __VA_ARGS__ )); MACRO_END()
#define DECLARE_PATHLESS_SERIALIZABLE_STRUCT(...) ITM_METACODING_OUTER_EXPAND(PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL( __VA_ARGS__ )); MACRO_END()
#define DEFINE_PATHLESS_SERIALIZABLE_STRUCT(...) ITM_METACODING_OUTER_EXPAND(PATHLESS_SERIALIZABLE_STRUCT_DEFN_IMPL( , __VA_ARGS__ )) MACRO_END()
#else
#define GENERATE_PATHLESS_SERIALIZABLE_STRUCT(...) PATHLESS_SERIALIZABLE_STRUCT_IMPL( __VA_ARGS__ ); MACRO_END()
#define DECLARE_PATHLESS_SERIALIZABLE_STRUCT(...) PATHLESS_SERIALIZABLE_STRUCT_DECL_IMPL( __VA_ARGS__ ); MACRO_END()
#define DEFINE_PATHLESS_SERIALIZABLE_STRUCT(...) PATHLESS_SERIALIZABLE_STRUCT_DEFN_IMPL( , __VA_ARGS__ ) MACRO_END()
#endif