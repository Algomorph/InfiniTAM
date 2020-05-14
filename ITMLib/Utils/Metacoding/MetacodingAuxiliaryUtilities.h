//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/14/20.
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

// region ==== LOW-LEVEL MACROS
#define ITM_METACODING_IMPL_SEMICOLON() ;
#define ITM_METACODING_IMPL_COMMA() ,
#define ITM_METACODING_IMPL_AND() &&
#define ITM_METACODING_IMPL_NOTHING()

#define ITM_METACODING_IMPL_PRIMITIVE_CAT(a, ...) a##__VA_ARGS__
#define ITM_METACODING_IMPL_CAT(a, ...) ITM_METACODING_IMPL_PRIMITIVE_CAT(a, __VA_ARGS__)

#define ITM_METACODING_IMPL_ARG16(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, ...) _15
#define ITM_METACODING_IMPL_HAS_COMMA(...) ITM_METACODING_IMPL_EXPAND(ITM_METACODING_IMPL_ARG16(__VA_ARGS__, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0))
#define ITM_METACODING_IMPL_TRIGGER_PARENTHESIS_(...) ,

#define ITM_METACODING_IMPL_ISEMPTY(...) \
ITM_METACODING_IMPL_PRIMITIVE_ISEMPTY( \
          /* test if there is just one argument, eventually an empty \
             one */ \
          ITM_METACODING_IMPL_HAS_COMMA(__VA_ARGS__), \
          /* test if _TRIGGER_PARENTHESIS_ together with the argument \
             adds a comma */ \
          ITM_METACODING_IMPL_HAS_COMMA(_TRIGGER_PARENTHESIS_ __VA_ARGS__),\
          /* test if the argument together with a parenthesis \
             adds a comma */ \
          ITM_METACODING_IMPL_HAS_COMMA(__VA_ARGS__ (/*empty*/)), \
          /* test if placing it between _TRIGGER_PARENTHESIS_ and the \
             parenthesis adds a comma */ \
          ITM_METACODING_IMPL_HAS_COMMA(ITM_METACODING_IMPL_TRIGGER_PARENTHESIS_ __VA_ARGS__ (/*empty*/)) \
          )

#define ITM_METACODING_IMPL_PASTE5(_0, _1, _2, _3, _4) _0 ## _1 ## _2 ## _3 ## _4
#define ITM_METACODING_IMPL_PRIMITIVE_ISEMPTY(_0, _1, _2, _3) ITM_METACODING_IMPL_HAS_COMMA(ITM_METACODING_IMPL_PASTE5(ITM_METACODING_IMPL_IS_EMPTY_CASE_, _0, _1, _2, _3))
#define ITM_METACODING_IMPL_IS_EMPTY_CASE_0001 ,

// endregion