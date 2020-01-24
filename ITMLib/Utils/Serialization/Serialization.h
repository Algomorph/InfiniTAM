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
#include "SerializationDetails.h"
/// Macros for making serializable stucts and enums

/**
 * \brief Generate declaration and definition of a  struct that has utilities to be generated from program arguments or
 * a .json tree
 * \details default constructor, constructor with all members in order, constructor based
 * on boost::program_options::variables_map, BuildFromPropertyTree static member function that returns an instance,
 * and a ToPtree static member function, that returns a boost::property_tree::ptree holding all the data.
 *
 * Usage:
 * The first argument specifies the struct name. The subsequent arguments should be triplets of values, in parenthesis,
 * one for each field. Each triplet should have the form (<type>, <name>, <default value>).
 *
 * @ref GENERATE_SERIALIZABLE_STRUCT( MyWonderfulUnicorn, (float, age, 5.0f), (float, horn_length, 0.33f) )
 * Generates a struct called "MyWonderfulUnicorn" with two fields, i.e. public member variables, called
 * "age" and "horn_length". Both are typed as floats and have their respective default values set at 5.0f and 0.33f.
 */
#ifdef _MSC_VER
#define ITM_SERIALIZATION_OUTER_EXPAND(x) x
#define GENERATE_SERIALIZABLE_STRUCT(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_STRUCT_IMPL( __VA_ARGS__ ))
#define GENERATE_PATHLESS_SERIALIZABLE_STRUCT(...) ITM_SERIALIZATION_OUTER_EXPAND(PATHLESS_SERIALIZABLE_STRUCT_IMPL(__VA_ARGS__))
#define DECLARE_SERIALIZABLE_STRUCT(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_STRUCT_DECL_IMPL( __VA_ARGS__ ))
#define DEFINE_SERIALIZABLE_STRUCT(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_STRUCT_DEFN_IMPL( , __VA_ARGS__ ))
#define DEFINE_INNER_SERIALIZABLE_STRUCT(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_STRUCT_DEFN_IMPL( __VA_ARGS__ ))

#define GENERATE_SERIALIZABLE_ENUM(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_ENUM_IMPL( __VA_ARGS__ ))
#define DECLARE_SERIALIZABLE_ENUM(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_ENUM_DECL_IMPL( __VA_ARGS__ ))
#define DEFINE_SERIALIZABLE_ENUM(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_ENUM_DEFN_IMPL( , __VA_ARGS__ ))
#define DEFINE_INLINE_SERIALIZABLE_ENUM(...) ITM_SERIALIZATION_OUTER_EXPAND(SERIALIZABLE_ENUM_DEFN_IMPL( inline , __VA_ARGS__ ))
#else
#define GENERATE_SERIALIZABLE_STRUCT(...) SERIALIZABLE_STRUCT_IMPL( __VA_ARGS__ )
#define GENERATE_PATHLESS_SERIALIZABLE_STRUCT(...) PATHLESS_SERIALIZABLE_STRUCT_IMPL(__VA_ARGS__)
#define DECLARE_SERIALIZABLE_STRUCT(...) SERIALIZABLE_STRUCT_DECL_IMPL( __VA_ARGS__ )
#define DEFINE_SERIALIZABLE_STRUCT(...) SERIALIZABLE_STRUCT_DEFN_IMPL( , __VA_ARGS__ )
#define DEFINE_INNER_SERIALIZABLE_STRUCT(...) SERIALIZABLE_STRUCT_DEFN_IMPL( __VA_ARGS__ )

#define GENERATE_SERIALIZABLE_ENUM(...) SERIALIZABLE_ENUM_IMPL( __VA_ARGS__ )
#define DECLARE_SERIALIZABLE_ENUM(...) SERIALIZABLE_ENUM_DECL_IMPL( __VA_ARGS__ )
#define DEFINE_SERIALIZABLE_ENUM(...) SERIALIZABLE_ENUM_DEFN_IMPL( , __VA_ARGS__ )
#define DEFINE_INLINE_SERIALIZABLE_ENUM(...) SERIALIZABLE_ENUM_DEFN_IMPL( inline , __VA_ARGS__ )
#endif
