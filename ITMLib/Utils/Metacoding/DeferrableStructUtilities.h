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

//stdlib
#include <string>
//boost
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>


namespace ITMLib {


template<typename TDeferrableSerializableStruct>
inline void AddDeferrableFromSourceToTargetTree(boost::property_tree::ptree& target_tree,
                                                       const boost::property_tree::ptree& source_tree,
                                                       std::string origin = "") {
	boost::property_tree::ptree deferrable_subtree;
	auto subtree = source_tree.get_child_optional(TDeferrableSerializableStruct::default_parse_path);
	if (subtree) {
		deferrable_subtree = subtree.get();
	} else {
		deferrable_subtree = TDeferrableSerializableStruct().ToPTree(origin);
	}
	target_tree.add_child(TDeferrableSerializableStruct::default_parse_path, deferrable_subtree);
}

template<typename TDeferrableSerializableStruct>
inline void AddDeferrableToTargetTree(boost::property_tree::ptree& target_tree,
                                                const TDeferrableSerializableStruct& deferrable,
                                                std::string origin = "") {
	boost::property_tree::ptree deferrable_subtree = deferrable.ToPTree(origin);
	target_tree.add_child(TDeferrableSerializableStruct::default_parse_path, deferrable_subtree);
}

template<typename TDeferrableChild, typename TDeferrableParent>
inline TDeferrableChild BuildDeferrableFromParentIfPresent(const TDeferrableParent& parent) {
	const boost::property_tree::ptree& source_tree = parent.source_tree;
	auto subtree = source_tree.get_child_optional(TDeferrableChild::default_parse_path);
	if (subtree) {
		return TDeferrableChild::BuildFromPTree(subtree.get(), parent.origin);
	} else {
		return TDeferrableChild();
	}
}

} // namespace ITMLib