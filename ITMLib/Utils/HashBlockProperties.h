//  ================================================================
//  Created by Gregory Kramida on 11/30/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
namespace ITMLib {
enum HashBlockVisibility : unsigned char {
	VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED = 3,
	STREAMED_OUT_AND_VISIBLE = 2,
	IN_MEMORY_AND_VISIBLE = 1,
	INVISIBLE = 0
};
enum HashEntryAllocationState : unsigned char {
	NEEDS_NO_CHANGE = 0,
	NEEDS_ALLOCATION_IN_ORDERED_LIST = 1,
	NEEDS_ALLOCATION_IN_EXCESS_LIST = 2,
};
enum HashBlockPtrFlag : int {
	ENTRY_REMOVED = -1,
	ENTRY_EMPTY_POSITIVE = -2,
	ENTRY_EMPTY_NEGATIVE = -3
};
}//end namespace ITMLib
