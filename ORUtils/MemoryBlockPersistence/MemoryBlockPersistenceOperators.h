//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/21/20.
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
#include "../OStreamWrapper.h"
#include "../IStreamWrapper.h"
#include "../MemoryBlock.h"
#include "../Image.h"

namespace ORUtils {

template<typename TElementType>
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<TElementType>& memory_block);
template<typename TElementType>
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<TElementType>& memory_block);
template<typename TElementType>
IStreamWrapper& operator>>(IStreamWrapper& source, Image<TElementType>& image);
template<typename TElementType>
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<TElementType>& image);

} // namespace ORUtils