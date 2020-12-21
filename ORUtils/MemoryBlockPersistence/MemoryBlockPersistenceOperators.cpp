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
#include "MemoryBlockPersistenceOperators.tpp"

namespace ORUtils {
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<bool>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<char>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<unsigned char>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<short>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<unsigned short>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<int>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<unsigned int>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<unsigned long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<long long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<unsigned long long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<float>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<double>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector2<unsigned int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector2<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector2<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector2<float>>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector3<unsigned char>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector3<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector3<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector3<float>>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector4<unsigned char>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector4<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector4<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, MemoryBlock<Vector4<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<bool>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<char>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<unsigned char>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<short>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<unsigned short>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<int>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<unsigned int>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<unsigned long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<long long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<unsigned long long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<float>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<double>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector2<unsigned int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector2<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector2<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector2<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector3<unsigned char>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector3<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector3<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector3<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector4<unsigned char>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector4<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector4<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const MemoryBlock<Vector4<float>>& memory_block);

}// namespace ORUtils