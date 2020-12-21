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
IStreamWrapper& operator>>(IStreamWrapper& source, Image<bool>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<char>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<unsigned char>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<short>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<unsigned short>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<int>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<unsigned int>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<unsigned long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<long long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<unsigned long long>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<float>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<double>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector2<unsigned int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector2<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector2<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector2<float>>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector3<unsigned char>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector3<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector3<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector3<float>>& memory_block);

template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector4<unsigned char>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector4<short>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector4<int>>& memory_block);
template
IStreamWrapper& operator>>(IStreamWrapper& source, Image<Vector4<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<bool>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<char>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<unsigned char>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<short>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<unsigned short>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<int>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<unsigned int>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<unsigned long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<long long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<unsigned long long>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<float>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<double>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector2<unsigned int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector2<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector2<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector2<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector3<unsigned char>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector3<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector3<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector3<float>>& memory_block);

template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector4<unsigned char>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector4<short>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector4<int>>& memory_block);
template
OStreamWrapper& operator<<(OStreamWrapper& destination, const Image<Vector4<float>>& memory_block);

}// namespace ORUtils