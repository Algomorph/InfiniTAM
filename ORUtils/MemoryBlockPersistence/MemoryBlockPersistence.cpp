//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/23/20.
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
//local
#include "MemoryBlockPersistence.h"
#include "../Vector.h"

namespace ORUtils {

//TODO: internalize the Save/Load functions, leave only operators >> and << in the ORUtils namespace (this will involve massive refactoring
// of ITMLib code).

template void MemoryBlockPersistence::SaveImage<bool>(OStreamWrapper& file, const ORUtils::Image<bool>& image);
template void MemoryBlockPersistence::SaveImage<char>(OStreamWrapper& file, const ORUtils::Image<char>& image);
template void MemoryBlockPersistence::SaveImage<unsigned char>(OStreamWrapper& file, const ORUtils::Image<unsigned char>& image);
template void MemoryBlockPersistence::SaveImage<short>(OStreamWrapper& file, const ORUtils::Image<short>& image);
template void MemoryBlockPersistence::SaveImage<unsigned short>(OStreamWrapper& file, const ORUtils::Image<unsigned short>& image);
template void MemoryBlockPersistence::SaveImage<int>(OStreamWrapper& file, const ORUtils::Image<int>& image);
template void MemoryBlockPersistence::SaveImage<unsigned int>(OStreamWrapper& file, const ORUtils::Image<unsigned int>& image);
template void MemoryBlockPersistence::SaveImage<long>(OStreamWrapper& file, const ORUtils::Image<long>& image);
template void MemoryBlockPersistence::SaveImage<unsigned long>(OStreamWrapper& file, const ORUtils::Image<unsigned long>& image);
template void MemoryBlockPersistence::SaveImage<long long>(OStreamWrapper& file, const ORUtils::Image<long long>& image);
template void MemoryBlockPersistence::SaveImage<unsigned long long>(OStreamWrapper& file, const ORUtils::Image<unsigned long long>& image);
template void MemoryBlockPersistence::SaveImage<float>(OStreamWrapper& file, const ORUtils::Image<float>& image);
template void MemoryBlockPersistence::SaveImage<double>(OStreamWrapper& file, const ORUtils::Image<double>& image);

template void MemoryBlockPersistence::SaveImage<Vector2<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector2<unsigned char>>& image);
template void MemoryBlockPersistence::SaveImage<Vector2<short>>(OStreamWrapper& file, const ORUtils::Image<Vector2<short>>& image);
template void MemoryBlockPersistence::SaveImage<Vector2<int>>(OStreamWrapper& file, const ORUtils::Image<Vector2<int>>& image);
template void MemoryBlockPersistence::SaveImage<Vector2<float>>(OStreamWrapper& file, const ORUtils::Image<Vector2<float>>& image);

template void MemoryBlockPersistence::SaveImage<Vector3<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector3<unsigned char>>& image);
template void MemoryBlockPersistence::SaveImage<Vector3<short>>(OStreamWrapper& file, const ORUtils::Image<Vector3<short>>& image);
template void MemoryBlockPersistence::SaveImage<Vector3<int>>(OStreamWrapper& file, const ORUtils::Image<Vector3<int>>& image);
template void MemoryBlockPersistence::SaveImage<Vector3<float>>(OStreamWrapper& file, const ORUtils::Image<Vector3<float>>& image);

template void MemoryBlockPersistence::SaveImage<Vector4<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector4<unsigned char>>& image);
template void MemoryBlockPersistence::SaveImage<Vector4<short>>(OStreamWrapper& file, const ORUtils::Image<Vector4<short>>& image);
template void MemoryBlockPersistence::SaveImage<Vector4<int>>(OStreamWrapper& file, const ORUtils::Image<Vector4<int>>& image);
template void MemoryBlockPersistence::SaveImage<Vector4<float>>(OStreamWrapper& file, const ORUtils::Image<Vector4<float>>& image);

template ORUtils::Image<bool> MemoryBlockPersistence::LoadImage<bool>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<char> MemoryBlockPersistence::LoadImage<char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<unsigned char> MemoryBlockPersistence::LoadImage<unsigned char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<short> MemoryBlockPersistence::LoadImage<short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<unsigned short> MemoryBlockPersistence::LoadImage<unsigned short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<int> MemoryBlockPersistence::LoadImage<int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<unsigned int> MemoryBlockPersistence::LoadImage<unsigned int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<long> MemoryBlockPersistence::LoadImage<long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<unsigned long> MemoryBlockPersistence::LoadImage<unsigned long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<long long> MemoryBlockPersistence::LoadImage<long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<unsigned long long> MemoryBlockPersistence::LoadImage<unsigned long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<float> MemoryBlockPersistence::LoadImage<float>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<double> MemoryBlockPersistence::LoadImage<double>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::Image<Vector2<unsigned char>> MemoryBlockPersistence::LoadImage<Vector2<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector2<short>> MemoryBlockPersistence::LoadImage<Vector2<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector2<int>> MemoryBlockPersistence::LoadImage<Vector2<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector2<float>> MemoryBlockPersistence::LoadImage<Vector2<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::Image<Vector3<unsigned char>> MemoryBlockPersistence::LoadImage<Vector3<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector3<short>> MemoryBlockPersistence::LoadImage<Vector3<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector3<int>> MemoryBlockPersistence::LoadImage<Vector3<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector3<float>> MemoryBlockPersistence::LoadImage<Vector3<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::Image<Vector4<unsigned char>> MemoryBlockPersistence::LoadImage<Vector4<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector4<short>> MemoryBlockPersistence::LoadImage<Vector4<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector4<int>> MemoryBlockPersistence::LoadImage<Vector4<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::Image<Vector4<float>> MemoryBlockPersistence::LoadImage<Vector4<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template void MemoryBlockPersistence::SaveMemoryBlock<bool>(OStreamWrapper& file, const ORUtils::MemoryBlock<bool>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<char>(OStreamWrapper& file, const ORUtils::MemoryBlock<char>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<unsigned char>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned char>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<short>(OStreamWrapper& file, const ORUtils::MemoryBlock<short>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<unsigned short>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned short>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<int>(OStreamWrapper& file, const ORUtils::MemoryBlock<int>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<unsigned int>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned int>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<long>(OStreamWrapper& file, const ORUtils::MemoryBlock<long>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<unsigned long>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned long>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<long long>(OStreamWrapper& file, const ORUtils::MemoryBlock<long long>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<unsigned long long>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned long long>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<float>(OStreamWrapper& file, const ORUtils::MemoryBlock<float>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<double>(OStreamWrapper& file, const ORUtils::MemoryBlock<double>& image);

template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<unsigned char>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<short>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<int>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<float>>& image);

template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<unsigned char>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<short>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<int>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<float>>& image);

template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<unsigned char>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<short>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<int>>& image);
template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<float>>& image);

template ORUtils::MemoryBlock<bool> MemoryBlockPersistence::LoadMemoryBlock<bool>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<char> MemoryBlockPersistence::LoadMemoryBlock<char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<unsigned char> MemoryBlockPersistence::LoadMemoryBlock<unsigned char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<short> MemoryBlockPersistence::LoadMemoryBlock<short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<unsigned short> MemoryBlockPersistence::LoadMemoryBlock<unsigned short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<int> MemoryBlockPersistence::LoadMemoryBlock<int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<unsigned int> MemoryBlockPersistence::LoadMemoryBlock<unsigned int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<long> MemoryBlockPersistence::LoadMemoryBlock<long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<unsigned long> MemoryBlockPersistence::LoadMemoryBlock<unsigned long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<long long> MemoryBlockPersistence::LoadMemoryBlock<long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<unsigned long long> MemoryBlockPersistence::LoadMemoryBlock<unsigned long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<float> MemoryBlockPersistence::LoadMemoryBlock<float>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<double> MemoryBlockPersistence::LoadMemoryBlock<double>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::MemoryBlock<Vector2<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector2<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector2<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector2<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::MemoryBlock<Vector3<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector3<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector3<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector3<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

template ORUtils::MemoryBlock<Vector4<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector4<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector4<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
template ORUtils::MemoryBlock<Vector4<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

} // namespace ORUtils

