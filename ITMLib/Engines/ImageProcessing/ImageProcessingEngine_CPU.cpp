//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/3/21.
//  Copyright (c) 2021 Gregory Kramida
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
#include "ImageProcessingEngine.tpp"
#include "../Traversal/CPU/TwoImageTraversal_CPU.h"
#include "../Traversal/CPU/ImageTraversal_CPU.h"

namespace ITMLib {

template
class ImageProcessingEngine<MEMORYDEVICE_CPU>;

} // namespace ITMLib