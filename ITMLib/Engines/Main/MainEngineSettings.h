//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/21/20.
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
#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"
#include "../../Utils/Metacoding/SerializableEnum.h"
#include "../../Utils/Enums/IndexingMethod.h"

namespace ITMLib {

#define LIBMODE_ENUM_DESCRIPTION LibMode, \
    (LIBMODE_BASIC, "basic"), \
    (LIBMODE_BASIC_SURFELS, "surfels"), \
    (LIBMODE_LOOPCLOSURE, "loop_closure"), \
    (LIBMODE_DYNAMIC, "dynamic")

DECLARE_SERIALIZABLE_ENUM(LIBMODE_ENUM_DESCRIPTION);

#define MAIN_ENGINE_SETTINGS_STRUCT_DESCRIPTION MainEngineSettings, "main_engine_settings",\
    (bool, draw_frame_index_labels, false, PRIMITIVE, "Draw index labels on top of output frame images"), \
    (LibMode, library_mode, LIBMODE_DYNAMIC, ENUM, "Switch between various library modes - basic, with loop closure, etc."), \
    (IndexingMethod, indexing_method, INDEX_HASH, ENUM, "Indexing method to use in the 3D volumes, i.e. array or hash."),    \
    (bool, halt_on_non_rigid_alignment_convergence_failure, false, PRIMITIVE, "Whether to halt on non-rigid alignment optimization convergence failure"), \
    (bool, enable_rigid_alignment, true, PRIMITIVE, "Enables or disables rigid (camera) tracking/alignment.")


DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(MAIN_ENGINE_SETTINGS_STRUCT_DESCRIPTION);

} // namespace ITMLib