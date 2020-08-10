//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/8/20.
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
#include "../../Utils/Geometry/CardinalAxesAndPlanes.h"

namespace ITMLib{

#define TELEMETRY_SETTINGS_STRUCT_DESCRIPTION TelemetrySettings, "telemetry_settings", \
    (bool, record_volume_memory_usage, false, PRIMITIVE, "Whether to record information required to debug memory" \
    " usage, e.g. used block locations for the VoxelBlockHash index."), \
    (bool, record_live_volume_as_2D_slices, false, PRIMITIVE, "Whether to record 2D slices (images, with pixel " \
    "representing TSDF value) of the live volume once per frame before the beginning of the surface tracking optimization."), \
    (bool, record_canonical_volume_as_2D_slices, false, PRIMITIVE, "Whether to record 2D slices (images, with pixel " \
    "representing TSDF value) of the canonical volume once per frame before the beginning of the surface tracking optimization."), \
    (bool, record_live_focus_point_TSDF_graph, false, PRIMITIVE, "Whether to record graphs of SDF value of a single " \
    "voxel plotted against iteration number of the surface tracking optimization."), \
    (bool, record_live_focus_layer_TSDF_heatmap, false, PRIMITIVE, "Whether to record TSDF heatmaps for the voxel layer in " \
    "the warped live volume (in the plane specified by TSDF_heatmap_plane parameter) for each iteration of the surface" \
    "tracking optimization."), \
    (Plane, TSDF_heatmap_plane, PLANE_ZX, ENUM, "Plane in which to record TSDF heatmaps " \
    "(see record_live_focus_layer_TSDF_heatmap parameter help)."), \
    (bool, record_focus_neighborhood_live_tsdf_sequence, false, PRIMITIVE, "Whether to record a sequence of TSDF " \
    "volumes representing the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) " \
    "in the warped live frame over the course of the entire surface tracking optimization. [WITH_VTK compilation required!]"), \
    (int, focus_neighborhood_size, 3, PRIMITIVE, "For focus neighborhood recording: a cube of size " \
    "2 x focus_neighborhood_size + 1 around the focus_coordinates will be recorded. [WITH_VTK compilation required!]"), \
    (bool, record_focus_neighborhood_warp_sequence, false, PRIMITIVE, "Whether to record a sequence of warp " \
    "vectors in the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) over " \
    "the course of the entire surface tracking optimization. [WITH_VTK compilation required!]"), \
    (bool, record_surface_tracking_optimization_energies, false, PRIMITIVE, "Whether to record optimization energies " \
    "for each iteration of the surface tracking optimization in a separate file. Only works when non_rigid_tracking_parameters.functor_type " \
    "parameter is set to \"slavcheva_diagnostic\""), \
    (bool, record_frame_meshes, false, PRIMITIVE, "Whether to log three meshes at every frame: (a) from live volume " \
    "after camera tracking and before surface tracking, (b) from live volume after surface tracking, and (c) from " \
    "canonical volume after fusion."), \
    (bool, use_CPU_for_mesh_recording, false, PRIMITIVE, "Whether to ALWAYS use CPU & regular RAM when recording mesh telemetry. For CUDA runs, this will reduce GPU memory usage."), \
    (bool, record_camera_matrices, false, PRIMITIVE, "Whether to record estimated camera trajectory matrices in world space.")


DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION);


} // namespace ITMLib