//  ================================================================
//  Created by Gregory Kramida on 01/03/20.
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

// inspired in part by InifinTAM/Utils/ITMSceneParameters.h of the original InfiniTAM repository, Oxford University

#pragma once

//local
#include "Serialization/Serialization.h"

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {

/** \brief
	Stores parameters of a voxel volume, such as voxel size
*/
GENERATE_SERIALIZABLE_STRUCT(VoxelVolumeParameters,
        /** Size of a voxel, usually given in meters.*/
                             (float, voxel_size, 0.004),
		/** \brief
		    Fallback parameters: consider only parts of the
		    scene from @p near_clipping_distance in front of the camera
		    to a distance of @p far_clipping_distance. Usually the
		    actual depth range should be determined
		    automatically by a ITMLib::Engine::ITMVisualisationEngine.
		*/
                             (float, near_clipping_distance, 0.2f),
                             (float, far_clipping_distance, 3.0f),

		/** \brief
		    Encodes the width of the band of the truncated
		    signed distance transform that is actually stored
		    in the volume. This is again usually specified in
		    meters. The resulting width in voxels is @ref narrow_band_half_width
		    divided by @ref voxel_size.
		*/
                             (float, narrow_band_half_width, 0.04f),
		/** \brief
		 * Up to @ref max_integration_weight observations per voxel are averaged.
		 * Beyond that a sliding average is computed.
		 */
                             (int, max_integration_weight, 100),
        /** Stop integration/fusion once max_integration_weight has been reached. */
                             (bool, stop_integration_at_max_weight, false),
        /**
         * \brief (voxel block hash indexing only) allocates an extra 1-ring of hash blocks around
         * the depth-based ones
         */
                             (bool, add_extra_block_ring_during_allocation, false),
         /**
          * \brief (voxel block hash indexing only) factor of narrow band width that will be
          * considered for depth-based allocation. For instance, a factor of 2 will make sure that blocks that are twice
          * as far from the surface as the boundary of the narrow (non-truncated) TSDF band will be allocated
          */
                             (float, block_allocation_band_factor, 2.0f)
);

} // namespace ITMLib
