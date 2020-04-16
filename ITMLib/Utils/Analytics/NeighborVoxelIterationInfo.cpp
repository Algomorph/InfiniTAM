//  ================================================================
//  Created by Gregory Kramida on 2/5/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "NeighborVoxelIterationInfo.h"


bool ITMLib::NeighborVoxelIterationInfo::operator==(const ITMLib::NeighborVoxelIterationInfo& rhs) const {

	return this == &rhs ||
	       (this->notAllocated == rhs.notAllocated && this->unknown == rhs.unknown && this->hash == rhs.hash &&
	        this->localId == rhs.localId && this->sdf == rhs.sdf && this->struckKnownVoxels == rhs.struckKnownVoxels
	        && this->struckNarrowBand == rhs.struckNarrowBand && this->liveSdf == rhs.liveSdf && this->warp == rhs.warp
	        && this->warpGradient == rhs.warpGradient);
}

bool ITMLib::NeighborVoxelIterationInfo::operator!=(const ITMLib::NeighborVoxelIterationInfo& rhs) const {
	return !(*this == rhs);
}

bool ITMLib::ITMHighlightIterationInfo::operator==(const ITMLib::ITMHighlightIterationInfo& rhs) const {
	if (this == &rhs) {
		return true;
	}

	if (this->hash != rhs.hash || this->localId != rhs.localId ||
	    this->sdf != rhs.sdf || this->liveSdf != rhs.liveSdf || this->warp != rhs.warp ||
	    this->warpUpdate != rhs.warpUpdate || this->warpUpdateData != rhs.warpUpdateData ||
	    this->warpUpdateLevelSet != rhs.warpUpdateLevelSet || this->warpUpdateKilling != rhs.warpUpdateKilling ||
	    this->voxelEnergy != rhs.voxelEnergy || this->voxelEnergyData != rhs.voxelEnergyData ||
	    this->voxelEnergyLevelSet != rhs.voxelEnergyLevelSet || this->voxelEnergyKilling != rhs.voxelEnergyKilling ||
	    this->voxelEnergySmoothness != rhs.voxelEnergySmoothness || this->liveJacobian != rhs.liveJacobian ||
	    this->warpedJacobian != rhs.warpedJacobian || this->warpedHessian != rhs.warpedHessian ||
	    this->warpJacobian != rhs.warpJacobian || this->warpHessianU != rhs.warpHessianU ||
	    this->warpHessianV != rhs.warpHessianV || this->warpHessianW != rhs.warpHessianW) {
		return false;
	};
	for (int iNeighbor = 0; iNeighbor < this->neighbors.size(); iNeighbor++) {
		if (neighbors[iNeighbor] != rhs.neighbors[iNeighbor]) return false;
	}
	return true;
}

bool ITMLib::ITMHighlightIterationInfo::operator!=(const ITMLib::ITMHighlightIterationInfo& rhs) const {
	return !(*this == rhs);
}

namespace ITMLib {
std::ostream& operator<<(std::ostream& stream, const ITMHighlightIterationInfo& highlight) {
	stream << "[highlight| sdf:" << highlight.sdf << ",liveSdf:"
	       << highlight.liveSdf << ",warp:" << highlight.warp << ",warpUpdate:" << highlight.warpUpdate
	       << ",warpUpdateData:" << highlight.warpUpdateData << ",warpUpdateLevelSet:" << highlight.warpUpdateLevelSet
	       << ",warpUpdateKilling: " << highlight.warpUpdateKilling << "]";
	return stream;
}
}//namespace ITMLib
