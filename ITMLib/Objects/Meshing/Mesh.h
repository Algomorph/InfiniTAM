//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/27/20.
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
#include "../../Utils/Math.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {
class Mesh {
public: // inner classes / structs
	struct Triangle {
		Vector3f p0, p1, p2;
		_CPU_AND_GPU_CODE_
		friend bool operator==(const Triangle& triangle1, const Triangle& triangle2);
		_CPU_AND_GPU_CODE_
		friend bool operator!=(const Triangle& triangle1, const Triangle& triangle2);
		_CPU_AND_GPU_CODE_
		friend bool AlmostEqual(const Triangle& triangle1, const Triangle& triangle2, const float tolerance);

		_CPU_AND_GPU_CODE_
		friend bool operator<(const Triangle& a, const Triangle& b) {
			if (a.p2 == b.p2) {
				if (a.p1 == b.p1) {
					return a.p0 < b.p0;
				} else {
					return a.p1 < b.p1;
				}
			} else {
				return a.p2 < b.p2;
			}
		}
	};
public: // instance variables

private: // instance variables
	ORUtils::MemoryBlock<Triangle> triangles;
	unsigned int triangle_count;
public: // instance functions
	Mesh();
	Mesh(const Mesh& other, MemoryDeviceType memory_type);
	Mesh(ORUtils::MemoryBlock<Triangle>& triangles, unsigned int triangle_count);
	MemoryDeviceType GetMemoryDeviceType() const;

	friend bool AlmostEqual(const Mesh& mesh1, const Mesh& mesh2, const float tolerance, bool presort_triangles);

	void WriteOBJ(const std::string& path);

	void WriteSTL(const std::string& path);

	void WritePLY(const std::string& path, bool ascii = false, bool use_compression = false);

private: // instance functions
	template<typename TWriteTriangleArrayFunction>
	void GenericWriteToDisk(TWriteTriangleArrayFunction&& write_triangle_array);
};

bool AlmostEqual(const Mesh& mesh1, const Mesh& mesh2, const float tolerance, bool presort_triangles = true);

} // namespace ITMLib



