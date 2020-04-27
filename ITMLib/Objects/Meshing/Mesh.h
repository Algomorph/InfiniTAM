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

#include "../Volume/VoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib {
class Mesh {
public: // inner classes / structs
	struct Triangle {
		Vector3f p0, p1, p2;
	};
public: // member variables

private: // member variables
	ORUtils::MemoryBlock<Triangle> triangles;
	unsigned int triangle_count;
public: // member functions
	Mesh() : triangle_count(0) {};
	Mesh(ORUtils::MemoryBlock<Triangle>& triangles, unsigned int triangle_count) :
			triangles(triangles), triangle_count(triangle_count) {};

	void WriteOBJ(const char* path) {
		this->GenericWriteToDisk(
				[&path, this](Triangle* triangle_array) {
					FILE* f = fopen(path, "w+");
					if (f != nullptr) {
						for (uint i = 0; i < triangle_count; i++) {
							fprintf(f, "v %f %f %f\n", triangle_array[i].p0.x, triangle_array[i].p0.y,
							        triangle_array[i].p0.z);
							fprintf(f, "v %f %f %f\n", triangle_array[i].p1.x, triangle_array[i].p1.y,
							        triangle_array[i].p1.z);
							fprintf(f, "v %f %f %f\n", triangle_array[i].p2.x, triangle_array[i].p2.y,
							        triangle_array[i].p2.z);
						}

						for (uint i = 0; i < triangle_count; i++)
							fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
						fclose(f);
					}
				}
		);

	}

	void WriteSTL(const char* path) {
		this->GenericWriteToDisk(
				[&path, this](Triangle* triangle_array) {
					FILE* f = fopen(path, "wb+");

					if (f != nullptr) {
						for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

						fwrite(&triangle_count, sizeof(int), 1, f);

						float zero = 0.0f;
						short attribute = 0;
						for (uint i = 0; i < triangle_count; i++) {
							fwrite(&zero, sizeof(float), 1, f);
							fwrite(&zero, sizeof(float), 1, f);
							fwrite(&zero, sizeof(float), 1, f);

							fwrite(&triangle_array[i].p2.x, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p2.y, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p2.z, sizeof(float), 1, f);

							fwrite(&triangle_array[i].p1.x, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p1.y, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p1.z, sizeof(float), 1, f);

							fwrite(&triangle_array[i].p0.x, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p0.y, sizeof(float), 1, f);
							fwrite(&triangle_array[i].p0.z, sizeof(float), 1, f);

							fwrite(&attribute, sizeof(short), 1, f);

							//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
							//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
							//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
						}

						//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
						fclose(f);
					}
				}
		);
	}

private: // member functions
	template<typename TWriteTriangleArrayFunction>
	void GenericWriteToDisk(TWriteTriangleArrayFunction&& write_triangle_array) {
		ORUtils::MemoryBlock<Triangle>* cpu_triangles;
		bool temporary_memory_used = false;
		if (triangles.GetAccessMode() == MEMORYDEVICE_CUDA) {
			cpu_triangles = new ORUtils::MemoryBlock<Triangle>(triangles.size(), MEMORYDEVICE_CPU);
			cpu_triangles->SetFrom(triangles, MemoryCopyDirection::CUDA_TO_CPU);
			temporary_memory_used = true;
		} else {
			cpu_triangles = &triangles;
		}

		Triangle* triangle_array = cpu_triangles->GetData(MEMORYDEVICE_CPU);

		std::forward<TWriteTriangleArrayFunction>(write_triangle_array)(triangle_array);

		if (temporary_memory_used) delete cpu_triangles;
	}
};
}
