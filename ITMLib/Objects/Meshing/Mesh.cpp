//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/28/20.
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
//stdlib
#include <cstdlib>
#include <algorithm>
//TODO: uncomment when officially-supported compilers supply C++17 parallel execution policies for sorting
//#include <execution>

//tinyply.h
#include <tinyply/tinyply.h>

//local
#include "Mesh.h"
#include "../../Utils/Analytics/AlmostEqual.h"
#include "../../Utils/Collections/MemoryBlock_StdContainer_Convertions.h"
#include "../../../ORUtils/MemoryBlockPersistence/MemoryBlockPersistence.h"
#include "../../Utils/Collections/OperationsOnSTLContainers.h"

using namespace ITMLib;

Mesh::Mesh() : triangle_count(0) {}

Mesh::Mesh(const Mesh& other, MemoryDeviceType memory_type)
		: triangle_count(other.triangle_count),
		  triangles(other.triangles.size(), memory_type) {
	this->triangle_count = other.triangle_count;
	MemoryCopyDirection memory_copy_direction = DetermineMemoryCopyDirection(memory_type,
	                                                                         other.triangles.GetAccessMode());
	this->triangles.SetFrom(other.triangles, memory_copy_direction);
}

Mesh::Mesh(ORUtils::MemoryBlock<Triangle>& triangles, unsigned int triangle_count)
		: triangles(triangles), triangle_count(triangle_count) {}

MemoryDeviceType Mesh::GetMemoryDeviceType() const {
	return this->triangles.GetAccessMode();
}

void Mesh::WriteOBJ(const std::string& path) {
	this->GenericWriteToDisk(
			[&path, this](Triangle* triangle_array) {
				FILE* f = fopen(path.c_str(), "w+");
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

void Mesh::WriteSTL(const std::string& path) {
	this->GenericWriteToDisk(
			[&path, this](Triangle* triangle_array) {
				FILE* f = fopen(path.c_str(), "wb+");

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

void Mesh::WritePLY(const std::string& path, bool ascii, bool use_compression) {
	using namespace tinyply;
	this->GenericWriteToDisk(
			[&path, ascii, use_compression, this](Triangle* triangle_array) {
				ORUtils::OStreamWrapper file(path, use_compression);
				if (!file) return;
				const unsigned int vertex_count = triangle_count * 3;
				PlyFile mesh_file;
				mesh_file.add_properties_to_element(
						"vertex", {"x", "y", "z"},
						Type::FLOAT32, vertex_count, reinterpret_cast<uint8_t*>(triangle_array),
						Type::INVALID, 0
				);
				std::vector<uint32_t> triangle_indices = std::arange(0u, static_cast<uint32_t>(vertex_count), 1u);
				mesh_file.add_properties_to_element(
						"face", {"vertex_indices"}, Type::UINT32, triangle_count,
						reinterpret_cast<uint8_t*>(triangle_indices.data()), Type::UINT8, 3
				);
				mesh_file.write(file.OStream(), !ascii);
			}
	);
}

template<typename TWriteTriangleArrayFunction>
void Mesh::GenericWriteToDisk(TWriteTriangleArrayFunction&& write_triangle_array) {
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

namespace ITMLib {
_CPU_AND_GPU_CODE_
bool operator==(const Mesh::Triangle& triangle1, const Mesh::Triangle& triangle2) {
	return triangle1.p0 == triangle2.p0 &&
	       triangle1.p1 == triangle2.p1 &&
	       triangle1.p2 == triangle2.p2;
}

_CPU_AND_GPU_CODE_
bool operator!=(const Mesh::Triangle& triangle1, const Mesh::Triangle& triangle2) {
	return !(triangle1 == triangle2);
}

_CPU_AND_GPU_CODE_
bool AlmostEqual(const Mesh::Triangle& triangle1, const Mesh::Triangle& triangle2, const float tolerance) {
	return AlmostEqual(triangle1.p0, triangle2.p0, tolerance) &&
	       AlmostEqual(triangle1.p1, triangle2.p1, tolerance) &&
	       AlmostEqual(triangle1.p2, triangle2.p2, tolerance);
}

bool AlmostEqual(const Mesh& mesh1, const Mesh& mesh2, const float tolerance, bool presort_triangles) {
	if (mesh1.triangle_count != mesh2.triangle_count) return false;
	const int triangle_count = mesh1.triangle_count;

	std::vector<Mesh::Triangle> triangles1 = ORUtils_MemoryBlock_to_std_vector(mesh1.triangles,
	                                                                           mesh1.GetMemoryDeviceType(),
	                                                                           triangle_count);
	std::vector<Mesh::Triangle> triangles2 = ORUtils_MemoryBlock_to_std_vector(mesh2.triangles,
	                                                                           mesh2.GetMemoryDeviceType(),
	                                                                           triangle_count);

	if (presort_triangles) {
		//TODO: uncomment when officially-supported compilers supply C++17 parallel execution policies for sorting
		std::sort(/*std::execution::parallel_unsequenced_policy,*/ triangles1.begin(), triangles1.end());
		std::sort(/*std::execution::parallel_unsequenced_policy,*/ triangles2.begin(), triangles2.end());
	}

	volatile bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(triangles1, triangles2, mismatch_found) \
firstprivate(triangle_count, tolerance)
#endif
	for (int i_triangle = 0; i_triangle < triangle_count; i_triangle++) {
		if (mismatch_found) continue;
		if (!AlmostEqual(triangles1[i_triangle], triangles2[i_triangle], tolerance)) {
			mismatch_found = true;
		}
	}

	return !mismatch_found;
}

} // namespace ITMLib