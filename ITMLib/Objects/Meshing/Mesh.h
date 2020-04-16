// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Volume/VoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	class Mesh
	{
	public:
		struct Triangle { Vector3f p0, p1, p2; };

		MemoryDeviceType memory_type;

		uint triangle_count;
		uint max_triangle_count;

		ORUtils::MemoryBlock<Triangle> triangles;

		explicit Mesh(MemoryDeviceType memory_type, uint max_triangles) :
				memory_type(memory_type),
				triangle_count(0),
				max_triangle_count(max_triangles),
				triangles(max_triangle_count, memory_type) {}

		void WriteOBJ(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memory_type == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(max_triangle_count, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, MemoryCopyDirection::CUDA_TO_CPU);
				shoulDelete = true;
			} else {
				cpu_triangles = &triangles;
			}

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "w+");
			if (f != nullptr)
			{
				for (uint i = 0; i < triangle_count; i++)
				{
					fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				for (uint i = 0; i < triangle_count; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}

		void WriteSTL(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool deletion_needed = false;
			if (memory_type == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(max_triangle_count, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, MemoryCopyDirection::CUDA_TO_CPU);
				deletion_needed = true;
			} else {
				cpu_triangles = &triangles;
			}

			Triangle* triangle_array = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "wb+");

			if (f != nullptr) {
				for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

				fwrite(&triangle_count, sizeof(int), 1, f);

				float zero = 0.0f; short attribute = 0;
				for (uint i = 0; i < triangle_count; i++)
				{
					fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

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

			if (deletion_needed) delete cpu_triangles;
		}

		// Suppress the default copy constructor and assignment operator
		Mesh(const Mesh&);
		Mesh& operator=(const Mesh&);
	};
}
