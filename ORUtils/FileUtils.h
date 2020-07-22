#pragma once

#include <stdio.h>

#include "Image.h"
#include "Vector.h"

namespace ORUtils{

void SaveImageToFile(const ORUtils::Image<ORUtils::Vector4<unsigned char> >& image, const char* file_name, bool flip_vertical = false);
void SaveImageToFile(const ORUtils::Image<short>& image, const char* file_name);
void SaveImageToFile(const ORUtils::Image<float>& image, const char* file_name);
bool ReadImageFromFile(ORUtils::Image<ORUtils::Vector4<unsigned char> >& image, const char* file_name);
bool ReadImageFromFile(ORUtils::Image<short>& image, const char *file_name);
bool ReadImageFromFile(ORUtils::Image<unsigned char>& image, const char *file_name);

template <typename T> void ReadFromBinary(T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "rb");
	fread(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

template <typename T> void WriteToBinary(const T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "wb");
	fwrite(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

} // namespace ORUtils