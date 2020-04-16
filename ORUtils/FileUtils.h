// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdio.h>

#include "Image.h"
#include "Vector.h"

void SaveImageToFile(const ORUtils::Image<ORUtils::Vector4<unsigned char> >& image, const char* file_name, bool flip_vertical = false);
void SaveImageToFile(const ORUtils::Image<short>& image, const char* file_name);
void SaveImageToFile(const ORUtils::Image<float>& image, const char* file_name);
bool ReadImageFromFile(ORUtils::Image<ORUtils::Vector4<unsigned char> >& image, const char* file_name);
bool ReadImageFromFile(ORUtils::Image<short>& image, const char *file_name);
bool ReadImageFromFile(ORUtils::Image<unsigned char>& image, const char *file_name);

void MakeDir(const char *directory_name);

template <typename T> void ReadFromBIN(T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "rb");
	fread(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

template <typename T> void WriteToBIN(const T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "wb");
	fwrite(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}
