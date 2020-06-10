// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified code: Copyright 2020 Gregory Kramida

#pragma once

//stdlib
#include <string>
#include <fstream>
#include <sstream>

//local
#include "MemoryBlock.h"
#include "Image.h"
#include "OStreamWrapper.h"
#include "IStreamWrapper.h"
#include "TypeTraits.h"

namespace ORUtils {

/**
 * \brief This class provides functions for loading and saving memory blocks.
 */
class MemoryBlockPersistence {
public: // static functions
	/**
	 * \brief Loads data from a file on disk into a memory block.
	 *
	 * \param path            Path to the file.
	 * \param block           The memory block into which to load the data.
	 * \param memory_type     The type of memory device on which to load the data.
	 * \param use_compression Whether or not to use (zlib) compression
	 * (only useful when code is compiled with Boost Iostreams)
	 */
	template<typename T>
	static void
	LoadMemoryBlock(const std::string& path, ORUtils::MemoryBlock<T>& block, MemoryDeviceType memory_type,
	                bool use_compression = false) {
		IStreamWrapper file(path.c_str(), use_compression);
		if (!file) throw std::runtime_error("Could not open " + path + " for reading");
		LoadMemoryBlock(file, block, memory_type);
	}

	/**
	 * \brief Loads data from an input stream.
	 *
	 * \param file          	Successfully-opened handle to the file.
	 * \param block             The memory block into which to load the data.
	 * \param memory_type       The type of memory device on which to load the data.
	 */
	template<typename T, typename TIStreamWrapper>
	static void
	LoadMemoryBlock(TIStreamWrapper& file, ORUtils::MemoryBlock<T>& block, MemoryDeviceType memory_type = MEMORYDEVICE_CPU) {
		size_t block_size = ReadBlockSize(file.IStream());
		if (memory_type == MEMORYDEVICE_CUDA) {
			// If we're loading into a block on the CUDA, first try and read the data into a temporary block on the CPU.
			ORUtils::MemoryBlock<T> cpu_block(block.size(), MEMORYDEVICE_CPU);
			ReadArray<T,ORUtils::MemoryBlock<T>>(file.IStream(), cpu_block, block_size);

			// Then copy the data across to the CUDA.
			block.SetFrom(cpu_block, MemoryCopyDirection::CPU_TO_CUDA);
		} else {
			// If we're loading into a block on the CPU, read the data directly into the block.
			ReadArray<T,ORUtils::MemoryBlock<T>>(file.IStream(), block, block_size);
		}
	}

	/**
	 * \brief Loads data from an input stream.
	 *
	 * \param file          	Successfully-opened handle to the file.
	 * \param memory_type       The type of memory device on which to load the data.
	 */
	template<typename T>
	static ORUtils::MemoryBlock<T>
	LoadMemoryBlock(ORUtils::IStreamWrapper& file, MemoryDeviceType memory_type = MEMORYDEVICE_CPU) {
		size_t block_size = ReadBlockSize(file.IStream());
		ORUtils::MemoryBlock<T> block(block_size, memory_type);
		if (memory_type == MEMORYDEVICE_CUDA) {
			// If we're loading into a block on the CUDA, first try and read the data into a temporary block on the CPU.
			ORUtils::MemoryBlock<T> cpu_block(block.size(), MEMORYDEVICE_CPU);
			ReadArray<T,ORUtils::MemoryBlock<T>>(file.IStream(), cpu_block, block_size);

			// Then copy the data across to the CUDA.
			block.SetFrom(cpu_block, MemoryCopyDirection::CPU_TO_CUDA);
		} else {
			// If we're loading into a block on the CPU, read the data directly into the block.
			ReadArray<T,ORUtils::MemoryBlock<T>>(file.IStream(), block, block_size);
		}
		return block;
	}

	/**
	 * \brief Loads image from an input stream.
	 *
	 * \param file          	Successfully-opened handle to the file.
	 * \param memory_type       The type of memory device to which to load the data.
	 * \return 				    A new memory block with the input stream contents at the give stream position.
	 */
	template<typename T>
	static ORUtils::Image<T>
	LoadImage(ORUtils::IStreamWrapper& file, MemoryDeviceType memory_type = MEMORYDEVICE_CPU) {
		Vector2<int> dimensions = ReadImageDimensions(file.IStream());
		int channel_count = ReadImageChannelCount(file.IStream());
		if(TypeTraits<T>::element_count != channel_count){
			std::stringstream ss;
			ss << "Read-in channel count (" << channel_count << ") doesn't match expected channel count (" << TypeTraits<T>::element_count << ").";
			throw std::runtime_error(ss.str());
		}
		int pixel_count = dimensions.width * dimensions.height;
		ORUtils::Image<T> cpu_image(dimensions, MEMORYDEVICE_CPU);
		ReadArray<T,ORUtils::Image<T>>(file.IStream(), cpu_image, pixel_count);
		if (memory_type == MEMORYDEVICE_CUDA) {
			return ORUtils::Image<T>(cpu_image, MEMORYDEVICE_CUDA);
		} else {
			return cpu_image;
		}
	}

	/**
	 * \brief Loads data from a file on disk into a memory block newly-allocated on the CPU with the appropriate size.
	 * \tparam T	    Element type
	 * \param filename  The name of the file.
	 * \param dummy     An optional dummy parameter that can be used for type inference.
	 * \return          The loaded memory block.
	 */
	template<typename T>
	static ORUtils::MemoryBlock<T>
	LoadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T>* dummy = nullptr) {
		size_t block_size = ReadBlockSize(filename);
		ORUtils::MemoryBlock<T> block(block_size, MEMORYDEVICE_CPU);
		ReadBlockData(filename, block, block_size);
		return block;
	}

	/**
	 * \brief Attempts to read the size of a memory block from a file containing data for a single block.
	 *
	 * The size is stored as a single integer and precedes the data for the block.
	 *
	 * \param filename            The name of the file.
	 * \return                    The size of the memory block in the file.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	static size_t ReadBlockSize(const std::string& filename) {
		std::ifstream fs(filename.c_str(), std::ios::binary);
		if (!fs) throw std::runtime_error("Could not open " + filename + " for reading");
		return ReadBlockSize(fs);
	}

	/**
	 * \brief Saves a memory block to a file on disk.
	 *
	 * \param filename          The name of the file.
	 * \param block             The memory block to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 * \param use_compression   Whether or not to use compression (with ZLib).
	 * Has no effect (or compression) if the code is compiled without Boost Iostreams library.
	 */
	template<typename T>
	static void SaveMemoryBlock(const std::string& filename, const ORUtils::MemoryBlock<T>& block,
	                            MemoryDeviceType memory_type, bool use_compression = false) {
		OStreamWrapper file(filename.c_str(), use_compression);
		if (!file) throw std::runtime_error("Could not open " + filename + " for writing");
		SaveMemoryBlock(file, block, memory_type);
	}

	template<typename TElement, typename TBlockType, typename TOstreamWrapper>
	static void SaveArray(TOstreamWrapper& file, const TBlockType& block, MemoryDeviceType memory_type) {
		if (memory_type == MEMORYDEVICE_CUDA) {
			// If we are saving the memory block from the CUDA, first make a CPU copy of it.
			TBlockType block_CPU(block.size(), MEMORYDEVICE_CPU);
			block_CPU.SetFrom(block, MemoryCopyDirection::CUDA_TO_CPU);

			// Then write the CPU copy to disk.
			WriteArray<TElement>(file.OStream(), block_CPU);
		} else {
			// If we are saving the memory block from the CPU, write it directly to disk.
			WriteArray<TElement>(file.OStream(), block);
		}
	}


	/**
	 * \brief Saves a memory block to a file on disk.
	 *
	 * \param file          	Successfully-opened handle to the file
	 * \param block             The memory block to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 */
	template<typename T>
	static void SaveMemoryBlock(OStreamWrapper& file, const ORUtils::MemoryBlock<T>& block,
	                            MemoryDeviceType memory_type) {
		SaveArray<T, ORUtils::MemoryBlock<T>>(file, block, memory_type);
	}

	/**
	 * \brief Saves a memory block to a file on disk.
	 *
	 * \param file          	Successfully-opened handle to the file
	 * \param block             The memory block to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 */
	template<typename T>
	static void SaveMemoryBlock(OStreamWrapper& file, const ORUtils::MemoryBlock<T>& block) {
		SaveArray<T, ORUtils::MemoryBlock<T>>(file, block, block.GetAccessMode());
	}

	/**
	 * \brief Saves image data to a file on disk.
	 *
	 * \param file          	Successfully-opened handle to the file
	 * \param image             The image whose data to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 */
	template<typename T>
	static void SaveImage(OStreamWrapper& file, const ORUtils::Image<T>& image) {
		SaveArray<T, ORUtils::Image<T>>(file, image, image.GetAccessMode());
	}

private: // static functions
	/**
	 * \brief Attempts to read data into a memory block or image allocated on the CPU from an input stream.
	 *
	 * The underlying Array-type object needs to already have been allocated to the specified size (which should have been obtained by a call to ReadBlockSize or ReadImageDimensions).
	 *
	 * \param is                  The input stream.
	 * \param array               The array into which to read.
	 * \param block_size          The required size for the array.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	template<typename T, typename ArrayType>
	static void ReadArray(std::istream& is, ArrayType& array, size_t block_size) {
		if (array.size() != block_size) {
			std::stringstream ss;
			ss << "Could not read data of size(" << block_size << ") into a memory block of the wrong size (" << array.size() << ")." "\n[" __FILE__ ":" TOSTRING(__LINE__) "]";
			throw std::runtime_error(ss.str());
		}

		// Try and read the block's data.
		if (!is.read(reinterpret_cast<char*>(array.GetData(MEMORYDEVICE_CPU)), block_size * sizeof(T))) {
			throw std::runtime_error("Could not read memory block data");
		}
	}

	/**
	 * \brief Attempts to read data into a memory block allocated on the CPU from a file that contains data for a single block.
	 *
	 * The memory block must have the specified size (which should have been obtained by a call to ReadBlockSize).
	 *
	 * \param filename            The name of the file.
	 * \param block               The memory block into which to read.
	 * \param blockSize           The required size for the memory block.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	template<typename T>
	static void ReadBlockData(const std::string& filename, ORUtils::MemoryBlock<T>& block, size_t blockSize) {
		std::ifstream fs(filename.c_str(), std::ios::binary);
		if (!fs) throw std::runtime_error("Could not open " + filename + " for reading");

		// Try and skip the block's size.
		if (!fs.seekg(sizeof(size_t))) throw std::runtime_error("Could not skip memory block size");

		// Try and read the block's data.
		ReadArray<T,ORUtils::MemoryBlock<T>>(fs, block, blockSize);
	}

	/**
	 * \brief Attempts to read the size of a memory block from an input stream.
	 *
	 * The size is stored as a single 64-bit unsigned integer and precedes the data of the block.
	 *
	 * \param is                  The input stream.
	 * \return                    The size of the memory block.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	static size_t ReadBlockSize(std::istream& is) {
		size_t block_size;
		if (is.read(reinterpret_cast<char*>(&block_size), sizeof(size_t))) return block_size;
		else throw std::runtime_error("Could not read memory block size");
	}

	/**
	 * \brief Attempts to read the dimensions of an image from an input stream.
	 *
	 * The dimensions are stored as two 32-bit signed integers and precede the channel count and data of the image.
	 *
	 * \param is                  The input stream.
	 * \return                    The dimensions of the image.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	static Vector2<int> ReadImageDimensions(std::istream& is) {
		Vector2<int> dimensions;
		if (is.read(reinterpret_cast<char*>(&dimensions.x), sizeof(int)) && is.read(reinterpret_cast<char*>(&dimensions.y), sizeof(int)))
			return dimensions;
		else throw std::runtime_error("Could not read image dimensions.");
	}

	/**
	 * \brief Attempts to read the channel count of an image from an input stream.
	 *
	 * The channel count is stored as a 32-bit signed integer and precedes the data of the image.
	 *
	 * \param is                  The input stream.
	 * \return                    The channel count of the image
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	static int ReadImageChannelCount(std::istream& is) {
		int channel_count;
		if (is.read(reinterpret_cast<char*>(&channel_count), sizeof(int))) return channel_count;
		else throw std::runtime_error("Could not read image channel count.");
	}

	/**
	 * \brief Attempts to write a memory block allocated on the CPU to an output stream.
	 *
	 * A single integer containing the number of elements in the block is written prior to the block itself.
	 *
	 * \param os                  The output stream.
	 * \param block               The memory block to write.
	 * \throws std::runtime_error If the write is unsuccessful.
	 */
	template<typename T>
	static void WriteArray(std::ostream& os, const ORUtils::MemoryBlock<T>& block) {
		typename ORUtils::MemoryBlock<T>::size_type size = block.size();
		// Try to write the block's size.
		if (!os.write(reinterpret_cast<const char*>(&size), sizeof(typename ORUtils::MemoryBlock<T>::size_type))) {
			throw std::runtime_error("Could not write memory block size");
		}

		// Try to write the block's data.
		if (!os.write(reinterpret_cast<const char*>(block.GetData(MEMORYDEVICE_CPU)), block.size() * sizeof(T))) {
			throw std::runtime_error("Could not write memory block data");
		}
	}

	template<typename T>
	static void WriteArray(std::ostream& os, const ORUtils::Image<T>& image) {
		ORUtils::Vector2<int> dimensions = image.dimensions;
		// Try to write the image size.
		if (!os.write(reinterpret_cast<const char*>(&dimensions.x), sizeof(int))) {
			throw std::runtime_error("Could not write memory block size");
		}
		if (!os.write(reinterpret_cast<const char*>(&dimensions.y), sizeof(int))) {
			throw std::runtime_error("Could not write memory block size");
		}
		int channel_count = TypeTraits<T>::element_count;
		if (!os.write(reinterpret_cast<const char*>(&channel_count), sizeof(int))) {
			throw std::runtime_error("Could not write memory block size");
		}
		// Try to write the block's data.
		if (!os.write(reinterpret_cast<const char*>(image.GetData(MEMORYDEVICE_CPU)), image.size() * sizeof(T))) {
			throw std::runtime_error("Could not write memory block data");
		}
	}
};


template<typename T> class Vector2;
template<typename T> class Vector3;
template<typename T> class Vector4;
template<typename T> class Vector6;

extern template void MemoryBlockPersistence::SaveImage<bool>(OStreamWrapper& file, const ORUtils::Image<bool>& image);
extern template void MemoryBlockPersistence::SaveImage<char>(OStreamWrapper& file, const ORUtils::Image<char>& image);
extern template void MemoryBlockPersistence::SaveImage<unsigned char>(OStreamWrapper& file, const ORUtils::Image<unsigned char>& image);
extern template void MemoryBlockPersistence::SaveImage<short>(OStreamWrapper& file, const ORUtils::Image<short>& image);
extern template void MemoryBlockPersistence::SaveImage<unsigned short>(OStreamWrapper& file, const ORUtils::Image<unsigned short>& image);
extern template void MemoryBlockPersistence::SaveImage<int>(OStreamWrapper& file, const ORUtils::Image<int>& image);
extern template void MemoryBlockPersistence::SaveImage<unsigned int>(OStreamWrapper& file, const ORUtils::Image<unsigned int>& image);
extern template void MemoryBlockPersistence::SaveImage<long>(OStreamWrapper& file, const ORUtils::Image<long>& image);
extern template void MemoryBlockPersistence::SaveImage<unsigned long>(OStreamWrapper& file, const ORUtils::Image<unsigned long>& image);
extern template void MemoryBlockPersistence::SaveImage<long long>(OStreamWrapper& file, const ORUtils::Image<long long>& image);
extern template void MemoryBlockPersistence::SaveImage<unsigned long long>(OStreamWrapper& file, const ORUtils::Image<unsigned long long>& image);
extern template void MemoryBlockPersistence::SaveImage<float>(OStreamWrapper& file, const ORUtils::Image<float>& image);
extern template void MemoryBlockPersistence::SaveImage<double>(OStreamWrapper& file, const ORUtils::Image<double>& image);

extern template void MemoryBlockPersistence::SaveImage<Vector2<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector2<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector2<short>>(OStreamWrapper& file, const ORUtils::Image<Vector2<short>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector2<int>>(OStreamWrapper& file, const ORUtils::Image<Vector2<int>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector2<float>>(OStreamWrapper& file, const ORUtils::Image<Vector2<float>>& image);

extern template void MemoryBlockPersistence::SaveImage<Vector3<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector3<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector3<short>>(OStreamWrapper& file, const ORUtils::Image<Vector3<short>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector3<int>>(OStreamWrapper& file, const ORUtils::Image<Vector3<int>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector3<float>>(OStreamWrapper& file, const ORUtils::Image<Vector3<float>>& image);

extern template void MemoryBlockPersistence::SaveImage<Vector4<unsigned char>>(OStreamWrapper& file, const ORUtils::Image<Vector4<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector4<short>>(OStreamWrapper& file, const ORUtils::Image<Vector4<short>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector4<int>>(OStreamWrapper& file, const ORUtils::Image<Vector4<int>>& image);
extern template void MemoryBlockPersistence::SaveImage<Vector4<float>>(OStreamWrapper& file, const ORUtils::Image<Vector4<float>>& image);

extern template ORUtils::Image<bool> MemoryBlockPersistence::LoadImage<bool>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<char> MemoryBlockPersistence::LoadImage<char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<unsigned char> MemoryBlockPersistence::LoadImage<unsigned char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<short> MemoryBlockPersistence::LoadImage<short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<unsigned short> MemoryBlockPersistence::LoadImage<unsigned short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<int> MemoryBlockPersistence::LoadImage<int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<unsigned int> MemoryBlockPersistence::LoadImage<unsigned int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<long> MemoryBlockPersistence::LoadImage<long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<unsigned long> MemoryBlockPersistence::LoadImage<unsigned long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<long long> MemoryBlockPersistence::LoadImage<long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<unsigned long long> MemoryBlockPersistence::LoadImage<unsigned long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<float> MemoryBlockPersistence::LoadImage<float>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<double> MemoryBlockPersistence::LoadImage<double>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::Image<Vector2<unsigned char>> MemoryBlockPersistence::LoadImage<Vector2<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector2<short>> MemoryBlockPersistence::LoadImage<Vector2<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector2<int>> MemoryBlockPersistence::LoadImage<Vector2<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector2<float>> MemoryBlockPersistence::LoadImage<Vector2<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::Image<Vector3<unsigned char>> MemoryBlockPersistence::LoadImage<Vector3<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector3<short>> MemoryBlockPersistence::LoadImage<Vector3<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector3<int>> MemoryBlockPersistence::LoadImage<Vector3<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector3<float>> MemoryBlockPersistence::LoadImage<Vector3<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::Image<Vector4<unsigned char>> MemoryBlockPersistence::LoadImage<Vector4<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector4<short>> MemoryBlockPersistence::LoadImage<Vector4<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector4<int>> MemoryBlockPersistence::LoadImage<Vector4<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::Image<Vector4<float>> MemoryBlockPersistence::LoadImage<Vector4<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template void MemoryBlockPersistence::SaveMemoryBlock<bool>(OStreamWrapper& file, const ORUtils::MemoryBlock<bool>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<char>(OStreamWrapper& file, const ORUtils::MemoryBlock<char>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<unsigned char>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned char>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<short>(OStreamWrapper& file, const ORUtils::MemoryBlock<short>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<unsigned short>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned short>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<int>(OStreamWrapper& file, const ORUtils::MemoryBlock<int>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<unsigned int>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned int>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<long>(OStreamWrapper& file, const ORUtils::MemoryBlock<long>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<unsigned long>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned long>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<long long>(OStreamWrapper& file, const ORUtils::MemoryBlock<long long>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<unsigned long long>(OStreamWrapper& file, const ORUtils::MemoryBlock<unsigned long long>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<float>(OStreamWrapper& file, const ORUtils::MemoryBlock<float>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<double>(OStreamWrapper& file, const ORUtils::MemoryBlock<double>& image);

extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<short>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<int>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector2<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector2<float>>& image);

extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<short>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<int>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector3<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector3<float>>& image);

extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<unsigned char>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<unsigned char>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<short>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<short>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<int>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<int>>& image);
extern template void MemoryBlockPersistence::SaveMemoryBlock<Vector4<float>>(OStreamWrapper& file, const ORUtils::MemoryBlock<Vector4<float>>& image);

extern template ORUtils::MemoryBlock<bool> MemoryBlockPersistence::LoadMemoryBlock<bool>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<char> MemoryBlockPersistence::LoadMemoryBlock<char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<unsigned char> MemoryBlockPersistence::LoadMemoryBlock<unsigned char>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<short> MemoryBlockPersistence::LoadMemoryBlock<short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<unsigned short> MemoryBlockPersistence::LoadMemoryBlock<unsigned short>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<int> MemoryBlockPersistence::LoadMemoryBlock<int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<unsigned int> MemoryBlockPersistence::LoadMemoryBlock<unsigned int>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<long> MemoryBlockPersistence::LoadMemoryBlock<long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<unsigned long> MemoryBlockPersistence::LoadMemoryBlock<unsigned long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<long long> MemoryBlockPersistence::LoadMemoryBlock<long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<unsigned long long> MemoryBlockPersistence::LoadMemoryBlock<unsigned long long>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<float> MemoryBlockPersistence::LoadMemoryBlock<float>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<double> MemoryBlockPersistence::LoadMemoryBlock<double>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::MemoryBlock<Vector2<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector2<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector2<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector2<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector2<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::MemoryBlock<Vector3<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector3<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector3<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector3<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector3<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

extern template ORUtils::MemoryBlock<Vector4<unsigned char>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<unsigned char>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector4<short>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<short>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector4<int>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<int>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);
extern template ORUtils::MemoryBlock<Vector4<float>> MemoryBlockPersistence::LoadMemoryBlock<Vector4<float>>(IStreamWrapper& file,  MemoryDeviceType memory_device_type);

} // namespace ORUtils
