// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//stdlib
#include <string>
#include <fstream>

//local
#include "MemoryBlock.h"
#include "Image.h"
#include "OStreamWrapper.h"
#include "IStreamWrapper.h"

namespace ORUtils {

/**
 * \brief This class provides functions for loading and saving memory blocks.
 */
class MemoryBlockPersistence {
	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
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
	LoadMemoryBlock(TIStreamWrapper& file, ORUtils::MemoryBlock<T>& block, MemoryDeviceType memory_type) {
		size_t block_size = ReadBlockSize(file.IStream());
		if (memory_type == MEMORYDEVICE_CUDA) {
			// If we're loading into a block on the CUDA, first try and read the data into a temporary block on the CPU.
			ORUtils::MemoryBlock<T> cpu_block(block.size(), MEMORYDEVICE_CPU);
			ReadBlockData(file.IStream(), cpu_block, block_size);

			// Then copy the data across to the CUDA.
			block.SetFrom(cpu_block, MemoryCopyDirection::CPU_TO_CUDA);
		} else {
			// If we're loading into a block on the CPU, read the data directly into the block.
			ReadBlockData(file.IStream(), block, block_size);
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
	static ORUtils::MemoryBlock<T>*
	LoadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T>* dummy = nullptr) {
		size_t block_size = ReadBlockSize(filename);
		ORUtils::MemoryBlock<T>* block = new ORUtils::MemoryBlock<T>(block_size, MEMORYDEVICE_CPU);
		ReadBlockData(filename, *block, block_size);
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
	static void SaveMemoryBlock_Generic(TOstreamWrapper& file, const TBlockType& block,
	                                    MemoryDeviceType memory_type) {
		if (memory_type == MEMORYDEVICE_CUDA) {
			// If we are saving the memory block from the CUDA, first make a CPU copy of it.
			TBlockType block_CPU(block.size(), MEMORYDEVICE_CPU);
			block_CPU.SetFrom(block, MemoryCopyDirection::CUDA_TO_CPU);

			// Then write the CPU copy to disk.
			WriteBlock_Generic<TElement, TBlockType>(file.OStream(), block_CPU);
		} else {
			// If we are saving the memory block from the CPU, write it directly to disk.
			WriteBlock_Generic<TElement, TBlockType>(file.OStream(), block);
		}
	}

	/**
	 * \brief Saves a memory block to a file on disk.
	 *
	 * \param file          	Successfully-opened handle to the file
	 * \param block             The memory block to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 */
	template<typename T, typename TOStreamWrapper>
	static void SaveMemoryBlock(TOStreamWrapper& file, const ORUtils::MemoryBlock<T>& block,
	                            MemoryDeviceType memory_type) {
		SaveMemoryBlock_Generic<T, ORUtils::MemoryBlock<T>, TOStreamWrapper>(file, block, memory_type);
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
		SaveMemoryBlock_Generic<T, ORUtils::MemoryBlock<T>>(file, block, block.GetAccessMode());
	}

	/**
	 * \brief Saves image data to a file on disk.
	 *
	 * \param file          	Successfully-opened handle to the file
	 * \param image             The image whose data to save.
	 * \param memory_type       The type of memory device from which to save the data.
	 */
	template<typename T>
	static void SaveImageData(OStreamWrapper& file, const ORUtils::Image<T>& image) {
		SaveMemoryBlock_Generic<T, ORUtils::Image<T>>(file, image, image.GetAccessMode());
	}


	//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:


	/**
	 * \brief Attempts to read data into a memory block allocated on the CPU from an input stream.
	 *
	 * The memory block must have the specified size (which should have been obtained by a call to ReadBlockSize).
	 *
	 * \param is                  The input stream.
	 * \param block               The memory block into which to read.
	 * \param block_size           The required size for the memory block.
	 * \throws std::runtime_error If the read is unsuccessful.
	 */
	template<typename T>
	static void ReadBlockData(std::istream& is, ORUtils::MemoryBlock<T>& block, size_t block_size) {
		// Try and read the block's size.
		if (block.size() != block_size) {
			throw std::runtime_error("Could not read data into a memory block of the wrong size");
		}

		// Try and read the block's data.
		if (!is.read(reinterpret_cast<char*>(block.GetData(MEMORYDEVICE_CPU)), block_size * sizeof(T))) {
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
		ReadBlockData(fs, block, blockSize);
	}

	/**
	 * \brief Attempts to read the size of a memory block from an input stream.
	 *
	 * The size is stored as a single integer and precedes the data for the block.
	 *
	 * \param is                  The input stream.
	 * \return                    The size of the memory block.
	 * \throws std::runtime_error If the read is unsuccesssful.
	 */
	static size_t ReadBlockSize(std::istream& is) {
		size_t block_size;
		if (is.read(reinterpret_cast<char*>(&block_size), sizeof(size_t))) return block_size;
		else throw std::runtime_error("Could not read memory block size");
	}

	template<typename TElement, typename TBlockType>
	static void WriteBlock_Generic(std::ostream& os, const TBlockType& block) {
		typename ORUtils::MemoryBlock<TElement>::size_type size = block.size();
		// Try and write the block's size.
		if (!os.write(reinterpret_cast<const char*>(&size), sizeof(typename ORUtils::MemoryBlock<TElement>::size_type))) {
			throw std::runtime_error("Could not write memory block size");
		}

		// Try and write the block's data.
		if (!os.write(reinterpret_cast<const char*>(block.GetData(MEMORYDEVICE_CPU)), block.size() * sizeof(TElement))) {
			throw std::runtime_error("Could not write memory block data");
		}
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
	static void WriteBlock(std::ostream& os, const ORUtils::MemoryBlock<T>& block) {
		WriteBlock_Generic<T, ORUtils::MemoryBlock<T>>(os, block);
	}
	template<typename T>
	static void WriteImageData(std::ostream& os, const ORUtils::Image<T>& image) {
		WriteBlock_Generic<T, ORUtils::Image<T>>(os, image);
	}
};
}
