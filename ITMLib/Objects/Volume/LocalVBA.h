// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <iostream>
#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"

namespace ITMLib
{
	/** \brief
	Stores the actual voxel content that is referred to by a
	ITMLib::ITMHashTable.
	*/
	template<class TVoxel>
	class LocalVBA
	{
	private:
		ORUtils::MemoryBlock<TVoxel> *voxelBlocks;
		ORUtils::MemoryBlock<int> *allocationList;

		MemoryDeviceType memoryType;

	public:
		inline TVoxel *GetVoxelBlocks() { return voxelBlocks->GetData(memoryType); }
		inline const TVoxel *GetVoxelBlocks() const { return voxelBlocks->GetData(memoryType); }
		int *GetAllocationList() { return allocationList->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
		const void* GetVoxelBlocks_MB() const { return voxelBlocks->GetMetalBuffer(); }
		const void* GetAllocationList_MB(void) const { return allocationList->GetMetalBuffer(); }
#endif
		int lastFreeBlockId;

		int allocatedSize;

		MemoryDeviceType GetMemoryType() const { return this->memoryType; }

		void SaveToDirectory(const std::string &outputDirectory) const
		{
			std::string VBFileName = outputDirectory + "voxel.dat";
			std::string ALFileName = outputDirectory + "alloc.dat";
			std::string AllocSizeFileName = outputDirectory + "vba.txt";

			ORUtils::MemoryBlockPersister::SaveMemoryBlock(VBFileName, *voxelBlocks, memoryType, true);
			ORUtils::MemoryBlockPersister::SaveMemoryBlock(ALFileName, *allocationList, memoryType, true);

			std::ofstream ofs(AllocSizeFileName.c_str());
			if (!ofs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for writing");

			ofs << lastFreeBlockId << ' ' << allocatedSize;
		}

		void LoadFromDirectory(const std::string &inputDirectory)
		{
			std::string VBFileName = inputDirectory + "voxel.dat";
			std::string ALFileName = inputDirectory + "alloc.dat";
			std::string AllocSizeFileName = inputDirectory + "vba.txt";

			ORUtils::MemoryBlockPersister::LoadMemoryBlock(VBFileName, *voxelBlocks, memoryType, true);
			ORUtils::MemoryBlockPersister::LoadMemoryBlock(ALFileName, *allocationList, memoryType, true);

			std::ifstream ifs(AllocSizeFileName.c_str());
			if (!ifs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for reading");

			ifs >> lastFreeBlockId >> allocatedSize;
		}

		void UpdateDeviceFromHost(){
			this->voxelBlocks->UpdateDeviceFromHost();
		}

		void UpdateHostFromDevice(){
			this->voxelBlocks->UpdateHostFromDevice();
		}


		LocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize);
		LocalVBA(const LocalVBA& other, MemoryDeviceType memoryType);
		void SetFrom(const LocalVBA& other);

		~LocalVBA();

		// Suppress the default copy constructor and assignment operator
		LocalVBA(const LocalVBA&) = delete;
		LocalVBA& operator=(const LocalVBA&) = delete;
	};
}
