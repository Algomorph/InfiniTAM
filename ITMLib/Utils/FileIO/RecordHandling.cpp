//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 8/31/20.
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
#include <filesystem>
#include <log4cplus/log4cplus.h>
#include <chrono>

//local
#include "RecordHandling.h"
#include "../Logging/LoggingUtilities.h"

namespace fs = std::filesystem;

namespace ITMLib {
template<typename TP>
std::time_t to_time_t(TP tp) {
	using namespace std::chrono;
	auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now()
	                                                    + system_clock::now());
	return system_clock::to_time_t(sctp);
}

void ArchivePossibleExistingRecords(const std::string& record_path, const std::string& record_archive_folder) {
	fs::path fs_record_path(record_path);
	if (fs::exists(fs_record_path)) {
		if (!fs::is_regular_file(fs_record_path)) {
			auto root = log4cplus::Logger::getRoot();
			LOG4CPLUS_FATAL(root, "Record file path," << fs_record_path
			                                          << ", is occupied by a non-file, i.e. directory or symlink! Aborting.");
			DIEWITHEXCEPTION_REPORTLOCATION(
					"Log file path occupied by a non-file, i.e. directory or symlink! Aborting. ");
		} else {
			auto write_time = fs::last_write_time(fs_record_path);
			auto printable_time = to_time_t(write_time);
			std::stringstream buffer;
			buffer << std::put_time(std::localtime(&printable_time), "%y%m%d%H%M%S");
			fs::path archive_directory = fs_record_path.parent_path() / fs::path(record_archive_folder);
			fs::create_directories(archive_directory);
			fs::path move_destination =
					archive_directory / (fs_record_path.stem().string() + buffer.str() + fs_record_path.extension().string());
			fs::rename(fs_record_path, move_destination);
		}
	}
}
} // namespace ITMLib