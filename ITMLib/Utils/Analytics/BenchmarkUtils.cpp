//  ================================================================
//  Created by Gregory Kramida on 5/3/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

// stdlib
#include <map>
#include <chrono>
#include <iostream>

// Boost
#include <boost/filesystem/path.hpp>

// local
#include "../../../ORUtils/PlatformIndependence.h"
#include "BenchmarkUtils.h"
#include "../ITMPrintHelpers.h"
#include "../Configuration.h"

namespace fs = boost::filesystem;

namespace ITMLib {
namespace Bench {
std::map<std::string, std::pair<double, std::chrono::time_point<std::chrono::steady_clock>>> timers;

/**
 * \brief Starts the timer with the specified name (creates it if it doesn't yet exist)
 * \details Not thread-safe
 * \param name name of the timer
 */
void StartTimer(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		(*itr).second.second = std::chrono::steady_clock::now();
	} else {
		timers[name] = std::make_pair(0.0, std::chrono::steady_clock::now());
	}
}

/**
 * \brief Stops timer with the specified name
 * \details Not thread-safe
 * \param name name of the timer
 */
void StopTimer(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		double cumulativeTime = std::get<0>((*itr).second);
		auto start = std::get<1>((*itr).second);
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start;
		cumulativeTime += std::chrono::duration<double, std::milli>(diff).count();
		(*itr).second.first = cumulativeTime;
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

/**
 * \brief Print all cumulative times for timers recorded so far.
 * \details Not thread-safe
 */
void all_times_to_stream(std::ostream& out, bool colors_enabled) {
	if (colors_enabled) {
		out << green << "Logged cumulative runtimes:" << reset << std::endl;
	} else {
		out << "Logged cumulative runtimes:" << std::endl;
	}
	for (const auto& timer_pair : timers) {
		out << "  " << timer_pair.first << ": " << timer_pair.second.first << std::endl;
	}
}

void PrintAllCumulativeTimes() {
	all_times_to_stream(std::cout, true);
}

void SaveAllCumulativeTimesToDisk() {
	std::ofstream output_file;
	std::string path = (fs::path(configuration::get().paths.output_path) / "benchmark.txt").string();
	output_file.open(path);
	all_times_to_stream(output_file,false);
	output_file.close();
}


double StopTimerAndGetCumulativeTime(std::string name) {
	StopTimer(name);
	return GetCumulativeTime(name);
}

double StopTimerAndGetLastTime(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		double cumulativeTime = std::get<0>((*itr).second);
		auto start = std::get<1>((*itr).second);
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start;
		double lastTime = std::chrono::duration<double, std::milli>(diff).count();
		cumulativeTime += lastTime;
		(*itr).second.first = cumulativeTime;
		return lastTime;
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

double GetCumulativeTime(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		double cumulativeTime = std::get<0>((*itr).second);
		return cumulativeTime;
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

}//namespace Bench
}//namespace ITMLib