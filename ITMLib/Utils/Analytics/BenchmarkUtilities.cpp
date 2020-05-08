//  ================================================================
//  Created by Gregory Kramida on 5/3/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "BenchmarkUtilities.h"
#include "../CPPPrintHelpers.h"
#include "../Configuration/Configuration.h"
#include "../Logging/LoggingConfigruation.h"

namespace fs = boost::filesystem;

namespace ITMLib::benchmarking {
class Timer {
public:
	Timer() : cumulative_time(0.0), run_count(0u), start_point(std::chrono::steady_clock::now()) {}

	void start() {
		start_point = std::chrono::steady_clock::now();
	}

	double stop_and_get_last_time() {
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start_point;
		double last_time = std::chrono::duration<double, std::milli>(diff).count();
		cumulative_time += last_time;
		run_count++;
		return last_time;
	}

	void stop() {
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start_point;
		cumulative_time += std::chrono::duration<double, std::milli>(diff).count();
		run_count++;
	}

	unsigned int get_run_count() const { return run_count; }

	double get_cumulative_time() const;

	double get_mean_time() const { return cumulative_time / run_count; }

private:
	double cumulative_time;
	unsigned int run_count;
	std::chrono::time_point<std::chrono::steady_clock> start_point;

};

double Timer::get_cumulative_time() const { return cumulative_time; }

std::map<std::string, Timer> timers;

/**
 * \brief Starts the timer with the specified name (creates it if it doesn't yet exist)
 * \details Not thread-safe
 * \param name name of the timer
 */
void start_timer(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		(*itr).second.start();
	} else {
		timers[name] = Timer();
	}
}

/**
 * \brief Stops timer with the specified name
 * \details Not thread-safe
 * \param name name of the timer
 */
void stop_timer(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		(*itr).second.stop();
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

/**
 * \brief Print all cumulative times for timers recorded so far.
 * \details Not thread-safe
 */
void all_cumulative_times_to_stream(std::ostream& out, bool colors_enabled) {
	if (colors_enabled) {
		out << green << "Logged cumulative runtimes:" << reset << std::endl;
	} else {
		out << "Logged cumulative runtimes:" << std::endl;
	}
	for (const auto& timer_pair : timers) {
		out << "  " << timer_pair.first << ": " << timer_pair.second.get_cumulative_time() << std::endl;
	}
}

/**
 * \brief Print all average times for timers based on the cumulative times and run counts recorded so far.
 * \details Not thread-safe
 */
void all_average_times_to_stream(std::ostream& out, bool colors_enabled) {
	if (colors_enabled) {
		out << green << "Logged average runtimes:" << reset << std::endl;
	} else {
		out << "Logged average runtimes:" << std::endl;
	}
	for (const auto& timer_pair : timers) {
		out << "  " << timer_pair.first << ": "
		    << timer_pair.second.get_cumulative_time() / timer_pair.second.get_run_count() << std::endl;
	}
}

void print_all_cumulative_times_to_stdout() {
	all_cumulative_times_to_stream(std::cout, true);
}

void save_all_cumulative_times_to_disk() {
	std::ofstream output_file;
	std::string path = (fs::path(configuration::get().paths.output_path) / "benchmark.txt").string();
	output_file.open(path);
	all_cumulative_times_to_stream(output_file, false);
	output_file.close();
}

double stop_timer_and_get_cumulative_time(std::string name) {
	stop_timer(name);
	return get_cumulative_time(name);
}

double stop_timer_and_get_last_time(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		return (*itr).second.stop_and_get_last_time();
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

double get_cumulative_time(std::string name) {
	auto itr = timers.find(name);
	if (itr != timers.end()) {
		return (*itr).second.get_cumulative_time();
	} else {
		std::cerr << "Timer name: " << name << std::endl;
		DIEWITHEXCEPTION_REPORTLOCATION("Timer with this name not found.");
	}
}

void log_all_timers() {
	std::stringstream out;
	all_cumulative_times_to_stream(out, true);
	all_average_times_to_stream(out, true);
	LOG4CPLUS_TOP_LEVEL(logging::get_logger(), out.str());
}

}//namespace ITMLib::bench