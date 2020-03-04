//  ================================================================
//  Created by Gregory Kramida on 2/9/20.
//  Copyright (c)  2020 Gregory Kramida
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

//local
#include "CSV_Utilities.h"

namespace ITMLib {
std::istream& operator>>(std::istream& str, CSV_Row& data) {
	data.read_next_row(str);
	return str;
}

std::ostream& operator<<(std::ostream& str, CSV_Row& data) {
	data.write_next_row(str);
	return str;
}
}
using namespace ITMLib;


std::string const& CSV_Row::operator[](std::size_t index) const {
	return data[index];
}

std::size_t CSV_Row::size() const {
	return data.size();
}

void CSV_Row::read_next_row(std::istream& str) {
	std::string line;
	std::getline(str, line);

	std::stringstream line_stream(line);
	std::string cell;

	data.clear();
	while (std::getline(line_stream, cell, ',')) {
		data.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!line_stream && cell.empty()) {
		// If there was a trailing comma then add an empty element.
		data.push_back("");
	}
}

void CSV_Row::write_next_row(std::ostream& str) {
	int i = 0;
	for (auto item : data) {
		str << item;
		if (i != data.size() - 1) {
			str << ", ";
		}
		i++;
	}
}

CSV_Row::CSV_Row(std::vector<int> input) : data(to_string_std_vector<int>(input)) {}

CSV_Row::CSV_Row(std::vector<unsigned int> input) : data(to_string_std_vector<unsigned int>(input)) {}

CSV_Row::CSV_Row(std::vector<float> input) : data(to_string_std_vector<float>(input)) {}

CSV_Row::CSV_Row(std::vector<double> input) : data(to_string_std_vector<double>(input)) {}

template<typename T>
std::vector<std::string> CSV_Row::to_string_std_vector(std::vector<T> vector) {
	std::vector<std::string> out;
	for(auto& item : vector){
		out.push_back(std::to_string(item));
	}
	return out;
}

template<typename T>
void CSV_Row::operator<<(T item) {
	this->data.push_back(std::to_string(item));
}


CSV_Iterator::CSV_Iterator(std::istream& str) : m_str(str.good() ? &str : nullptr) { ++(*this); }

CSV_Iterator::CSV_Iterator() : m_str(nullptr) {}
