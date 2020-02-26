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
std::istream& operator>>(std::istream& str, CSVRow& data) {
	data.readNextRow(str);
	return str;
}
}
using namespace ITMLib;


std::string const& CSVRow::operator[](std::size_t index) const {
	return m_data[index];
}

std::size_t CSVRow::size() const {
	return m_data.size();
}

void CSVRow::readNextRow(std::istream& str) {
	std::string line;
	std::getline(str, line);

	std::stringstream lineStream(line);
	std::string cell;

	m_data.clear();
	while (std::getline(lineStream, cell, ',')) {
		m_data.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty()) {
		// If there was a trailing comma then add an empty element.
		m_data.push_back("");
	}
}

CSVIterator::CSVIterator(std::istream& str) : m_str(str.good() ? &str : nullptr) { ++(*this); }

CSVIterator::CSVIterator() : m_str(nullptr) {}
