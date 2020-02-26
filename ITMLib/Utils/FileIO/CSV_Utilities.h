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

#pragma once

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace ITMLib {
class CSVRow {
public:
	std::string const& operator[](std::size_t index) const;
	std::size_t size() const;
	void readNextRow(std::istream& str);
	friend std::istream& operator>>(std::istream& str, CSVRow& data);

private:
	std::vector<std::string> m_data;
};


class CSVIterator {
public:
	typedef std::input_iterator_tag iterator_category;
	typedef CSVRow value_type;
	typedef std::size_t difference_type;
	typedef CSVRow* pointer;
	typedef CSVRow& reference;

	explicit CSVIterator(std::istream& str);
	CSVIterator();

	// Pre Increment
	CSVIterator& operator++() {
		if (m_str) { if (!((*m_str) >> m_row)) { m_str = nullptr; }}
		return *this;
	}

	// Post increment
	CSVIterator operator++(int) {
		CSVIterator tmp(*this);
		++(*this);
		return tmp;
	}

	CSVRow const& operator*() const { return m_row; }

	CSVRow const* operator->() const { return &m_row; }

	bool operator==(CSVIterator const& rhs) {
		return ((this == &rhs) || ((this->m_str == nullptr) && (rhs.m_str == nullptr)));
	}

	bool operator!=(CSVIterator const& rhs) { return !((*this) == rhs); }

private:
	std::istream* m_str;
	CSVRow m_row;
};
} // namespace ITMLib