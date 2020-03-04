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
class CSV_Row {
public:
	CSV_Row() = default;
	explicit CSV_Row(std::vector<int> input);
	explicit CSV_Row(std::vector<unsigned int> input);
	explicit CSV_Row(std::vector<float> input);
	explicit CSV_Row(std::vector<double> input);
	std::string const& operator[](std::size_t index) const;
	std::size_t size() const;
	void read_next_row(std::istream& str);
	void write_next_row(std::ostream& str);
	friend std::istream& operator>>(std::istream& str, CSV_Row& data);
	friend std::ostream& operator<<(std::ostream& str, CSV_Row& data);
	template<typename T>
	void operator<<(T item);
private:
	template<typename T>
	static std::vector<std::string> to_string_std_vector(std::vector<T> vector);
	std::vector<std::string> data;
};


class CSV_Iterator {
public:
	typedef std::input_iterator_tag iterator_category;
	typedef CSV_Row value_type;
	typedef std::size_t difference_type;
	typedef CSV_Row* pointer;
	typedef CSV_Row& reference;

	explicit CSV_Iterator(std::istream& str);
	CSV_Iterator();

	// Pre Increment
	CSV_Iterator& operator++() {
		if (m_str) { if (!((*m_str) >> m_row)) { m_str = nullptr; }}
		return *this;
	}

	// Post increment
	CSV_Iterator operator++(int) {
		CSV_Iterator tmp(*this);
		++(*this);
		return tmp;
	}

	CSV_Row const& operator*() const { return m_row; }

	CSV_Row const* operator->() const { return &m_row; }

	bool operator==(CSV_Iterator const& rhs) {
		return ((this == &rhs) || ((this->m_str == nullptr) && (rhs.m_str == nullptr)));
	}

	bool operator!=(CSV_Iterator const& rhs) { return !((*this) == rhs); }

private:
	std::istream* m_str;
	CSV_Row m_row;
};
} // namespace ITMLib