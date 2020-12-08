//  ================================================================
//  Created by nitronoid (https://stackoverflow.com/users/7878485/nitronoid?tab=profile)
//  Copyright (c) 2020 nitronoid
//  Source: https://stackoverflow.com/questions/38955940/how-to-concatenate-static-strings-at-compile-time/62823211#62823211
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

#include <array>
#include <iostream>
#include <string_view>

namespace ORUtils{


template <std::string_view const&... Strs>
struct join
{
	// Helper to get a string literal from a std::array
	template <std::size_t N, std::array<char, N> const& S, typename>
	struct to_char_array;
	template <std::size_t N, std::array<char, N> const& S, std::size_t... I>
	struct to_char_array<N, S, std::index_sequence<I...>>
	{
		static constexpr const char value[]{S[I]..., 0};
	};
	// Join all strings into a single std::array of chars
	static constexpr auto impl() noexcept
	{
		constexpr std::size_t len = (Strs.size() + ... + 0);
		std::array<char, len + 1> arr{};
		auto append = [i = 0, &arr](auto const& s) mutable {
			for (auto c : s) arr[i++] = c;
		};
		(append(Strs), ...);
		arr[len] = 0;
		return arr;
	}
	// Give the joined string static storage
	static constexpr auto arr = impl();
	// Convert to a string literal, then view as a std::string_view
	static constexpr std::string_view value =
			to_char_array<arr.size(), arr, std::make_index_sequence<arr.size()>>::value;
};
// Helper to get the value out
template <std::string_view const&... Strs>
static constexpr auto join_v = join<Strs...>::value;
} // namespace ORUtils