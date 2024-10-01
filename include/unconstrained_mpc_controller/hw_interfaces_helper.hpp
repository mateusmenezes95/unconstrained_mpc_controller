// Copyright (c) 2024 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <fmt/format.h>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "unconstrained_mpc_controller/types/types.hpp"

namespace unconstrained_mpc_controller
{

class HwInterfacesHelper
{
public:
  HwInterfacesHelper() = delete;
  ~HwInterfacesHelper() = delete;

  /**
   * @brief Get the index of the interface with the given name in the given vector.
   * 
   * @tparam T The type of the interface (e.g. CommandInterface, StateInterface).
   * @param name Name of the interface to search for.
   * @param interfaces Vector of interfaces to search in.
   * @return size_t The index of the interface with the given name.
   * @throws std::runtime_error If the interface with the given name is not found.
   */
  template <typename T>
  static size_t getInterfaceIndexByName(const std::string & name, const std::vector<T> & interfaces)
  {
    for (size_t i = 0; i < interfaces.size(); i++) {
      if (interfaces[i].get_name() == name) return i;
    }
    throw std::runtime_error(fmt::format("Interface with name '{}' not found.", name));
  }

  /**
   * @brief Get pair of indexes for each hw interface
   * 
   * @tparam T The type of the interface (e.g. CommandInterface, StateInterface).
   * @param interfaces Vector of interfaces to search in.
   * @param eigen_indexes Unordered map with the name of the interface as key and the index in the
   * Eigen vector as value.
   * @return std::unordered_map<std::string, types::EigenHwInterfaceIndexPair> Pair of indexes for
   * each hw interface.
   * @throws std::runtime_error If the interface with the given name is not found.
   */
  template <typename T>
  static std::unordered_map<std::string, types::EigenHwInterfaceIndexPair> getIndexesPairs(
    const std::vector<T> & interfaces,
    const std::unordered_map<std::string, size_t> & eigen_indexes)
  {
    std::unordered_map<std::string, types::EigenHwInterfaceIndexPair> index_map;
    for (const auto & [name, idx] : eigen_indexes) {
      index_map[name].eigen_vec_idx = idx;
      index_map[name].hw_iface_idx = getInterfaceIndexByName(name, interfaces);
    }

    return index_map;
  }
};

}  // namespace unconstrained_mpc_controller
