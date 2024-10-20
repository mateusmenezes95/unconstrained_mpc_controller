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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace unconstrained_mpc_controller
{

template <typename T>
class RealtimePublisherWrapper
{
public:
  RealtimePublisherWrapper() = default;
  ~RealtimePublisherWrapper() = default;

  void create(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & topic_name)
  {
    this->node_ = node;
    this->topic_name_ = topic_name;
    pub_ptr_ = this->node_->create_publisher<T>(this->topic_name_, rclcpp::SystemDefaultsQoS());
    rt_pub_ptr_ = std::make_shared<realtime_tools::RealtimePublisher<T>>(pub_ptr_);
  }

  T & getMsg()
  {
    return msg_;
  }

  void publish()
  {
    if (node_ == nullptr) {
      throw std::runtime_error(
        "Node not initialized in Realtime Publisher Wrapper object. "
        "Did you called the 'create' method?");
    }

    if (rt_pub_ptr_->trylock()) {
      rt_pub_ptr_->msg_ = this->msg_;
      rt_pub_ptr_->unlockAndPublish();
    } else {
      RCLCPP_WARN(this->node_->get_logger(),
        "'%s' unable to publish in the topic '%s'",
        this->node_->get_name(),
        topic_name_.c_str());
    }
  }

private:
  T msg_;
  std::string topic_name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<T>::SharedPtr pub_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<T>> rt_pub_ptr_;
};

}  // namespace unconstrained_mpc_controller
