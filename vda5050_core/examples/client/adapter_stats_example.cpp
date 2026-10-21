/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/client/adapter/action_execution.hpp"
#include "vda5050_core/client/adapter/action_request.hpp"
#include "vda5050_core/client/adapter/adapter.hpp"
#include "vda5050_core/client/adapter/edge_request.hpp"
#include "vda5050_core/client/adapter/localization_request.hpp"
#include "vda5050_core/client/adapter/node_request.hpp"
#include "vda5050_core/client/adapter/order_execution.hpp"

using vda5050_core::client::adapter::ActionExecution;
using vda5050_core::client::adapter::ActionRequest;
using vda5050_core::client::adapter::Adapter;
using vda5050_core::client::adapter::EdgeRequest;
using vda5050_core::client::adapter::LocalizationRequest;
using vda5050_core::client::adapter::NodeRequest;
using vda5050_core::client::adapter::OrderExecution;
using vda5050_core::execution::ProtocolAdapter;

using Clock = std::chrono::steady_clock;

std::atomic_bool running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  running = false;
}

std::string serial_for(int index)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "S%03d", index + 1);
  return buf;
}

/// Counts completed orders per robot and overall.
///
/// The adapter API has no end-of-order callback, so an order counts as
/// completed when the last node navigated to for that order_id finishes and no
/// further node for the same order arrives within the grace period. The clock
/// starts when the first order is received.
class OrderStats
{
public:
  explicit OrderStats(
    std::chrono::milliseconds grace = std::chrono::milliseconds(500),
    double bucket_s = 10.0)
  : grace_(grace), bucket_s_(bucket_s)
  {
  }

  void node_started(int robot, const std::string & order_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      started_ = true;
      start_ = Clock::now();
      VDA5050_INFO_STREAM(
        "First order [" << order_id << "] received; stats clock started");
    }
    auto it = pending_.find(robot);
    if (it != pending_.end()) {
      if (it->second.order_id != order_id) {
        commit(robot);
      } else {
        pending_.erase(it);
      }
    }
  }

  void node_finished(int robot, const std::string & order_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_[robot] = {order_id, Clock::now()};
  }

  void sweep(bool flush = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    std::vector<int> due;
    for (const auto & [robot, p] : pending_) {
      if (flush || now - p.time >= grace_) {
        due.push_back(robot);
      }
    }
    for (int robot : due) {
      commit(robot);
    }
  }

  void report()
  {
    sweep(true);
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t total = completions_.size();
    if (!started_ || total == 0) {
      VDA5050_INFO_STREAM("No orders were completed");
      return;
    }
    VDA5050_INFO_STREAM("==== Orders completed per robot ====");
    for (const auto & [robot, n] : counts_) {
      VDA5050_INFO_STREAM("  " << serial_for(robot) << ": " << n);
    }
    VDA5050_INFO_STREAM("  TOTAL: " << total);

    const double elapsed =
      *std::max_element(completions_.begin(), completions_.end());
    VDA5050_INFO_STREAM(
      "==== Orders completed over time (" << bucket_s_ << "s buckets) ====");
    const int nbuckets = static_cast<int>(elapsed / bucket_s_) + 1;
    size_t cumulative = 0;
    for (int b = 0; b < nbuckets; b++) {
      const double lo = b * bucket_s_;
      const double hi = (b + 1) * bucket_s_;
      const auto n = std::count_if(
        completions_.begin(), completions_.end(),
        [lo, hi](double c) {return c >= lo && c < hi;});
      cumulative += n;
      VDA5050_INFO_STREAM(
        "  " << lo << "-" << hi << "s: +" << n << " cumulative " << cumulative);
    }
    VDA5050_INFO_STREAM(
      "Total " << total << " orders in " << elapsed
               << "s since first order (" << (elapsed > 0 ? total / elapsed : 0.0)
               << " orders/s)");
  }

private:
  struct Pending
  {
    std::string order_id;
    Clock::time_point time;
  };

  // Caller must hold mutex_.
  void commit(int robot)
  {
    const Pending p = pending_[robot];
    pending_.erase(robot);
    const size_t robot_total = ++counts_[robot];
    completions_.push_back(
      std::chrono::duration<double>(p.time - start_).count());
    VDA5050_INFO_STREAM(
      "Robot [" << serial_for(robot) << "] completed order [" << p.order_id
                << "] (robot total " << robot_total << ", overall "
                << completions_.size() << ")");
  }

  std::mutex mutex_;
  std::chrono::milliseconds grace_;
  double bucket_s_;
  bool started_ = false;
  Clock::time_point start_;
  std::map<int, Pending> pending_;
  std::map<int, size_t> counts_;
  std::vector<double> completions_;  // seconds since start
};

std::shared_ptr<Adapter> make_numbered_adapter(
  int num, std::shared_ptr<OrderStats> stats)
{
  auto mqtt_client = vda5050_core::transport::create_default_client_unique(
    "tcp://localhost:1883", "adapter_stats_example_" + std::to_string(num));

  auto protocol_adapter = ProtocolAdapter::make(
    std::move(mqtt_client), "uagv", "2.0.0", "Manufacturer", serial_for(num));

  auto adapter = Adapter::make(protocol_adapter);

  auto state_manager = adapter->state_manager();

  adapter->on_navigate([state_manager, stats, num](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    VDA5050_INFO_STREAM(
      "Navigating to node [" << node_request.node_id() << "]");

    const std::string order_id = execution->order_id();
    stats->node_started(num, order_id);

    std::thread([node_request, execution, state_manager, stats, num,
                 order_id]() {
      state_manager->set_driving(true);

      // std::this_thread::sleep_for(std::chrono::seconds(2));

      state_manager->set_driving(false);

      execution->finished();
      stats->node_finished(num, order_id);

      VDA5050_INFO_STREAM(
        "AMR[" << num << "]: Reached node [" << node_request.node_id() << "]");

      auto position = node_request.node_position();
      if (position.has_value()) {
        state_manager->set_position(
          position.value().x, position.value().y,
          position.value().theta.value_or(0.0), position.value().map_id);
      }
    }).detach();
  });

  adapter->on_action(
    [](ActionRequest request, std::shared_ptr<ActionExecution> execution) {
      VDA5050_INFO_STREAM(
        "Action of type [" << request.action_type() << "] requested");

      execution->running();

      // std::this_thread::sleep_for(std::chrono::seconds(1));

      execution->finished();
    });

  adapter->on_localize(
    [state_manager](
      LocalizationRequest request, std::shared_ptr<ActionExecution> execution) {
      VDA5050_INFO(
        "Received localization request with point [{}, {}, {}] on map: {}",
        request.x(), request.y(), request.theta(), request.map_id());

      state_manager->initialize_position(
        request.x(), request.y(), request.theta(), request.map_id());

      execution->finished();
    });
  return adapter;
}

int main()
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  int num_robots = 200;
  if (const char * env = std::getenv("NUM_ROBOTS")) {
    num_robots = std::atoi(env);
  }

  auto stats = std::make_shared<OrderStats>();
  std::vector<std::shared_ptr<Adapter>> adapters;
  adapters.reserve(num_robots);

  for (int i = 0; i < num_robots; i++) {
    adapters.push_back(make_numbered_adapter(i, stats));
  }

  int i = 0;
  for (auto & a : adapters) {
    VDA5050_INFO_STREAM("Adapter [" << i << "]: started ...");
    std::thread([a]() {a->start();}).detach();
    i++;
  }

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    stats->sweep();
  }

  for (auto & a : adapters) {
    std::thread([a]() {a->stop();}).detach();
  }

  stats->report();

  return 0;
}
