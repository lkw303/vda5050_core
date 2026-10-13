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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <vda5050_core/logger/logger.hpp>

#include "vda5050_core/master/actions/action_factory.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using vda5050_core::master::ActionFactory;
using vda5050_core::master::VDA5050Master;
namespace types = vda5050_core::types;

namespace {

constexpr auto kBroker = "tcp://localhost:1883";
constexpr auto kInterface = "uagv";
constexpr auto kManufacturer = "Manufacturer";
constexpr auto kMapId = "map_1";

// true  = drive the demo loop, assigning orders automatically.
// false = connect, onboard, and localize only; assign orders yourself.
constexpr bool kAutoDispatch = true;

// 0 = assign the next order on every completion indefinitely; set > 0 to stop
// after N orders per AGV instead. Ignored when kAutoDispatch is false.
constexpr int kMaxOrders = 0;

std::atomic_bool running{true};

void signal_handler([[maybe_unused]] int signal)
{
  running = false;
}

// Setup/dispatch progress tracked independently for each AGV.
struct AgvProgress
{
  bool init_sent = false;
  bool first_order_sent = false;
  int orders_sent = 0;
};

// Serials are S001, S002, ... (S001 matches master_example).
std::string serial_for(int index)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "S%03d", index + 1);
  return buf;
}

// The master reports AGVs as "<manufacturer>/<serial>"; keep only the serial.
std::string serial_of(const std::string& agv_id)
{
  return agv_id.substr(agv_id.find('/') + 1);
}

// Wrap one instant action and hand it to the master; the header is auto-filled.
void send_action(
  const std::shared_ptr<VDA5050Master>& master, const std::string& serial,
  const types::Action& action)
{
  types::InstantActions ia;
  ia.actions = {action};
  master->assign_instant_actions(kManufacturer, serial, ia);
}

types::Node make_node(const std::string& id, uint32_t seq)
{
  types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = true;
  return n;
}

// A minimal released route from -> to (header auto-filled by assign_order).
types::Order make_order(
  const std::string& order_id, const std::string& from, const std::string& to)
{
  types::Order o;
  o.order_id = order_id;
  o.order_update_id = 0;
  o.nodes = {make_node(from, 0), make_node(to, 2)};
  types::Edge e;
  e.edge_id = "E0";
  e.sequence_id = 1;
  e.start_node_id = from;
  e.end_node_id = to;
  e.released = true;
  o.edges = {e};
  return o;
}

void print_usage(const char* prog)
{
  std::fprintf(
    stderr, "Usage: %s <n_robots>\n       %s --n-robots <n_robots>\n", prog,
    prog);
}

// Accepts "<n>" or "--n-robots <n>"; returns -1 on invalid input.
int parse_n_robots(int argc, char** argv)
{
  const char* value = nullptr;
  if (argc == 2)
  {
    value = argv[1];
  }
  else if (argc == 3 && std::strcmp(argv[1], "--n-robots") == 0)
  {
    value = argv[2];
  }
  if (value == nullptr)
  {
    return -1;
  }
  char* end = nullptr;
  const long n = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || n < 1 || n > 999)
  {
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace

int main(int argc, char** argv)
{
  const int n_robots = parse_n_robots(argc, argv);
  if (n_robots < 0)
  {
    print_usage(argv[0]);
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto mqtt = vda5050_core::transport::create_default_client_shared(
    kBroker, "master_multi_example");
  auto master = VDA5050Master::make(mqtt);

  // Callbacks may arrive for any AGV, so per-AGV state is guarded by a mutex.
  // Do not touch these, or next_order(), from main.
  std::mutex mtx;
  std::map<std::string, AgvProgress> progress;

  // Alternate N0->N1 / N1->N0 so each order starts where the AGV parked.
  // Caller must hold mtx.
  auto next_order = [&](const std::string& agv_id) -> bool {
    const std::string serial = serial_of(agv_id);
    auto& p = progress[agv_id];
    const bool forward = p.orders_sent % 2 == 0;
    const std::string order_id =
      "order-" + serial + "-" + std::to_string(p.orders_sent);
    auto res = master->assign_order(
      kManufacturer, serial,
      make_order(order_id, forward ? "N0" : "N1", forward ? "N1" : "N0"));
    if (res)
    {
      ++p.orders_sent;
      VDA5050_INFO("Assigned new order [{}] to [{}]", order_id, agv_id);
      return true;
    }
    VDA5050_WARN(
      "Order [{}] not assigned to [{}] ({} error(s))", order_id, agv_id,
      res.errors.size());
    return false;
  };

  master->on_connect([&](const std::string& agv_id) {
    {
      // Re-run setup on every (re)connect in case the AGV restarted.
      std::lock_guard<std::mutex> lock(mtx);
      progress[agv_id] = AgvProgress{};
    }
    VDA5050_INFO("[{}] online; requesting factsheet + state", agv_id);
    const std::string serial = serial_of(agv_id);
    send_action(
      master, serial,
      ActionFactory::build_factsheet_request(
        ActionFactory::generate_action_id()));
    send_action(
      master, serial,
      ActionFactory::build_state_request(ActionFactory::generate_action_id()));
  });

  master->on_factsheet([](const std::string& agv_id, const types::Factsheet&) {
    VDA5050_INFO("[{}] factsheet received", agv_id);
  });

  // Drive setup off State: localize first, then dispatch if the demo is on.
  master->on_state([&](const std::string& agv_id, const types::State& state) {
    const bool initialized = state.agv_position.has_value() &&
                             state.agv_position->position_initialized;
    std::lock_guard<std::mutex> lock(mtx);
    auto& p = progress[agv_id];
    if (!initialized && !p.init_sent)
    {
      VDA5050_INFO("[{}] uninitialized; sending initPosition", agv_id);
      send_action(
        master, serial_of(agv_id),
        ActionFactory::build_init_position(
          ActionFactory::generate_action_id(), 0.0, 0.0, 0.0, kMapId, "N0"));
      p.init_sent = true;
      return;
    }
    if (!kAutoDispatch)
    {
      return;
    }
    if (initialized && !p.first_order_sent)
    {
      p.first_order_sent = next_order(agv_id);
    }
  });

  master->on_node_reached(
    [](const std::string& agv_id, const std::string& node_id) {
      VDA5050_INFO("[{}] reached node [{}]", agv_id, node_id);
    });

  // Completion drives the demo loop; a no-op when kAutoDispatch is false.
  master->on_order_complete(
    [&](const std::string& agv_id, const std::string& order_id) {
      VDA5050_INFO("[{}] completed order [{}]", agv_id, order_id);
      if (!kAutoDispatch)
      {
        return;
      }
      std::lock_guard<std::mutex> lock(mtx);
      const int sent = progress[agv_id].orders_sent;
      if (kMaxOrders != 0 && sent >= kMaxOrders)
      {
        VDA5050_INFO("[{}] demo complete after {} order(s)", agv_id, sent);
        return;
      }
      next_order(agv_id);
    });

  // An order discarded before it reached the AGV never completes, so a
  // scheduler driven only by on_order_complete would stall here.
  master->on_order_rejected([](
                              const std::string& agv_id,
                              const std::string& order_id,
                              const std::vector<types::Error>& errors) {
    VDA5050_WARN(
      "[{}] order [{}] rejected ({} error(s))", agv_id, order_id,
      errors.size());
  });

  master->on_offline([](const std::string& agv_id) {
    VDA5050_WARN("[{}] went offline", agv_id);
  });
  master->on_connection_broken([](const std::string& agv_id) {
    VDA5050_WARN("[{}] connection lost", agv_id);
  });

  master->connect();
  for (int i = 0; i < n_robots; ++i)
  {
    master->onboard_agv(kInterface, kManufacturer, serial_for(i));
  }
  VDA5050_INFO(
    "Connected; waiting for {} AGV(s) [{}/{} .. {}/{}] ...", n_robots,
    kManufacturer, serial_for(0), kManufacturer, serial_for(n_robots - 1));

  while (running)
  {
    // With kAutoDispatch = false, assign orders here via master->assign_order.
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  master->disconnect();
  return 0;
}
