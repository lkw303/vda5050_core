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

// Variant of master_example.cpp that builds its orders from each AGV's own
// reported position. Once an AGV's state shows an initialised position, that
// pose becomes its first node (N0) and the second node (N1) is placed
// diagonally away by a configurable offset: (x + d, y + d). Orders then
// alternate N0 -> N1 / N1 -> N0. Drives N AGVs named <prefix>001 ... <prefix>N.
//
// Usage: master_diagonal_example [-n N] [--offset D] [--serial-prefix P]
//                                [--broker URI]

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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

std::string kBroker = "tcp://localhost:1883";
constexpr auto kInterface = "uagv";
constexpr auto kManufacturer = "Manufacturer";
constexpr auto kMapId = "map_1";

// Serials are <kSerialPrefix>001, <kSerialPrefix>002, ... (matches
// vda5050_amr_sim's S001..Snnn). Override with --serial-prefix.
std::string kSerialPrefix = "S";
int kNumRobots = 1;

// Distance in metres added to both x and y to place the second node.
// Override with --offset; negative values go the other way.
double kDiagonalOffset = 10.0;

// true  = drive the demo loop, assigning orders automatically.
// false = connect, onboard, and localize only; assign orders yourself.
constexpr bool kAutoDispatch = true;

// 0 = assign the next order on every completion indefinitely; set > 0 to stop
// after N orders (per AGV) instead. Ignored when kAutoDispatch is false.
constexpr int kMaxOrders = 0;

std::atomic_bool running{true};

void signal_handler([[maybe_unused]] int signal)
{
  running = false;
}

std::string serial_for(int index)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%03d", index + 1);
  return kSerialPrefix + buf;
}

// Everything the demo tracks per AGV, keyed by agv_id ("<mfr>/<serial>").
struct AgvContext
{
  std::string serial;
  bool init_sent = false;
  bool first_order_sent = false;
  int orders_sent = 0;
  // Captured from the AGV's first initialised state; N0 and N1 for all orders.
  std::optional<types::NodePosition> start_pos;
  std::optional<types::NodePosition> goal_pos;
};

// Wrap one instant action and hand it to the master; the header is auto-filled.
void send_action(
  const std::shared_ptr<VDA5050Master>& master, const std::string& serial,
  const types::Action& action)
{
  types::InstantActions ia;
  ia.actions = {action};
  master->assign_instant_actions(kManufacturer, serial, ia);
}

types::Node make_node(
  const std::string& id, uint32_t seq, const types::NodePosition& pos)
{
  types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = true;
  n.node_position = pos;
  return n;
}

// A minimal released route from -> to (header auto-filled by assign_order).
types::Order make_order(
  const std::string& order_id, const std::string& from,
  const types::NodePosition& from_pos, const std::string& to,
  const types::NodePosition& to_pos)
{
  types::Order o;
  o.order_id = order_id;
  o.order_update_id = 0;
  o.nodes = {make_node(from, 0, from_pos), make_node(to, 2, to_pos)};
  types::Edge e;
  e.edge_id = "E0";
  e.sequence_id = 1;
  e.start_node_id = from;
  e.end_node_id = to;
  e.released = true;
  o.edges = {e};
  return o;
}

}  // namespace

int main(int argc, char** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if ((arg == "-n" || arg == "--n-robots") && has_value)
    {
      kNumRobots = std::atoi(argv[++i]);
    }
    else if (arg == "--offset" && has_value)
    {
      kDiagonalOffset = std::strtod(argv[++i], nullptr);
    }
    else if (arg == "--serial-prefix" && has_value)
    {
      kSerialPrefix = argv[++i];
    }
    else if (arg == "--broker" && has_value)
    {
      kBroker = argv[++i];
    }
    else
    {
      std::fprintf(
        stderr,
        "Usage: %s [-n N] [--offset D] [--serial-prefix P] [--broker URI]\n",
        argv[0]);
      return 1;
    }
  }
  if (kNumRobots < 1)
  {
    std::fprintf(stderr, "n-robots must be >= 1\n");
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto mqtt = vda5050_core::transport::create_default_client_shared(
    kBroker, "master_diagonal_example");
  auto master = VDA5050Master::make(mqtt);

  // Callbacks fire on the MQTT thread(s); ctx_mutex guards every AgvContext.
  // The map is fully built before connect() and never resized afterwards.
  std::mutex ctx_mutex;
  std::map<std::string, AgvContext> agvs;
  for (int i = 0; i < kNumRobots; ++i)
  {
    const auto serial = serial_for(i);
    agvs[std::string(kManufacturer) + "/" + serial].serial = serial;
  }

  // Caller holds ctx_mutex. Alternate N0->N1 / N1->N0 so each order starts
  // where the AGV parked.
  auto next_order = [&](const std::string& agv_id, AgvContext& ctx) -> bool {
    if (!ctx.start_pos || !ctx.goal_pos)
    {
      return false;
    }
    const bool forward = ctx.orders_sent % 2 == 0;
    const std::string order_id =
      ctx.serial + "-order-" + std::to_string(ctx.orders_sent);
    auto res = master->assign_order(
      kManufacturer, ctx.serial,
      make_order(
        order_id, forward ? "N0" : "N1",
        forward ? *ctx.start_pos : *ctx.goal_pos, forward ? "N1" : "N0",
        forward ? *ctx.goal_pos : *ctx.start_pos));
    if (res)
    {
      ++ctx.orders_sent;
      VDA5050_INFO("Assigned new order [{}] to [{}]", order_id, agv_id);
      return true;
    }
    VDA5050_WARN(
      "Order [{}] not assigned to [{}] ({} error(s))", order_id, agv_id,
      res.errors.size());
    return false;
  };

  master->on_connect([&](const std::string& agv_id) {
    std::lock_guard<std::mutex> lock(ctx_mutex);
    auto it = agvs.find(agv_id);
    if (it == agvs.end())
    {
      return;
    }
    auto& ctx = it->second;
    // Re-run setup on every (re)connect in case the AGV restarted.
    ctx.init_sent = false;
    ctx.first_order_sent = false;
    ctx.start_pos.reset();
    ctx.goal_pos.reset();
    VDA5050_INFO("[{}] online; requesting factsheet + state", agv_id);
    send_action(
      master, ctx.serial,
      ActionFactory::build_factsheet_request(
        ActionFactory::generate_action_id()));
    send_action(
      master, ctx.serial,
      ActionFactory::build_state_request(ActionFactory::generate_action_id()));
  });

  master->on_factsheet([](const std::string& agv_id, const types::Factsheet&) {
    VDA5050_INFO("[{}] factsheet received", agv_id);
  });

  // Drive setup off State: localize first, then dispatch if the demo is on.
  master->on_state([&](const std::string& agv_id, const types::State& state) {
    std::lock_guard<std::mutex> lock(ctx_mutex);
    auto it = agvs.find(agv_id);
    if (it == agvs.end())
    {
      return;
    }
    auto& ctx = it->second;
    const bool initialized = state.agv_position.has_value() &&
                             state.agv_position->position_initialized;
    if (!initialized && !ctx.init_sent)
    {
      VDA5050_INFO("[{}] uninitialized; sending initPosition", agv_id);
      send_action(
        master, ctx.serial,
        ActionFactory::build_init_position(
          ActionFactory::generate_action_id(), 0.0, 0.0, 0.0, kMapId, "N0"));
      ctx.init_sent = true;
      return;
    }
    if (!kAutoDispatch)
    {
      return;
    }
    if (initialized && !ctx.start_pos)
    {
      // Use the AGV's reported pose as N0; N1 is diagonally away from it.
      const auto& p = *state.agv_position;
      types::NodePosition start;
      start.x = p.x;
      start.y = p.y;
      start.theta = p.theta;
      start.map_id = p.map_id;
      ctx.start_pos = start;

      types::NodePosition goal = start;
      goal.x += kDiagonalOffset;
      goal.y += kDiagonalOffset;
      ctx.goal_pos = goal;
      VDA5050_INFO(
        "[{}] N0 = ({:.2f}, {:.2f}) from AGV state; N1 = ({:.2f}, {:.2f}) "
        "(offset {:.2f})",
        agv_id, start.x, start.y, goal.x, goal.y, kDiagonalOffset);
    }
    if (initialized && !ctx.first_order_sent)
    {
      ctx.first_order_sent = next_order(agv_id, ctx);
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
      std::lock_guard<std::mutex> lock(ctx_mutex);
      auto it = agvs.find(agv_id);
      if (it == agvs.end())
      {
        return;
      }
      auto& ctx = it->second;
      if (kMaxOrders != 0 && ctx.orders_sent >= kMaxOrders)
      {
        VDA5050_INFO("[{}] demo complete after {} order(s)", agv_id,
                     ctx.orders_sent);
        return;
      }
      next_order(agv_id, ctx);
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
  for (const auto& [agv_id, ctx] : agvs)
  {
    master->onboard_agv(kInterface, kManufacturer, ctx.serial);
  }
  VDA5050_INFO(
    "Connected; waiting for {} AGV(s) [{}/{} ... {}/{}] ...", kNumRobots,
    kManufacturer, serial_for(0), kManufacturer, serial_for(kNumRobots - 1));

  while (running)
  {
    // With kAutoDispatch = false, assign orders here via master->assign_order.
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  master->disconnect();
  return 0;
}
