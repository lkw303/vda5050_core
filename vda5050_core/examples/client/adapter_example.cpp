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
#include <iostream>
#include <memory>
#include <optional>
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

std::atomic_bool running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  running = false;
}

// Settings are read from the command line arguments in main(), falling back to
// the defaults.
std::string BROKER_URI = "tcp://localhost:1883";
std::string MQTT_CLIENT_ID = "adapter_example";
std::string MANUFACTURER = "Manufacturer";
int NUM_ROBOTS = 200;
double DRIVE_TIME_S = 2.0;
double ACTION_TIME_S = 1.0;
double SLEEP_TIME_S = 0.2;

void print_usage(const char * program)
{
  std::cerr << "Usage: " << program << " [options]\n"
            << "  --broker <uri>          MQTT broker URI (default: "
            << BROKER_URI << ")\n"
            << "  --client-id <id>        MQTT client ID prefix (default: "
            << MQTT_CLIENT_ID << ")\n"
            << "  --manufacturer <name>   VDA5050 manufacturer (default: "
            << MANUFACTURER << ")\n"
            << "  --num-robots <n>        Number of robots (default: "
            << NUM_ROBOTS << ")\n"
            << "  --drive-time <s>        Drive time in seconds (default: "
            << DRIVE_TIME_S << ")\n"
            << "  --action-time <s>       Action time in seconds (default: "
            << ACTION_TIME_S << ")\n"
            << "  --sleep-time <s>        Main loop sleep in seconds (default: "
            << SLEEP_TIME_S << ")\n"
            << "  -h, --help              Show this message" << std::endl;
}

// Accepts both "--name value" and "--name=value".
bool parse_args(int argc, char ** argv)
{
  try {
    for (int i = 1; i < argc; i++) {
      std::string name = argv[i];
      std::string value;
      bool has_value = false;

      const auto eq = name.find('=');
      if (name.rfind("--", 0) == 0 && eq != std::string::npos) {
        value = name.substr(eq + 1);
        name = name.substr(0, eq);
        has_value = true;
      }

      if (name == "-h" || name == "--help") {
        return false;
      }

      if (!has_value) {
        if (i + 1 >= argc) {
          std::cerr << "Missing value for " << name << std::endl;
          return false;
        }
        value = argv[++i];
      }

      if (name == "--broker") {
        BROKER_URI = value;
      } else if (name == "--client-id") {
        MQTT_CLIENT_ID = value;
      } else if (name == "--manufacturer") {
        MANUFACTURER = value;
      } else if (name == "--n-robots") {
        NUM_ROBOTS = std::stoi(value);
      } else if (name == "--drive-time") {
        DRIVE_TIME_S = std::stod(value);
      } else if (name == "--action-time") {
        ACTION_TIME_S = std::stod(value);
      } else if (name == "--sleep-time") {
        SLEEP_TIME_S = std::stod(value);
      } else {
        std::cerr << "Unknown argument: " << name << std::endl;
        return false;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Invalid argument: " << e.what() << std::endl;
    return false;
  }
  return true;
}

std::string serial_for(int index)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "S%03d", index + 1);
  return buf;
}

std::shared_ptr<Adapter> make_numbered_adapter(int num)
{
  auto mqtt_client = vda5050_core::transport::create_default_client_unique(
    BROKER_URI, MQTT_CLIENT_ID + "_" + std::to_string(num));

  auto protocol_adapter = ProtocolAdapter::make(
    std::move(mqtt_client), "uagv", "2.0.0", MANUFACTURER, serial_for(num));

  auto adapter = Adapter::make(protocol_adapter);

  auto state_manager = adapter->state_manager();

  adapter->on_navigate([state_manager, num](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    VDA5050_INFO_STREAM(
      "Navigating to node [" << node_request.node_id() << "]");

    std::thread([node_request, execution, state_manager, num]() {
      state_manager->set_driving(true);

      std::this_thread::sleep_for(std::chrono::duration<double>(DRIVE_TIME_S));

      state_manager->set_driving(false);

      execution->finished();

      VDA5050_INFO_STREAM("AMR[" << std::to_string(num) << "]: Reached node [" << node_request.node_id() << "]");

      auto position = node_request.node_position();
      if (position.has_value())
      {
        state_manager->set_position(
          position.value().x, position.value().y,
          position.value().theta.value_or(0.0), position.value().map_id);
      }
    }).detach();
  });

  adapter->on_action(
    [state_manager](
      ActionRequest request, std::shared_ptr<ActionExecution> execution) {
      VDA5050_INFO_STREAM(
        "Action of type [" << request.action_type() << "] requested");

      execution->running();

      std::this_thread::sleep_for(std::chrono::duration<double>(ACTION_TIME_S));

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

void make_adapters(std::vector<std::shared_ptr<Adapter>> & adapters, int num)
{
  for (int i = 0; i < num; i++) {
    adapters.push_back(make_numbered_adapter(i));
  }
}

int main(int argc, char ** argv)
{
  if (!parse_args(argc, argv)) {
    print_usage(argv[0]);
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  const int num_robots = NUM_ROBOTS;
  std::vector<std::shared_ptr<Adapter>> adapters;
  adapters.reserve(num_robots);

  make_adapters(adapters, num_robots);
  int i = 0;
  for (auto & a : adapters) {

    VDA5050_INFO_STREAM("Adapter [" << std::to_string(i) << "]: started ...");
    std::thread([a]() {
      a->start();
    }).detach();
    i++;
  }

  while (running)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(SLEEP_TIME_S));
  }

  for (auto & a: adapters)
  {
    std::thread([a]() {
      a->stop();
    }).detach();
  }

  return 0;
}
