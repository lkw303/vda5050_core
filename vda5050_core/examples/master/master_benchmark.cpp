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

#include <sys/resource.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"


#include "vda5050_core/master/actions/action_factory.hpp"
#include "vda5050_core/master/master.hpp"

// Order-throughput benchmark master. Run adapter_example (N robots) first,
// then this with the same -n. It keeps every robot busy: it assigns a
// one-edge order and, the moment the robot reports it complete, assigns the
// next. Reports orders completed per second, in total and per robot.

using vda5050_core::master::ActionFactory;
using vda5050_core::master::VDA5050Master;
namespace types = vda5050_core::types;

using Clock = std::chrono::steady_clock;

namespace {

constexpr auto kInterface = "uagv";
constexpr auto kMapId = "map_1";

std::atomic_bool running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  running = false;
}

struct Options
{
  int num_robots = 10;
  std::string broker = "tcp://localhost:1883";
  std::string manufacturer = "Manufacturer";
  double duration = 10;         // measured seconds; 0 = until Ctrl-C
  double warmup_timeout = 30;   // max wait for every robot to get its first order
  double bench_period = 5;      // seconds between progress lines; 0 = off
  bool per_robot = true;        // print the per-robot table in the summary
};

constexpr const char * USAGE =
  "Usage: master_benchmark [-n N] [--broker URI] [--manufacturer M]\n"
  "                       [--duration S] [--warmup-timeout S]\n"
  "                       [--bench-period S] [--summary-only]\n"
  "  --duration 0       measure until Ctrl-C\n"
  "  --bench-period 0   disable progress lines\n"
  "  --summary-only     skip the per-robot table";

Options parse_args(int argc, char ** argv)
{
  Options o;
  if (const char * env = std::getenv("MQTT_BROKER")) {
    o.broker = env;
  }
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      std::puts(USAGE);
      std::exit(0);
    }
    if (a == "--summary-only") {
      o.per_robot = false;
      continue;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "missing value for %s\n%s\n", a.c_str(), USAGE);
      std::exit(2);
    }
    const std::string v = argv[++i];
    if (a == "-n" || a == "--n-robots") {
      o.num_robots = std::stoi(v);
    } else if (a == "--broker") {
      o.broker = v;
    } else if (a == "--manufacturer") {
      o.manufacturer = v;
    } else if (a == "--duration") {
      o.duration = std::stod(v);
    } else if (a == "--warmup-timeout") {
      o.warmup_timeout = std::stod(v);
    } else if (a == "--bench-period") {
      o.bench_period = std::stod(v);
    } else {
      std::fprintf(stderr, "unknown option %s\n%s\n", a.c_str(), USAGE);
      std::exit(2);
    }
  }
  return o;
}

double cpu_seconds()
{
  rusage u{};
  getrusage(RUSAGE_SELF, &u);
  return u.ru_utime.tv_sec + u.ru_stime.tv_sec +
         (u.ru_utime.tv_usec + u.ru_stime.tv_usec) * 1e-6;
}

double rss_mb()
{
  std::ifstream f("/proc/self/statm");
  long size = 0, resident = 0;
  f >> size >> resident;
  return resident * sysconf(_SC_PAGESIZE) / 1e6;
}

double seconds_between(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double>(b - a).count();
}

std::string serial_for(int index)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "S%03d", index + 1);
  return buf;
}

// The master reports AGVs as "<manufacturer>/<serial>"; "S007" -> index 6.
int index_of(const std::string & agv_id)
{
  return std::atoi(agv_id.c_str() + agv_id.find('/') + 2) - 1;
}

types::Node make_node(const std::string & id, uint32_t seq)
{
  types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = true;
  return n;
}

// A minimal released route from -> to (header auto-filled by assign_order).
types::Order make_order(
  const std::string & order_id, const std::string & from, const std::string & to)
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

// Per-robot benchmark state; each robot has its own lock so the master
// callbacks for different robots never serialize on one mutex.
struct Robot
{
  std::mutex mtx;
  bool init_sent = false;
  bool started = false;
  long orders_sent = 0;
  Clock::time_point sent_at;
  // Measured window only:
  long completed = 0;
  double latency_sum = 0;  // seconds, assign -> complete
  long window_completed = 0;  // for the progress lines
};

struct Stats
{
  double mean = 0, stddev = 0, min = 0, max = 0;
};

Stats stats_of(const std::vector<double> & v)
{
  Stats s;
  if (v.empty()) {
    return s;
  }
  double sum = 0;
  for (double x : v) {
    sum += x;
  }
  s.mean = sum / v.size();
  double var = 0;
  for (double x : v) {
    var += (x - s.mean) * (x - s.mean);
  }
  s.stddev = std::sqrt(var / v.size());
  s.min = *std::min_element(v.begin(), v.end());
  s.max = *std::max_element(v.begin(), v.end());
  return s;
}

}  // namespace

int main(int argc, char ** argv)
{
  vda5050_core::logger::set_log_level(vda5050_core::logger::LogLevel::DEBUG);
  const Options opt = parse_args(argc, argv);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const int n = opt.num_robots;

  // --- Master that keeps every robot busy -----------------------
  std::vector<std::unique_ptr<Robot>> robots;
  robots.reserve(n);
  for (int i = 0; i < n; i++) {
    robots.push_back(std::make_unique<Robot>());
  }
  std::atomic_bool measuring{false};
  std::atomic_bool stopping{false};
  std::atomic<int> started_count{0};
  std::atomic<long> rejected{0};

  auto master_mqtt = vda5050_core::transport::create_default_client_shared(
    opt.broker, "master_benchmark");
  auto master = VDA5050Master::make(master_mqtt);

  // Alternate N0->N1 / N1->N0 so each order starts where the robot parked.
  // Caller must hold the robot's mutex.
  auto next_order = [&](int idx, Robot & r) -> bool {
    const std::string serial = serial_for(idx);
    const bool forward = r.orders_sent % 2 == 0;
    r.sent_at = Clock::now();
    auto res = master->assign_order(
      opt.manufacturer, serial,
      make_order(
        "order-" + serial + "-" + std::to_string(r.orders_sent),
        forward ? "N0" : "N1", forward ? "N1" : "N0"));
    if (res) {
      ++r.orders_sent;
      return true;
    }
    ++rejected;
    return false;
  };

  master->on_connect([&](const std::string & agv_id) {
    const std::string serial = agv_id.substr(agv_id.find('/') + 1);
    types::InstantActions ia;
    ia.actions = {ActionFactory::build_state_request(ActionFactory::generate_action_id())};
    master->assign_instant_actions(opt.manufacturer, serial, ia);
  });

  // State drives setup: localize first, then hand out the first order.
  master->on_state([&](const std::string & agv_id, const types::State & state) {
    const int idx = index_of(agv_id);
    if (idx < 0 || idx >= n) {
      return;
    }
    const bool initialized =
      state.agv_position.has_value() && state.agv_position->position_initialized;
    Robot & r = *robots[idx];
    std::lock_guard<std::mutex> lock(r.mtx);
    if (!initialized && !r.init_sent) {
      types::InstantActions ia;
      ia.actions = {ActionFactory::build_init_position(
        ActionFactory::generate_action_id(), 0.0, 0.0, 0.0, kMapId, "N0")};
      master->assign_instant_actions(opt.manufacturer, serial_for(idx), ia);
      r.init_sent = true;
      return;
    }
    if (initialized && !r.started && !stopping) {
      r.started = next_order(idx, r);
      if (r.started) {
        ++started_count;
      }
    }
  });

  master->on_order_complete([&](const std::string & agv_id, const std::string &) {
    const int idx = index_of(agv_id);
    if (idx < 0 || idx >= n) {
      return;
    }
    Robot & r = *robots[idx];
    std::lock_guard<std::mutex> lock(r.mtx);
    if (measuring) {
      ++r.completed;
      ++r.window_completed;
      r.latency_sum += seconds_between(r.sent_at, Clock::now());
    }
    if (!stopping) {
      next_order(idx, r);
    }
  });

  // A rejected order never completes, so re-issue to keep the robot busy.
  master->on_order_rejected(
    [&](const std::string & agv_id, const std::string &, const std::vector<types::Error> &) {
      const int idx = index_of(agv_id);
      if (idx < 0 || idx >= n) {
        return;
      }
      ++rejected;
      Robot & r = *robots[idx];
      std::lock_guard<std::mutex> lock(r.mtx);
      if (!stopping) {
        next_order(idx, r);
      }
    });

  master->connect();
  for (int i = 0; i < n; ++i) {
    master->onboard_agv(kInterface, opt.manufacturer, serial_for(i));
  }

  // --- Warm-up: wait until every robot has been given its first order ------
  const auto warm_start = Clock::now();
  while (running && started_count < n &&
    seconds_between(warm_start, Clock::now()) < opt.warmup_timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const int started = started_count;
  if (started < n) {
    VDA5050_WARN("only {}/{} robots started within {:.0f}s", started, n, opt.warmup_timeout);
  }
  VDA5050_INFO(
    "[bench] {}/{} robots dispatching; measuring for {}", started, n,
    opt.duration > 0 ? fmt::format("{:g}s", opt.duration) : std::string("until Ctrl-C"));

  // --- Measurement window --------------------------------------------------
  const double cpu_run0 = cpu_seconds();
  const auto run_start = Clock::now();
  auto win_start = run_start;
  rejected = 0;  // report only rejections inside the window
  measuring = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const auto now = Clock::now();
    if (opt.duration > 0 && seconds_between(run_start, now) >= opt.duration) {
      break;
    }
    const double wall = seconds_between(win_start, now);
    if (opt.bench_period > 0 && wall >= opt.bench_period) {
      long done = 0;
      for (auto & r : robots) {
        std::lock_guard<std::mutex> lock(r->mtx);
        done += r->window_completed;
        r->window_completed = 0;
      }
      VDA5050_INFO(
        "[bench] t={:.0f}s orders/s={:.0f} ({} orders) rss={:.0f}MB",
        seconds_between(run_start, now), done / wall, done, rss_mb());
      win_start = now;
    }
  }
  measuring = false;
  const auto run_end = Clock::now();
  const double run = seconds_between(run_start, run_end);
  const double cpu_run = cpu_seconds() - cpu_run0;
  stopping = true;

  // --- Report --------------------------------------------------------------
  std::vector<double> rates;
  rates.reserve(n);
  long total = 0;
  double latency_sum = 0;
  if (opt.per_robot) {
    std::printf("\n%-8s %10s %12s %12s\n", "robot", "orders", "orders/s", "latency_ms");
  }
  for (int i = 0; i < n; i++) {
    Robot & r = *robots[i];
    std::lock_guard<std::mutex> lock(r.mtx);
    total += r.completed;
    latency_sum += r.latency_sum;
    rates.push_back(r.completed / run);
    if (opt.per_robot) {
      std::printf(
        "%-8s %10ld %12.2f %12.3f\n", serial_for(i).c_str(), r.completed,
        r.completed / run, r.completed ? 1e3 * r.latency_sum / r.completed : 0.0);
    }
  }
  const Stats s = stats_of(rates);
  VDA5050_INFO(
    "=== Performance === robots={} started={} run={:.2f}s orders={} orders/s={:.1f} "
    "per-robot orders/s: mean={:.2f} sd={:.2f} min={:.2f} max={:.2f} "
    "latency={:.3f}ms rejected={} cpu={:.1f}% rss={:.0f}MB",
    n, started, run, total, total / run, s.mean, s.stddev, s.min, s.max,
    total ? 1e3 * latency_sum / total : 0.0, rejected.load(),
    100.0 * cpu_run / run, rss_mb());

  // --- Shutdown ------------------------------------------------------------
  for (int i = 0; i < n; ++i) {
    VDA5050_INFO("Attempting to offboard AGV[{}]", serial_for(i));
    master->offboard_agv(opt.manufacturer, serial_for(i));
  }
  master->disconnect();
  return 0;
}
