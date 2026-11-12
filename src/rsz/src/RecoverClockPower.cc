// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "RepairSetupProfiler.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "rsz/Resizer.hh"
#include "sta/ArcDelayCalc.hh"
#include "sta/Clock.hh"
#include "sta/EquivCells.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/PortDirection.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {
// Early and late paths share an envelope. Opposite source/sink transitions and
// different scenes never share one. Root arrival subtraction removes both the
// source edge phase and source latency (including generated-clock insertion).
using ClockKey = std::tuple<const sta::Clock*, const sta::Scene*, int, int>;
using ElectricalKey
    = std::tuple<const sta::Pin*, const sta::Scene*, int, int, bool>;
using SlackKey = std::pair<const sta::Scene*, int>;
struct Envelope
{
  float early = std::numeric_limits<float>::infinity();
  float late = -std::numeric_limits<float>::infinity();
  std::set<const sta::Pin*> sinks;
};
struct SinkArrival
{
  const sta::Pin* pin;
  ClockKey key;
  size_t min_max;
  float latency;
};
struct Snapshot
{
  std::map<ClockKey, Envelope> clocks;
  std::set<const sta::Pin*> unclocked_sinks;
  std::map<ElectricalKey, float> electrical;
  std::map<SlackKey, float> slacks;
  float power = std::numeric_limits<float>::quiet_NaN();
  bool timing_rejected = false;
  bool electrical_rejected = false;
  bool power_computed = false;
  std::vector<SinkArrival> sink_arrivals;
};
// Small relative tolerances cover floating-point STA recomputation only.
bool below(float actual, float floor)
{
  return actual < floor - std::max(1e-15f, std::abs(floor) * 1e-5f);
}
bool contains(const odb::Rect& outer, const odb::Rect& inner)
{
  return outer.xMin() <= inner.xMin() && outer.yMin() <= inner.yMin()
         && outer.xMax() >= inner.xMax() && outer.yMax() >= inner.yMax();
}
}  // namespace

int Resizer::recoverClockPower(const sta::LibertyCellSeq& requested,
                               float max_skew,
                               float max_latency,
                               float setup_margin,
                               float hold_margin,
                               int max_passes,
                               bool top_down,
                               bool verbose,
                               int max_trials,
                               bool batch,
                               int max_checkpoints,
                               double max_seconds,
                               float max_timing_regression,
                               float max_intermediate_timing_regression,
                               bool setup_margin_given,
                               bool hold_margin_given)
{
  constexpr int kInvalidLimits = 3304;
  const auto start = std::chrono::steady_clock::now();
  using ProfileDuration = std::chrono::duration<double, std::milli>;
  ProfileDuration refresh_time{0};
  ProfileDuration initial_prepare_time{0};
  ProfileDuration snapshot_prepare_time{0};
  ProfileDuration traversal_time{0};
  ProfileDuration slack_time{0};
  ProfileDuration power_time{0};
  size_t refresh_calls = 0;
  size_t initial_prepare_calls = 0;
  size_t snapshot_prepare_calls = 0;
  size_t traversal_calls = 0;
  size_t slack_queries = 0;
  size_t power_calls = 0;
  size_t skipped_power_trials = 0;
  size_t skipped_electrical_power_trials = 0;
  auto report_profile = [&]() {
    if (verbose) {
      logger_->report(
          "recover_clock_power profile: updateParasiticsAndTiming {} calls, "
          "{:.3f} ms; pre-snapshot findRequireds+ensureClkArrivals {} calls, "
          "{:.3f} ms; snapshot findRequireds+ensureClkArrivals {} calls, "
          "{:.3f} ms; graph clock/electrical traversal {} calls, {:.3f} ms; "
          "worstSlack {} queries, {:.3f} ms; whole-design power {} calls, "
          "{:.3f} ms; clock/slack rejected trials skipped power {}.",
          refresh_calls,
          refresh_time.count(),
          initial_prepare_calls,
          initial_prepare_time.count(),
          snapshot_prepare_calls,
          snapshot_prepare_time.count(),
          traversal_calls,
          traversal_time.count(),
          slack_queries,
          slack_time.count(),
          power_calls,
          power_time.count(),
          skipped_power_trials);
    }
  };
  auto refresh_timing = [&]() {
    ++refresh_calls;
    const auto phase_start = verbose ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
    try {
      updateParasiticsAndTiming();
    } catch (...) {
      if (verbose) {
        refresh_time += std::chrono::steady_clock::now() - phase_start;
      }
      throw;
    }
    if (verbose) {
      refresh_time += std::chrono::steady_clock::now() - phase_start;
    }
  };
  auto measure_power = [&]() {
    ++power_calls;
    const auto phase_start = verbose ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
    sta::PowerResult total, seq, comb, clock, macro, pad;
    sta_->power(sta_->cmdScene(), total, seq, comb, clock, macro, pad);
    if (verbose) {
      power_time += std::chrono::steady_clock::now() - phase_start;
    }
    return clock.total();
  };
  if ((batch
       && (max_checkpoints < 2 || !std::isfinite(max_seconds)
           || max_seconds <= 0))
      || max_passes <= 0 || max_trials < 0 || !std::isfinite(max_skew)
      || !std::isfinite(max_latency) || (max_skew < 0 && max_skew != -1)
      || (max_latency < 0 && max_latency != -1) || !std::isfinite(setup_margin)
      || setup_margin < 0 || !std::isfinite(hold_margin) || hold_margin < 0
      || !std::isfinite(max_timing_regression)
      || (max_timing_regression < 0 && max_timing_regression != -1)
      || !std::isfinite(max_intermediate_timing_regression)
      || (max_intermediate_timing_regression < 0
          && max_intermediate_timing_regression != -1)
      || (!batch
          && (max_timing_regression != -1
              || max_intermediate_timing_regression != -1))) {
    logger_->error(
        utl::RSZ, kInvalidLimits, "Invalid recover_clock_power limits.");
  }
  // sceneCell() falls back to the link cell when its scene map is empty.
  // Require an actual named model owned by this scene's libraries as well as
  // the expected pin interface and non-inverting function.
  auto scene_model = [](sta::LibertyCell* cell,
                        sta::Scene* scene,
                        const sta::MinMax* mm) -> sta::LibertyCell* {
    auto* mapped = cell->sceneCell(scene, mm);
    for (auto* library : scene->libertyLibraries(mm)) {
      auto* named = library->findLibertyCell(cell->name());
      if (named && mapped == named && named->isBuffer()
          && sta::equivCellPorts(cell, named)
          && sta::equivCellFuncs(cell, named)) {
        return named;
      }
    }
    return nullptr;
  };
  auto complete_models = [&](sta::LibertyCell* cell) {
    for (auto* scene : sta_->scenes()) {
      for (auto* mm : sta::MinMax::range()) {
        if (!scene_model(cell, scene, mm)) {
          return false;
        }
      }
    }
    return true;
  };
  // An explicit corner-library object identifies a physical master, not the
  // model to use in every scene. Resolve its actual link cell before checking
  // dont_use or model coverage.
  sta::LibertyCellSeq candidates;
  for (auto* requested_cell : requested) {
    auto* master = requested_cell
                       ? db_->findMaster(requested_cell->name().c_str())
                       : nullptr;
    auto* cell = master ? db_network_->libertyCell(db_network_->dbToSta(master))
                        : nullptr;
    if (!cell || !cell->isBuffer() || !master->isCore()) {
      logger_->error(
          utl::RSZ,
          3305,
          "recover_clock_power requires available core buffer masters.");
    }
    if (!complete_models(cell)) {
      logger_->error(
          utl::RSZ,
          3311,
          "Buffer {} lacks a compatible named model in a configured scene.",
          cell->name());
    }
    candidates.push_back(cell);
  }
  initBlock();
  init();
  for (auto* mode : sta_->modes()) {
    sta_->ensureClkNetwork(mode);
  }
  // Collect actual propagated clock buffers, regardless of CTS names/net type.
  struct Buffer
  {
    odb::dbInst* inst;
    size_t depth;
  };
  std::vector<Buffer> buffers;
  for (auto* inst : block_->getInsts()) {
    auto* si = db_network_->dbToSta(inst);
    auto* cell = network_->libertyCell(si);
    if (!cell || !cell->isBuffer() || !inst->getMaster()->isCore()
        || !inst->isPlaced() || isFixed(si) || dontTouch(si)) {
      continue;
    }
    bool protected_net = false;
    bool clock = false;
    size_t depth = 0;
    std::unique_ptr<sta::InstancePinIterator> pins(network_->pinIterator(si));
    while (pins->hasNext()) {
      auto* pin = pins->next();
      auto* net = network_->net(pin);
      protected_net |= net && dontTouch(net);
      if (!network_->direction(pin)->isAnyOutput()) {
        continue;
      }
      for (auto* mode : sta_->modes()) {
        if (sta_->isClock(pin, mode)) {
          if (!sta_->isPropagatedClock(pin, mode)
              || sta_->isIdealClock(pin, mode)) {
            logger_->error(
                utl::RSZ,
                3306,
                "recover_clock_power requires propagated clocks ({}).",
                network_->pathName(pin));
          }
          clock = true;
        }
      }
    }
    if (clock && !protected_net) {
      if (!complete_models(cell)) {
        logger_->error(utl::RSZ,
                       3315,
                       "Original clock buffer {} ({}) lacks a compatible named "
                       "model in a configured scene.",
                       inst->getName(),
                       cell->name());
      }
      buffers.push_back({inst, depth});
    }
  }
  // Model coverage is established before any timing evaluation or power
  // measurement of the eligible starting clock tree.
  ++initial_prepare_calls;
  auto phase_start = verbose ? std::chrono::steady_clock::now()
                             : std::chrono::steady_clock::time_point{};
  sta_->findRequireds();
  sta_->ensureClkArrivals();
  if (verbose) {
    initial_prepare_time += std::chrono::steady_clock::now() - phase_start;
  }
  for (auto& buffer : buffers) {
    auto* si = db_network_->dbToSta(buffer.inst);
    std::unique_ptr<sta::InstancePinIterator> pins(network_->pinIterator(si));
    while (pins->hasNext()) {
      auto* pin = pins->next();
      if (!network_->direction(pin)->isAnyOutput()) {
        continue;
      }
      auto* vertex = graph_->pinDrvrVertex(pin);
      if (vertex) {
        sta::VertexPathIterator paths(vertex, sta_);
        while (paths.hasNext()) {
          auto* path = paths.next();
          if (path->isClock(sta_)) {
            sta::PathExpanded expanded(path, false, sta_);
            buffer.depth = std::max(buffer.depth, expanded.size());
          }
        }
      }
    }
  }
  std::sort(
      buffers.begin(), buffers.end(), [&](const Buffer& a, const Buffer& b) {
        if (a.depth != b.depth) {
          return top_down ? a.depth < b.depth : a.depth > b.depth;
        }
        return std::string(a.inst->getName()) < b.inst->getName();
      });
  if (candidates.empty()) {
    std::unique_ptr<sta::LibertyLibraryIterator> libs(
        network_->libertyLibraryIterator());
    while (libs->hasNext()) {
      for (auto* cell : *libs->next()->buffers()) {
        if (isLinkCell(cell) && db_->findMaster(cell->name().c_str())) {
          candidates.push_back(cell);
        }
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [&](auto* a, auto* b) {
    const auto* am = db_->findMaster(a->name().c_str());
    const auto* bm = db_->findMaster(b->name().c_str());
    const int64_t aa = int64_t(am->getWidth()) * am->getHeight();
    const int64_t ba = int64_t(bm->getWidth()) * bm->getHeight();
    return aa != ba ? aa < ba : a->name() < b->name();
  });
  candidates.erase(std::unique(candidates.begin(), candidates.end()),
                   candidates.end());

  // Cache pairwise physical/model compatibility once per invocation. The
  // placed footprint remains an instance-specific check in every proposal.
  std::map<std::tuple<sta::LibertyCell*, sta::LibertyCell*, bool>, bool>
      compatible;
  auto available = [&](sta::LibertyCell* from,
                       sta::LibertyCell* to,
                       bool honor_dont_use = true) {
    const auto key = std::make_tuple(from, to, honor_dont_use);
    auto found = compatible.find(key);
    if (found != compatible.end()) {
      return found->second;
    }
    auto* master = db_->findMaster(from->name().c_str());
    auto* replacement = db_->findMaster(to->name().c_str());
    auto* site = master->getSite();
    bool ok = replacement && (!honor_dont_use || !dontUse(to))
              && replacement->isCore() && site && site->getWidth() > 0
              && replacement->getSite() == site && replacement->getWidth() > 0
              && replacement->getHeight() > 0
              && replacement->getWidth() % site->getWidth() == 0
              && replacement->getHeight() == master->getHeight()
              && replacement->getWidth() < master->getWidth()
              && (!master->getSymmetryX() || replacement->getSymmetryX())
              && (!master->getSymmetryY() || replacement->getSymmetryY())
              && (!master->getSymmetryR90() || replacement->getSymmetryR90())
              && sta::equivCellPorts(from, to) && sta::equivCellsArcs(from, to)
              && master->getMTerms().size() == replacement->getMTerms().size();
    if (ok) {
      for (auto* term : master->getMTerms()) {
        auto* other = replacement->findMTerm(term->getName().c_str());
        ok &= other && other->getIoType() == term->getIoType()
              && other->getSigType() == term->getSigType();
      }
    }
    for (auto* scene : sta_->scenes()) {
      for (auto* mm : sta::MinMax::range()) {
        auto* fm = scene_model(from, scene, mm);
        auto* tm = scene_model(to, scene, mm);
        ok &= fm && tm && sta::equivCellPorts(fm, tm)
              && sta::equivCellFuncs(fm, tm) && sta::equivCellsArcs(fm, tm);
      }
    }
    compatible[key] = ok;
    return ok;
  };
  float final_budget = 0;
  float intermediate_budget = 0;
  if (batch) {
    sta::LibertyCell* reference = nullptr;
    float intrinsic = std::numeric_limits<float>::infinity();
    bool valid_intrinsic = true;
    if (max_timing_regression < 0) {
      // The technology reference is independent of the allowed resize list
      // and dont_use: characterization never inserts this cell.
      sta::LibertyCellSeq references;
      std::unique_ptr<sta::LibertyLibraryIterator> libs(
          network_->libertyLibraryIterator());
      while (libs->hasNext()) {
        for (auto* cell : *libs->next()->buffers()) {
          if (isLinkCell(cell) && db_->findMaster(cell->name().c_str())) {
            references.push_back(cell);
          }
        }
      }
      std::sort(references.begin(), references.end(), [&](auto* a, auto* b) {
        const auto* am = db_->findMaster(a->name().c_str());
        const auto* bm = db_->findMaster(b->name().c_str());
        const int64_t aa = int64_t(am->getWidth()) * am->getHeight();
        const int64_t ba = int64_t(bm->getWidth()) * bm->getHeight();
        return aa != ba ? aa < ba : a->name() < b->name();
      });
      for (auto* candidate : references) {
        if (complete_models(candidate)
            && std::any_of(
                buffers.begin(), buffers.end(), [&](const Buffer& buffer) {
                  return available(
                      network_->libertyCell(db_network_->dbToSta(buffer.inst)),
                      candidate,
                      false);
                })) {
          reference = candidate;
          break;
        }
      }
      if (reference) {
        try {
          for (auto* scene : sta_->scenes()) {
            for (auto* mm : sta::MinMax::range()) {
              sta::LibertyPort *input, *output;
              reference->sceneCell(scene, mm)->bufferPorts(input, output);
              for (auto* rf : sta::RiseFall::range()) {
                bool found = false;
                for (auto* arc_set : output->libertyCell()->timingArcSets()) {
                  if (arc_set->from() != input || arc_set->to() != output
                      || arc_set->role()->isTimingCheck()) {
                    continue;
                  }
                  for (auto* arc : arc_set->arcs()) {
                    if (arc->toEdge()->asRiseFall() != rf) {
                      continue;
                    }
                    sta::LoadPinIndexMap loads(network_);
                    auto result = arc_delay_calc_->gateDelay(
                        nullptr, arc, 0, 0, nullptr, loads, scene, mm);
                    const float delay = sta::delayAsFloat(result.gateDelay());
                    valid_intrinsic &= std::isfinite(delay) && delay > 0;
                    intrinsic = std::min(intrinsic, delay);
                    found = true;
                  }
                }
                valid_intrinsic &= found;
              }
            }
          }
        } catch (...) {
          arc_delay_calc_->finishDrvrPin();
          throw;
        }
        arc_delay_calc_->finishDrvrPin();
      }
      if (!reference || !valid_intrinsic || !std::isfinite(intrinsic)
          || intrinsic <= 0) {
        logger_->warn(utl::RSZ,
                      3320,
                      "No finite positive compatible buffer intrinsic delay; "
                      "using zero timing regression budget.");
        final_budget = 0;
      } else {
        final_budget = intrinsic / 2;
      }
    } else {
      final_budget = max_timing_regression;
    }
    intermediate_budget = max_intermediate_timing_regression < 0
                              ? 2 * final_budget
                              : max_intermediate_timing_regression;
    if (!std::isfinite(intermediate_budget)
        || intermediate_budget < final_budget) {
      logger_->error(
          utl::RSZ, kInvalidLimits, "Invalid recover_clock_power limits.");
    }
    logger_->report(
        "recover_clock_power timing budgets: final {:.9g} s, intermediate "
        "{:.9g} s; "
        "reference {}, intrinsic delay {:.9g} s (minimum over "
        "scenes/min/max/transitions/arcs at zero input slew and zero external "
        "load); each metric is measured cumulatively from command start.",
        final_budget,
        intermediate_budget,
        reference ? reference->name() : "explicit/unavailable",
        reference && valid_intrinsic && std::isfinite(intrinsic) ? intrinsic
                                                                 : 0);
  }
  if (buffers.empty()) {
    const float clock_power = measure_power();
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
    logger_->report(
        "recover_clock_power: initial {:.9g} W, final {:.9g} W; changed 0, "
        "trials 0, STA evaluations 1, elapsed {:.3f}s, stop "
        "no_eligible_clock_buffers.",
        clock_power,
        clock_power,
        elapsed);
    report_profile();
    return 0;
  }
  int evaluations = 0;
  using ClockDomainsByPinMode
      = std::map<std::pair<const sta::Pin*, const sta::Mode*>, sta::ClockSet>;
  auto collect_electrical = [&](sta::Vertex* vertex,
                                Snapshot& result,
                                ClockDomainsByPinMode& domains_by_pin_mode) {
    auto* pin = vertex->pin();
    for (auto* scene : sta_->scenes()) {
      if (scene->sdc()->isDisabledConstraint(pin)
          || sta_->isIdealClock(pin, scene->mode())) {
        continue;
      }
      auto [domains_it, inserted]
          = domains_by_pin_mode.try_emplace({pin, scene->mode()});
      if (inserted) {
        domains_it->second = sta_->search()->clockDomains(pin, scene->mode());
      }
      const auto& domains = domains_it->second;
      for (auto* mm : sta::MinMax::range()) {
        float cap, cap_limit, cap_slack;
        const sta::RiseFall* cap_rf;
        const sta::Scene* cap_scene;
        sta_->checkCapacitance(
            pin, {scene}, mm, cap, cap_limit, cap_slack, cap_rf, cap_scene);
        float pin_cap_limit;
        bool pin_cap_exists;
        scene->sdc()->capacitanceLimit(pin, mm, pin_cap_limit, pin_cap_exists);
        if (network_->isDriver(pin) && pin_cap_exists
            && (!cap_scene || mm->compare(cap_limit, pin_cap_limit))) {
          cap_limit = pin_cap_limit;
          cap_scene = scene;
        }
        for (auto* rf : sta::RiseFall::range()) {
          float limit = 0;
          bool exists = false;
          sta_->findSlewLimit(
              network_->libertyPort(pin), scene, mm, limit, exists);
          auto tighten = [&](float value, bool found) {
            if (found && (!exists || mm->compare(limit, value))) {
              limit = value;
              exists = true;
            }
          };
          float value;
          bool found;
          for (auto* clock : domains) {
            // Match STA's propagated-clock/data limit selection.
            scene->sdc()->slewLimit(
                clock, rf, sta::PathClkOrData::data, mm, value, found);
            tighten(value, found);
          }
          if (network_->isTopLevelPort(pin)) {
            auto* port = network_->port(pin);
            scene->sdc()->slewLimit(port, mm, value, found);
            tighten(value, found);
            auto* drive = scene->sdc()->findInputDrive(port);
            if (drive) {
              for (auto* drive_rf : sta::RiseFall::range()) {
                const sta::LibertyCell* cell;
                const sta::LibertyPort *from, *to;
                const sta::DriveCellSlews* slews;
                drive->driveCell(drive_rf, mm, cell, from, slews, to);
                if (to) {
                  auto* sp = to->scenePort(scene, mm);
                  sp->slewLimit(mm, value, found);
                  if (!found && sp->direction()->isAnyOutput()
                      && mm == sta::MinMax::max()) {
                    sp->libertyLibrary()->defaultMaxSlew(value, found);
                  }
                  tighten(value, found);
                }
              }
            }
          }
          if (exists) {
            const float slew = sta::delayAsFloat(
                graph_->slew(vertex, rf, scene->dcalcAnalysisPtIndex(mm)));
            result.electrical[{pin, scene, rf->index(), mm->index(), false}]
                = mm == sta::MinMax::max() ? limit - slew : slew - limit;
          }
          if (cap_scene) {
            const float load
                = sta_->graphDelayCalc()->loadCap(pin, rf, scene, mm);
            result.electrical[{pin, scene, rf->index(), mm->index(), true}]
                = mm == sta::MinMax::max() ? cap_limit - load
                                           : load - cap_limit;
          }
        }
      }
    }
  };
  // Snapshot every pin, not merely a global worst, so a new local violation
  // cannot hide behind a preexisting violation elsewhere.
  auto snapshot
      = [&](const std::function<bool(const Snapshot&)>& early_check = {},
            const std::function<bool(const Snapshot&)>& electrical_check = {}) {
          ++evaluations;
          ++snapshot_prepare_calls;
          auto phase_start = verbose ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
          sta_->findRequireds();
          sta_->ensureClkArrivals();
          if (verbose) {
            snapshot_prepare_time
                += std::chrono::steady_clock::now() - phase_start;
          }
          Snapshot result;
          ++traversal_calls;
          phase_start = verbose ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
          // findRequireds() has already refreshed every arrival. Read those
          // paths directly rather than repeating the STA search preamble for
          // each pin.
          sta::VertexIterator clock_vertices(graph_);
          while (clock_vertices.hasNext()) {
            auto* vertex = clock_vertices.next();
            auto* pin = vertex->pin();
            if (vertex->isRegClk()) {
              bool clocked = false;
              sta::VertexPathIterator paths(vertex, sta_);
              while (paths.hasNext()) {
                auto* path = paths.next();
                if (!path->isClock(sta_) || !path->clock(sta_)) {
                  continue;
                }
                clocked = true;
                if (!sta_->isPropagatedClock(pin, path->mode(sta_))) {
                  logger_->error(
                      utl::RSZ, 3314, "Unpropagated sequential clock sink.");
                }
                sta::PathExpanded expanded(path, false, sta_);
                if (expanded.size() == 0) {
                  logger_->error(
                      utl::RSZ, 3307, "Unavailable clock source path.");
                }
                const float latency = sta::delayAsFloat(
                    path->arrival() - expanded.path(0)->arrival());
                if (!std::isfinite(latency)) {
                  throw std::runtime_error("nonfinite clock network latency");
                }
                auto& envelope
                    = result.clocks[{path->clock(sta_),
                                     path->scene(sta_),
                                     path->clkEdge(sta_)->transition()->index(),
                                     path->rfIndex(sta_)}];
                envelope.early = std::min(envelope.early, latency);
                envelope.late = std::max(envelope.late, latency);
                envelope.sinks.insert(pin);
                if (batch) {
                  result.sink_arrivals.push_back(
                      {pin,
                       {path->clock(sta_),
                        path->scene(sta_),
                        path->clkEdge(sta_)->transition()->index(),
                        path->rfIndex(sta_)},
                       path->minMax(sta_)->index(),
                       latency});
                }
              }
              if (!clocked) {
                result.unclocked_sinks.insert(pin);
              }
            }
          }
          if (verbose) {
            traversal_time += std::chrono::steady_clock::now() - phase_start;
          }
          phase_start = verbose ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
          for (auto* scene : sta_->scenes()) {
            for (auto* mm : sta::MinMax::range()) {
              ++slack_queries;
              sta::Slack slack;
              sta::Vertex* endpoint = nullptr;
              sta_->worstSlack(scene, mm, slack, endpoint);
              if (endpoint && std::isfinite(sta::delayAsFloat(slack))
                  && sta::delayAsFloat(slack) < sta::INF) {
                result.slacks[{scene, mm->index()}] = sta::delayAsFloat(slack);
              }
            }
          }
          if (verbose) {
            slack_time += std::chrono::steady_clock::now() - phase_start;
          }
          if (early_check && !early_check(result)) {
            result.timing_rejected = true;
            ++skipped_power_trials;
            return result;
          }
          // Scenes of the same mode share memberships; rebuild this cache after
          // every trial, commit and rollback along with the rest of the
          // snapshot.
          ClockDomainsByPinMode domains_by_pin_mode;
          ++traversal_calls;
          phase_start = verbose ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
          sta::VertexIterator electrical_vertices(graph_);
          while (electrical_vertices.hasNext()) {
            auto* vertex = electrical_vertices.next();
            collect_electrical(vertex, result, domains_by_pin_mode);
          }
          if (verbose) {
            traversal_time += std::chrono::steady_clock::now() - phase_start;
          }
          if (electrical_check && !electrical_check(result)) {
            result.electrical_rejected = true;
            ++skipped_electrical_power_trials;
            return result;
          }
          result.power = measure_power();
          result.power_computed = true;
          return result;
        };
  sta_->checkSlewsPreamble();
  sta_->checkCapacitancesPreamble(sta_->scenes());
  est::IncrementalParasiticsGuard parasitics(estimate_parasitics_);
  const auto baseline_validation_start = std::chrono::steady_clock::now();
  refresh_timing();
  const Snapshot baseline = snapshot();
  const double baseline_validation_seconds
      = std::chrono::duration<double>(std::chrono::steady_clock::now()
                                      - baseline_validation_start)
            .count();
  if (baseline.clocks.empty()) {
    logger_->error(utl::RSZ,
                   3308,
                   "No propagated sequential clock-sink groups available.");
  }
  if (!baseline.unclocked_sinks.empty()) {
    logger_->warn(utl::RSZ,
                  3312,
                  "{} sequential clock pins have no clock paths; their "
                  "latency/skew checks are unavailable.",
                  baseline.unclocked_sinks.size());
  }
  for (auto* scene : sta_->scenes()) {
    const bool has_group
        = std::any_of(baseline.clocks.begin(),
                      baseline.clocks.end(),
                      [scene](const auto& entry) {
                        return std::get<1>(entry.first) == scene;
                      });
    if (!has_group) {
      logger_->warn(utl::RSZ,
                    3313,
                    "No propagated clock-sink group in scene {}; latency/skew "
                    "checks unavailable.",
                    scene->name());
    }
    for (auto* mm : sta::MinMax::range()) {
      if (!baseline.slacks.count({scene, mm->index()})) {
        logger_->warn(
            utl::RSZ,
            3309,
            "No constrained {} endpoints in scene {}; check unavailable.",
            mm == sta::MinMax::max() ? "setup" : "hold",
            scene->name());
      }
    }
  }
  auto clock_limits = [&](const ClockKey& key, float budget) {
    const auto& initial = baseline.clocks.at(key);
    if (!batch) {
      return std::make_pair(
          max_latency < 0 ? initial.late : max_latency,
          max_skew < 0 ? initial.late - initial.early : max_skew);
    }
    const float extra = std::max(0.0f, budget - final_budget);
    return std::make_pair(
        std::min(initial.late + budget,
                 max_latency < 0 ? sta::INF : max_latency + extra),
        std::min(initial.late - initial.early + budget,
                 max_skew < 0 ? sta::INF : max_skew + extra));
  };
  auto slack_floor = [&](const SlackKey& key, float budget) {
    const float initial = baseline.slacks.at(key);
    const bool setup = key.second == sta::MinMax::max()->index();
    const float margin = setup ? setup_margin : hold_margin;
    if (!batch) {
      return std::min(initial, margin);
    }
    float floor = initial - budget;
    if (setup ? setup_margin_given : hold_margin_given) {
      floor = std::max(
          floor,
          std::min(initial, margin) - std::max(0.0f, budget - final_budget));
    }
    return floor;
  };
  auto feasible_clocks = [&](const Snapshot& state,
                             bool report_diagnostics,
                             float budget = -1) {
    budget = budget < 0 ? final_budget : budget;
    if (state.clocks.size() != baseline.clocks.size()
        || state.unclocked_sinks != baseline.unclocked_sinks) {
      return false;
    }
    for (const auto& [key, initial] : baseline.clocks) {
      auto it = state.clocks.find(key);
      if (it == state.clocks.end() || it->second.sinks != initial.sinks) {
        return false;
      }
      const auto& group = it->second;
      const auto [latency_limit, skew_limit] = clock_limits(key, budget);
      if (!std::isfinite(group.early) || !std::isfinite(group.late)
          || below(latency_limit, group.late)
          || below(skew_limit, group.late - group.early)) {
        if (verbose && report_diagnostics) {
          logger_->report(
              "recover_clock_power: clock envelope rejected for {} in scene "
              "{} (source transition {}, sink transition {}), latency {:.9g} "
              "(limit {:.9g}), skew {:.9g} (limit {:.9g}); early {:.9g} "
              "(initial {:.9g}), late {:.9g} (initial {:.9g}).",
              std::get<0>(key)->name(),
              std::get<1>(key)->name(),
              std::get<2>(key),
              std::get<3>(key),
              group.late,
              latency_limit,
              group.late - group.early,
              skew_limit,
              group.early,
              initial.early,
              group.late,
              initial.late);
        }
        return false;
      }
    }
    return true;
  };
  auto feasible_clock_slack
      = [&](const Snapshot& state, bool report_diagnostics, float budget = -1) {
          budget = budget < 0 ? final_budget : budget;
          if (!feasible_clocks(state, report_diagnostics, budget)
              || state.slacks.size() != baseline.slacks.size()) {
            return false;
          }
          for (const auto& [key, initial] : baseline.slacks) {
            auto it = state.slacks.find(key);
            const float floor = slack_floor(key, budget);
            if (it == state.slacks.end() || below(it->second, floor)) {
              if (verbose && report_diagnostics && it != state.slacks.end()) {
                logger_->report(
                    "recover_clock_power: {} slack {:.9g} below floor {:.9g}.",
                    key.second,
                    it->second,
                    floor);
              }
              return false;
            }
          }
          return true;
        };
  auto feasible_electrical = [&](const Snapshot& state) {
    for (const auto& [key, margin] : state.electrical) {
      auto it = baseline.electrical.find(key);
      const float floor
          = it == baseline.electrical.end() ? 0 : std::min(0.0f, it->second);
      // Capacitance margins are much smaller than time margins; no absolute
      // time tolerance is appropriate for them.
      const float tolerance = std::get<4>(key)
                                  ? std::max(1e-21f, std::abs(floor) * 1e-5f)
                                  : std::max(1e-15f, std::abs(floor) * 1e-5f);
      if (!std::isfinite(margin) || margin < floor - tolerance) {
        if (verbose) {
          logger_->report(
              "recover_clock_power: {} electrical {} margin {:.9g} below floor "
              "{:.9g}.",
              network_->pathName(std::get<0>(key)),
              std::get<4>(key) ? "cap" : "slew",
              margin,
              floor);
        }
        return false;
      }
    }
    return true;
  };
  auto feasible = [&](const Snapshot& state, float budget = -1) {
    return feasible_clock_slack(state, true, budget)
           && feasible_electrical(state) && state.power_computed
           && std::isfinite(state.power);
  };
  if (!feasible(baseline)) {
    logger_->error(
        utl::RSZ,
        3310,
        "Initial clock network exceeds supplied latency/skew limits.");
  }
  struct Original
  {
    odb::dbInst* inst;
    sta::LibertyCell* cell;
    odb::Point origin;
  };
  std::vector<Original> accepted;
  std::set<unsigned int> changed;
  float current_power = baseline.power;
  int trials = 0;
  bool journal_open = false;
  std::string stop = "max_passes";
  auto restore_all = [&]() {
    if (journal_open) {
      journalRestore();
      journal_open = false;
    }
    for (auto it = accepted.rbegin(); it != accepted.rend(); ++it) {
      auto* si = db_network_->dbToSta(it->inst);
      sta_->replaceCell(
          si, db_network_->dbToSta(db_->findMaster(it->cell->name().c_str())));
      it->inst->setOrigin(it->origin.x(), it->origin.y());
    }
    refresh_timing();
    initDesignArea();
  };
  if (batch) {
    auto report_slacks = [&](const char* phase, const Snapshot& state) {
      if (verbose) {
        for (const auto& [key, value] : state.slacks) {
          logger_->report(
              "recover_clock_power batch {}: scene {} {} slack {:.9g} s.",
              phase,
              key.first->name(),
              key.second == sta::MinMax::max()->index() ? "setup" : "hold",
              value);
        }
      }
    };
    report_slacks("baseline", baseline);
    // Do not build the proposal graph after the mandatory baseline has
    // already exhausted the wall budget, or when only final validation fits.
    if (max_checkpoints == 2
        || std::chrono::duration<double>(std::chrono::steady_clock::now()
                                         - start)
                   .count()
               >= max_seconds) {
      refresh_timing();
      const auto final = snapshot();
      if (!feasible(final)
          || std::abs(final.power - baseline.power)
                 > std::max(1e-20f, std::abs(baseline.power) * 1e-5f)) {
        throw std::runtime_error(
            "batch unchanged final feasibility/power check failed");
      }
      report_slacks("final", final);
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      logger_->report(
          "recover_clock_power: initial {:.9g} W, final {:.9g} W; changed 0, "
          "trials 0, STA evaluations {}, elapsed {:.3f}s, stop {}.",
          baseline.power,
          final.power,
          evaluations,
          elapsed,
          max_checkpoints == 2 ? "max_checkpoints" : "max_seconds");
      logger_->report(
          "recover_clock_power batch summary: attempts 0, accepted 0, full "
          "validations {}, native power calls {}, proposed 0, committed 0, "
          "excluded 0.",
          evaluations,
          power_calls);
      report_profile();
      initDesignArea();
      return 0;
    }
    // Proposal values are surrogates. Only snapshot() certifies timing,
    // electrical feasibility and native clock power; activities remain
    // STA-owned.
    struct Node
    {
      odb::dbInst* inst;
      sta::LibertyCell* cell;
      const sta::Pin* input;
      const sta::Pin* output;
      int parent = -1;
      std::vector<int> children;
      bool eligible = false;
      bool boundary = false;
    };
    std::vector<Node> nodes;
    std::map<const sta::Instance*, int> node_by_inst;
    std::map<sta::LibertyCell*, bool> model_coverage;
    // Shared high-fanout clock nets are traversed only once for driver lookup.
    std::map<const sta::Net*, std::vector<const sta::Pin*>> net_drivers;
    auto drivers_for
        = [&](const sta::Pin* pin) -> const std::vector<const sta::Pin*>& {
      auto [it, inserted] = net_drivers.try_emplace(network_->net(pin));
      if (inserted && it->first) {
        std::unique_ptr<sta::NetConnectedPinIterator> pins(
            network_->connectedPinIterator(it->first));
        while (pins->hasNext()) {
          const auto* other = pins->next();
          if (network_->isDriver(other)) {
            it->second.push_back(other);
          }
        }
      }
      return it->second;
    };
    std::set<unsigned int> eligible;
    for (const auto& buffer : buffers) {
      eligible.insert(buffer.inst->getId());
    }
    // Include fixed ordinary buffers as load/arrival anchors. A first buffer
    // behind a primary source or unsupported cell keeps its input capacitance.
    for (auto* inst : block_->getInsts()) {
      auto* si = db_network_->dbToSta(inst);
      auto* cell = network_->libertyCell(si);
      if (!cell || !cell->isBuffer()) {
        continue;
      }
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      const auto* in_pin = network_->findPin(si, input);
      const auto* out_pin = network_->findPin(si, output);
      if (!in_pin || !out_pin) {
        continue;
      }
      bool is_clock = false;
      for (auto* mode : sta_->modes()) {
        is_clock |= sta_->isClock(out_pin, mode);
      }
      if (!is_clock) {
        continue;
      }
      auto [coverage, inserted] = model_coverage.try_emplace(cell, false);
      if (inserted) {
        coverage->second = complete_models(cell);
      }
      if (!coverage->second) {
        continue;
      }
      node_by_inst[si] = nodes.size();
      nodes.push_back({inst,
                       cell,
                       in_pin,
                       out_pin,
                       -1,
                       {},
                       eligible.count(inst->getId()) != 0,
                       false});
    }
    for (size_t i = 0; i < nodes.size(); ++i) {
      auto& node = nodes[i];
      const auto& drivers = drivers_for(node.input);
      for (const auto* pin : drivers) {
        auto it = node_by_inst.find(network_->instance(pin));
        if (it != node_by_inst.end() && nodes[it->second].output == pin) {
          node.parent = it->second;
        }
      }
      if (drivers.size() != 1 || node.parent < 0) {
        node.parent = -1;
        node.eligible = false;
      } else {
        nodes[node.parent].children.push_back(i);
      }
      for (auto* mode : sta_->modes()) {
        for (auto* clock : mode->sdc()->clocks()) {
          if (clock->isGenerated()
              && (clock->leafPins().count(node.input)
                  || clock->leafPins().count(node.output))) {
            node.boundary = true;
          }
        }
      }
      std::unique_ptr<sta::PinConnectedPinIterator> loads(
          network_->connectedPinIterator(node.output));
      while (loads->hasNext()) {
        const auto* pin = loads->next();
        if (pin == node.output || !network_->isLoad(pin)) {
          continue;
        }
        auto it = node_by_inst.find(network_->instance(pin));
        if (it != node_by_inst.end() && nodes[it->second].input == pin) {
          continue;
        }
        auto* vertex = graph_->pinLoadVertex(pin);
        if (vertex && vertex->isRegClk()) {
          continue;
        }
        // CTS dummy loads have no functional fanout. Their input capacitance
        // is retained in the native baseline load and never resized here.
        auto* load_inst = network_->instance(pin);
        auto* load_cell
            = load_inst ? network_->libertyCell(load_inst) : nullptr;
        bool dummy
            = load_cell && (load_cell->isBuffer() || load_cell->isInverter());
        if (dummy) {
          std::unique_ptr<sta::InstancePinIterator> outputs(
              network_->pinIterator(load_inst));
          while (outputs->hasNext()) {
            const auto* output = outputs->next();
            if (network_->direction(output)->isAnyOutput()
                && network_->net(output)) {
              std::unique_ptr<sta::PinConnectedPinIterator> fanout(
                  network_->connectedPinIterator(output));
              while (fanout->hasNext()) {
                const auto* other = fanout->next();
                if (other != output && network_->isLoad(other)) {
                  dummy = false;
                }
              }
            }
          }
        }
        if (!dummy) {
          node.boundary = true;
        }
      }
    }
    // Changes above an unsupported clock boundary can change its generated
    // waveform. Freeze that ancestry, while retaining ordinary branches below
    // a frozen first buffer as independent supported islands.
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (nodes[i].boundary) {
        std::set<int> seen;
        for (int n = i; n >= 0 && seen.insert(n).second; n = nodes[n].parent) {
          nodes[n].eligible = false;
          // Keep immediate sibling/child input capacitances fixed too: their
          // load changes would perturb this frozen boundary's waveform.
          for (int child : nodes[n].children) {
            nodes[child].eligible = false;
          }
        }
      }
    }
    std::vector<int> order;
    std::vector<int> depth(nodes.size(), -1);
    std::function<int(int)> node_depth = [&](int n) {
      if (depth[n] == -2) {
        nodes[n].eligible = false;
        nodes[n].parent = -1;
        return 0;
      }
      if (depth[n] < 0) {
        depth[n] = -2;
        depth[n] = nodes[n].parent < 0 ? 0 : node_depth(nodes[n].parent) + 1;
      }
      return depth[n];
    };
    for (size_t i = 0; i < nodes.size(); ++i) {
      node_depth(i);
      order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      if (depth[a] != depth[b]) {
        return depth[a] > depth[b];
      }
      return std::string(nodes[a].inst->getName()) < nodes[b].inst->getName();
    });
    size_t excluded = 0;
    for (const auto& node : nodes) {
      excluded += eligible.count(node.inst->getId()) && !node.eligible;
    }
    logger_->report(
        "recover_clock_power batch: {} eligible buffers, {} excluded "
        "at unsupported/source boundaries; proposal delay/slew is "
        "a Liberty surrogate with fixed wire delay.",
        buffers.size() - excluded,
        excluded);

    auto ports = [&](sta::LibertyCell* cell,
                     const sta::Scene* scene,
                     const sta::MinMax* mm) {
      sta::LibertyPort *input, *output;
      cell->sceneCell(scene, mm)->bufferPorts(input, output);
      return std::make_pair(input, output);
    };
    // Evaluate every matching raw arc, including proper early/min aggregation.
    // The surrogate deliberately omits changes to wire RC and pin position.
    auto model = [&](sta::LibertyCell* cell,
                     float load,
                     float slew,
                     const sta::Scene* scene,
                     const sta::MinMax* mm,
                     const sta::RiseFall* rf) {
      auto* output = ports(cell, scene, mm).second;
      float delay = mm == sta::MinMax::max() ? -sta::INF : sta::INF;
      float out_slew = delay;
      bool found = false;
      for (auto* arc_set : output->libertyCell()->timingArcSets()) {
        if (arc_set->to() != output || arc_set->role()->isTimingCheck()) {
          continue;
        }
        for (auto* arc : arc_set->arcs()) {
          if (arc->fromEdge()->asRiseFall() != rf
              || arc->toEdge()->asRiseFall() != rf) {
            continue;
          }
          sta::LoadPinIndexMap load_pins(network_);
          auto result = arc_delay_calc_->gateDelay(nullptr,
                                                   arc,
                                                   slew,
                                                   std::max(0.0f, load),
                                                   nullptr,
                                                   load_pins,
                                                   scene,
                                                   mm);
          float d = sta::delayAsFloat(result.gateDelay());
          float s = sta::delayAsFloat(result.drvrSlew());
          delay = mm == sta::MinMax::max() ? std::max(delay, d)
                                           : std::min(delay, d);
          out_slew = mm == sta::MinMax::max() ? std::max(out_slew, s)
                                              : std::min(out_slew, s);
          found = true;
        }
      }
      if (!found || !std::isfinite(delay) || !std::isfinite(out_slew)) {
        return std::make_pair(0.0f, 0.0f);
      }
      return std::make_pair(delay, out_slew);
    };
    struct Move
    {
      int node;
      sta::LibertyCell* cell;
    };
    using ProjectionKey = std::tuple<const sta::Scene*, int, int>;
    using Delta = std::pair<float, float>;  // arrival and slew changes
    // Call-scoped candidate lists preserve the area ordering while avoiding
    // repeated physical/model discovery during local projection rounds.
    std::vector<sta::LibertyCellSeq> compatible_choices(nodes.size());
    for (size_t n = 0; n < nodes.size(); ++n) {
      if (!nodes[n].eligible) {
        continue;
      }
      for (auto* candidate : candidates) {
        if (available(nodes[n].cell, candidate)) {
          compatible_choices[n].push_back(candidate);
        }
      }
      compatible_choices[n].push_back(nodes[n].cell);
    }
    std::map<const sta::Pin*, int> sink_parents;
    for (const auto& arrival : baseline.sink_arrivals) {
      auto [it, inserted] = sink_parents.try_emplace(arrival.pin, -1);
      if (inserted) {
        for (const auto* pin : drivers_for(arrival.pin)) {
          auto node = node_by_inst.find(network_->instance(pin));
          if (node != node_by_inst.end()) {
            it->second = node->second;
          }
        }
      }
    }
    // Native per-instance reads share the still-valid baseline power cache.
    // Their values rank proposals only and are never refreshed after edits.
    std::vector<double> baseline_instance_power(nodes.size(), 0);
    size_t baseline_instance_power_reads = 0;
    for (size_t n = 0; n < nodes.size(); ++n) {
      if (nodes[n].eligible) {
        const double power
            = sta_->power(db_network_->dbToSta(nodes[n].inst), sta_->cmdScene())
                  .total();
        baseline_instance_power[n] = std::isfinite(power) ? power : 0;
        ++baseline_instance_power_reads;
      }
    }
    size_t projection_rounds = 0, candidate_relaxations = 0;
    size_t tentative_checks = 0, tentative_accepts = 0, tentative_rejects = 0;
    double tentative_wns_seconds = 0;
    size_t cached_clock_checks = 0, cached_clock_rejects = 0;
    double cached_clock_seconds = 0;
    size_t local_electrical_checks = 0, local_electrical_rejects = 0;
    double local_electrical_seconds = 0;
    std::set<const sta::Pin*> watched_slew_pins;
    for (const auto& [key, margin] : baseline.electrical) {
      if (!std::get<4>(key) && margin < 0) {
        watched_slew_pins.insert(std::get<0>(key));
      }
    }
    // Only buffer masters change, so sequential sink pins remain valid across
    // inner ECOs. Resolve vertices anew after every timing refresh.
    std::set<const sta::Pin*> cached_clock_sinks = baseline.unclocked_sinks;
    for (const auto& [key, group] : baseline.clocks) {
      cached_clock_sinks.insert(group.sinks.begin(), group.sinks.end());
    }
    auto cached_clocks_feasible = [&]() {
      Snapshot state;
      for (const auto* pin : cached_clock_sinks) {
        auto* vertex = graph_->pinLoadVertex(pin);
        if (!vertex || !vertex->isRegClk()) {
          return std::make_pair(false, false);
        }
        bool clocked = false;
        sta::VertexPathIterator paths(vertex, sta_);
        while (paths.hasNext()) {
          auto* path = paths.next();
          if (!path->isClock(sta_) || !path->clock(sta_)) {
            continue;
          }
          clocked = true;
          sta::PathExpanded expanded(path, false, sta_);
          if (expanded.size() == 0) {
            return std::make_pair(false, false);
          }
          const float latency = sta::delayAsFloat(
              path->arrival() - expanded.path(0)->arrival());
          if (!std::isfinite(latency)) {
            return std::make_pair(false, false);
          }
          auto& group
              = state.clocks[{path->clock(sta_),
                              path->scene(sta_),
                              path->clkEdge(sta_)->transition()->index(),
                              path->rfIndex(sta_)}];
          group.early = std::min(group.early, latency);
          group.late = std::max(group.late, latency);
          group.sinks.insert(pin);
        }
        if (!clocked) {
          state.unclocked_sinks.insert(pin);
        }
      }
      return std::make_pair(feasible_clocks(state, false, intermediate_budget),
                            feasible_clocks(state, false, final_budget));
    };
    auto local_electrical_feasible = [&](const Node& node) {
      std::set<const sta::Pin*> pins = watched_slew_pins;
      std::unique_ptr<sta::InstancePinIterator> instance_pins(
          network_->pinIterator(db_network_->dbToSta(node.inst)));
      while (instance_pins->hasNext()) {
        const auto* pin = instance_pins->next();
        pins.insert(pin);
        if (network_->direction(pin)->isAnyInput()) {
          const auto& drivers = drivers_for(pin);
          pins.insert(drivers.begin(), drivers.end());
        }
      }
      Snapshot state;
      ClockDomainsByPinMode domains_by_pin_mode;
      std::set<sta::Vertex*> vertices;
      for (const auto* pin : pins) {
        auto* load = graph_->pinLoadVertex(pin);
        auto* driver = graph_->pinDrvrVertex(pin);
        if (!load && !driver) {
          return false;
        }
        if (load) {
          vertices.insert(load);
        }
        if (driver) {
          vertices.insert(driver);
        }
      }
      for (auto* vertex : vertices) {
        collect_electrical(vertex, state, domains_by_pin_mode);
      }
      return feasible_electrical(state);
    };
    bool inner_journal_open = false;
    Snapshot certified = baseline;
    Snapshot best = baseline;
    size_t best_boundary = 0, best_updates = 0, boundary_deferrals = 0;
    struct MasterState
    {
      odb::dbInst* inst;
      odb::dbMaster* initial;
      odb::dbMaster* best;
    };
    std::vector<MasterState> master_states;
    for (const auto& buffer : buffers) {
      master_states.push_back(
          {buffer.inst, buffer.inst->getMaster(), buffer.inst->getMaster()});
    }
    size_t batch_attempts = 0, accepted_batches = 0;
    size_t proposed = 0, committed = 0;
    const auto expired = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now()
                                           - start)
                 .count()
             >= max_seconds;
    };
    const double validation_reserve
        = std::max(0.002, 2.5 * baseline_validation_seconds);
    auto time_for_tentative = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now()
                                           - start)
                     .count()
                 + validation_reserve
             < max_seconds;
    };
    auto expand_blocked = [&](std::set<int>& blocked) {
      const auto seeds = blocked;
      for (int seed : seeds) {
        std::set<int> seen;
        for (int n = seed; n >= 0 && seen.insert(n).second;
             n = nodes[n].parent) {
          blocked.insert(n);
          for (int child : nodes[n].children) {
            blocked.insert(child);
          }
        }
      }
    };
    auto near_bound = [](float value, float bound) {
      return std::abs(value - bound)
             <= std::max(1e-15f, std::abs(bound) * 1e-5f);
    };
    auto preserve_window = [&](const ClockKey& key) {
      const auto& current = certified.clocks.at(key);
      const auto [latency_limit, skew_limit]
          = clock_limits(key, intermediate_budget);
      return near_bound(latency_limit, current.late)
             || near_bound(skew_limit, current.late - current.early);
    };
    // Match the actual normalized clock path represented by one snapshot
    // entry. A sink may carry other clocks, transitions, scenes and path tags.
    auto mark_clock_arrival = [&](const SinkArrival& arrival,
                                  std::set<int>& marked) {
      auto* vertex = graph_->pinLoadVertex(arrival.pin);
      sta::VertexPathIterator paths(vertex, sta_);
      while (paths.hasNext()) {
        auto* path = paths.next();
        if (!path->isClock(sta_) || !path->clock(sta_)
            || path->minMax(sta_)->index() != arrival.min_max) {
          continue;
        }
        const ClockKey key{path->clock(sta_),
                           path->scene(sta_),
                           path->clkEdge(sta_)->transition()->index(),
                           path->rfIndex(sta_)};
        if (key != arrival.key) {
          continue;
        }
        sta::PathExpanded expanded(path, false, sta_);
        if (expanded.size() == 0
            || !near_bound(sta::delayAsFloat(path->arrival()
                                             - expanded.path(0)->arrival()),
                           arrival.latency)) {
          continue;
        }
        // Generated-clock source slew depends on the master-clock ancestry.
        // Include it in protection without changing normalized latency above.
        sta::PathExpanded protected_path(path, true, sta_);
        for (size_t i = 0; i < protected_path.size(); ++i) {
          const auto* pin = protected_path.path(i)->pin(sta_);
          auto node = node_by_inst.find(network_->instance(pin));
          if (node != node_by_inst.end()) {
            marked.insert(node->second);
          }
        }
      }
    };
    std::set<int> critical_frozen_total, early_frozen_total;
    size_t strong_fallbacks = 0;
    // Native diagnostics select a bounded revision. They never certify the
    // revision: its next complete snapshot does that.
    auto diagnose = [&](const Snapshot& failed) {
      std::set<int> blocked;
      auto mark_path = [&](const sta::Path* path) {
        sta::PathExpanded expanded(path, true, sta_);
        for (size_t i = 0; i < expanded.size(); ++i) {
          const auto* pin = expanded.path(i)->pin(sta_);
          auto it = node_by_inst.find(network_->instance(pin));
          if (it != node_by_inst.end()) {
            blocked.insert(it->second);
          }
        }
      };
      for (const auto& arrival : failed.sink_arrivals) {
        const auto& group = failed.clocks.at(arrival.key);
        const auto [latency_limit, skew_limit]
            = clock_limits(arrival.key, intermediate_budget);
        const bool spread_failed = below(skew_limit, group.late - group.early);
        const float upper = std::min(latency_limit, group.early + skew_limit);
        if (below(upper, arrival.latency)
            || (spread_failed
                && (near_bound(arrival.latency, group.early)
                    || near_bound(arrival.latency, group.late)))) {
          // Retain unrelated branches: do not mark every sink in a failed
          // clock group merely because its shared spread exceeds the bound.
          mark_clock_arrival(arrival, blocked);
        }
      }
      for (const auto& [key, initial] : baseline.slacks) {
        const auto it = failed.slacks.find(key);
        const float floor = slack_floor(key, intermediate_budget);
        if (it == failed.slacks.end() || !below(it->second, floor)) {
          continue;
        }
        auto* mm = sta::MinMax::find(key.second);
        for (auto* endpoint : sta_->search()->endpoints()) {
          float slack = sta::delayAsFloat(
              sta_->slack(endpoint,
                          sta::RiseFallBoth::riseFall(),
                          {sta_->findScene(key.first->name())},
                          mm));
          if (!below(slack, floor)) {
            continue;
          }
          sta::VertexPathIterator paths(endpoint, sta_);
          while (paths.hasNext()) {
            mark_path(paths.next());
          }
          // A data path's capture clock is not part of its expanded launch
          // path. Include all propagated clock pins on its endpoint instance.
          auto* inst = network_->instance(endpoint->pin());
          std::unique_ptr<sta::InstancePinIterator> pins(
              network_->pinIterator(inst));
          while (pins->hasNext()) {
            auto* vertex = graph_->pinLoadVertex(pins->next());
            if (vertex && vertex->isRegClk()) {
              sta::VertexPathIterator clock_paths(vertex, sta_);
              while (clock_paths.hasNext()) {
                auto* path = clock_paths.next();
                if (path->isClock(sta_)) {
                  mark_path(path);
                }
              }
            }
          }
        }
      }
      for (const auto& [key, margin] : failed.electrical) {
        auto it = baseline.electrical.find(key);
        const float floor
            = it == baseline.electrical.end() ? 0 : std::min(0.0f, it->second);
        if (margin < floor) {
          const auto* pin = std::get<0>(key);
          auto node = node_by_inst.find(network_->instance(pin));
          if (node != node_by_inst.end()) {
            blocked.insert(node->second);
          }
          std::unique_ptr<sta::PinConnectedPinIterator> pins(
              network_->connectedPinIterator(pin));
          while (pins->hasNext()) {
            auto other = node_by_inst.find(network_->instance(pins->next()));
            if (other != node_by_inst.end()) {
              blocked.insert(other->second);
            }
          }
        }
      }
      expand_blocked(blocked);
      return blocked;
    };
    auto restore_inner = [&]() {
      if (!inner_journal_open) {
        return;
      }
      // Undo consumes only the inner ECO. Release before throwable refresh so
      // exception cleanup cannot consume the outer or caller's ECO twice.
      inner_journal_open = false;
      odb::dbDatabase::undoEco(block_);
      RepairSetupProfiler::journalEnd(true);
      invalidateVertexOrdering();
      initDesignArea();
      refresh_timing();
    };
    auto restore_batch = [&]() {
      if (!journal_open) {
        return;
      }
      // Release ownership before undo: undo consumes this ECO before the
      // subsequent parasitic/timing update, which can throw. The invocation
      // handler must never undo a caller's enclosing ECO on that exception.
      journal_open = false;
      {
        const ProfileScope scope("journal");
        odb::dbDatabase::undoEco(block_);
        RepairSetupProfiler::journalEnd(true);
      }
      invalidateVertexOrdering();
      initDesignArea();
      refresh_timing();
    };
    try {
      for (int wave = 0; wave < max_passes; ++wave) {
        if (expired()) {
          stop = "max_seconds";
          break;
        }
        if (evaluations >= max_checkpoints - 1) {
          stop = "max_checkpoints";
          break;
        }
        if (max_trials && trials >= max_trials) {
          stop = "max_trials";
          break;
        }
        bool progress = false;
        bool wave_budget_stop = false;
        std::set<int> projection_frozen;
        // Preserve current certified early/late anchors when the original
        // clock budget has zero headroom. Their immediate children keep Cin
        // fixed, so sibling sizing cannot perturb the critical waveform.
        for (const auto& arrival : certified.sink_arrivals) {
          const auto& current = certified.clocks.at(arrival.key);
          if (preserve_window(arrival.key)
              && (near_bound(arrival.latency, current.early)
                  || near_bound(arrival.latency, current.late))) {
            mark_clock_arrival(arrival, projection_frozen);
          }
        }
        expand_blocked(projection_frozen);
        for (int n : projection_frozen) {
          if (nodes[n].eligible) {
            critical_frozen_total.insert(n);
          }
        }
        std::set<int> blocked;
        for (int revision = 0; revision < max_checkpoints; ++revision) {
          if (expired() || evaluations >= max_checkpoints - 1) {
            stop = expired() ? "max_seconds" : "max_checkpoints";
            break;
          }
          std::vector<sta::LibertyCellSeq> options(nodes.size());
          std::vector<size_t> choice(nodes.size(), 0);
          std::vector<sta::LibertyCell*> selected;
          int selection_count = 0;
          size_t max_options = 1;
          for (int n : order) {
            auto& node = nodes[n];
            node.cell = network_->libertyCell(db_network_->dbToSta(node.inst));
            if ((!max_trials || trials + selection_count < max_trials)
                && node.eligible && !blocked.count(n)
                && !projection_frozen.count(n)) {
              for (auto* candidate : compatible_choices[n]) {
                if (candidate == node.cell
                    || !available(node.cell, candidate)) {
                  continue;
                }
                auto* master = db_->findMaster(candidate->name().c_str());
                odb::Rect box;
                master->getPlacementBoundary(box);
                node.inst->getTransform().apply(box);
                if (contains(node.inst->getBBox()->getBox(), box)) {
                  options[n].push_back(candidate);
                }
              }
            }
            selection_count += !options[n].empty();
            // Original/current is the last fallback, never an untested larger
            // replacement. Local relaxation is only a proposal operation.
            options[n].push_back(node.cell);
            max_options = std::max(max_options, options[n].size());
          }
          for (size_t n = 0; n < nodes.size(); ++n) {
            selected.push_back(options[n].front());
          }
          struct ProjectionBase
          {
            float load;
            float input_slew;
            float output_slew;
            float delay;
            float model_slew;
          };
          std::vector<std::map<ProjectionKey, ProjectionBase>> bases(
              nodes.size());
          for (int n : order) {
            const auto& node = nodes[n];
            auto* input_vertex = graph_->pinLoadVertex(node.input);
            auto* output_vertex = graph_->pinDrvrVertex(node.output);
            if (!input_vertex || !output_vertex) {
              continue;
            }
            for (auto* scene : sta_->scenes()) {
              for (auto* mm : sta::MinMax::range()) {
                for (auto* rf : sta::RiseFall::range()) {
                  const float load = sta_->graphDelayCalc()->loadCap(
                      node.output, rf, scene, mm);
                  const float input_slew = sta::delayAsFloat(graph_->slew(
                      input_vertex, rf, scene->dcalcAnalysisPtIndex(mm)));
                  const float output_slew = sta::delayAsFloat(graph_->slew(
                      output_vertex, rf, scene->dcalcAnalysisPtIndex(mm)));
                  const auto old
                      = model(node.cell, load, input_slew, scene, mm, rf);
                  bases[n][{scene, mm->index(), rf->index()}]
                      = {load, input_slew, output_slew, old.first, old.second};
                }
              }
            }
          }
          // Local rounds advance only toward a larger compatible candidate.
          // All selected child capacitances and forward arrival/slew effects
          // are recomputed together; no design edit, STA update or native
          // power query occurs during these bounded surrogate rounds.
          const size_t max_projection_rounds = 2 * max_options + 2;
          for (size_t projection_round = 0;
               projection_round < max_projection_rounds && !expired();
               ++projection_round) {
            ++projection_rounds;
            std::vector<std::map<ProjectionKey, Delta>> deltas(nodes.size());
            std::vector<bool> path_changed(nodes.size(), false);
            std::set<int> relax, freeze_early;
            auto relax_ancestry = [&](int seed) {
              bool changed_ancestor = false;
              std::set<int> seen;
              for (int n = seed; n >= 0 && seen.insert(n).second;
                   n = nodes[n].parent) {
                if (selected[n] != nodes[n].cell) {
                  relax.insert(n);
                  changed_ancestor = true;
                }
              }
              return changed_ancestor;
            };
            for (auto it = order.rbegin(); it != order.rend(); ++it) {
              const int n = *it;
              const auto& node = nodes[n];
              path_changed[n]
                  = selected[n] != node.cell
                    || (node.parent >= 0 && path_changed[node.parent]);
              for (int child : node.children) {
                path_changed[n]
                    = path_changed[n] || selected[child] != nodes[child].cell;
              }
              for (auto* scene : sta_->scenes()) {
                for (auto* mm : sta::MinMax::range()) {
                  for (auto* rf : sta::RiseFall::range()) {
                    ProjectionKey key{scene, mm->index(), rf->index()};
                    auto base_it = bases[n].find(key);
                    if (base_it == bases[n].end()) {
                      continue;
                    }
                    const auto& base = base_it->second;
                    Delta parent{0, 0};
                    if (node.parent >= 0) {
                      parent = deltas[node.parent][key];
                    }
                    float new_load = base.load;
                    for (int child : node.children) {
                      new_load += ports(selected[child], scene, mm)
                                      .first->capacitance(rf, mm)
                                  - ports(nodes[child].cell, scene, mm)
                                        .first->capacitance(rf, mm);
                    }
                    auto replacement
                        = std::make_pair(base.delay, base.model_slew);
                    if (selected[n] != node.cell || new_load != base.load
                        || parent.second != 0) {
                      replacement = model(
                          selected[n],
                          new_load,
                          std::max(0.0f, base.input_slew + parent.second),
                          scene,
                          mm,
                          rf);
                    }
                    deltas[n][key]
                        = {parent.first + replacement.first - base.delay,
                           replacement.second - base.model_slew};
                    const auto original_ports = ports(node.cell, scene, mm);
                    const auto selected_ports = ports(selected[n], scene, mm);
                    bool weak = false;
                    auto check_limit = [&](float actual,
                                           float starting,
                                           float limit,
                                           bool exists,
                                           float original_limit,
                                           bool original_exists,
                                           float absolute_tolerance) {
                      if (!exists) {
                        return;
                      }
                      const float floor
                          = original_exists
                                ? std::min(0.0f,
                                           mm == sta::MinMax::max()
                                               ? original_limit - starting
                                               : starting - original_limit)
                                : 0;
                      const float margin = mm == sta::MinMax::max()
                                               ? limit - actual
                                               : actual - limit;
                      weak |= !std::isfinite(margin)
                              || margin
                                     < floor
                                           - std::max(absolute_tolerance,
                                                      std::abs(floor) * 1e-5f);
                    };
                    float limit, old_limit;
                    bool exists, old_exists;
                    selected_ports.second->capacitanceLimit(mm, limit, exists);
                    original_ports.second->capacitanceLimit(
                        mm, old_limit, old_exists);
                    check_limit(new_load,
                                base.load,
                                limit,
                                exists,
                                old_limit,
                                old_exists,
                                1e-21f);
                    auto check_slew =
                        [&](const sta::LibertyPort* candidate_port,
                            const sta::LibertyPort* original_port,
                            float actual,
                            float starting) {
                          candidate_port->slewLimit(mm, limit, exists);
                          original_port->slewLimit(mm, old_limit, old_exists);
                          if (mm == sta::MinMax::max()) {
                            if (!exists
                                && candidate_port->direction()->isAnyOutput()) {
                              candidate_port->libertyLibrary()->defaultMaxSlew(
                                  limit, exists);
                            }
                            if (!old_exists
                                && original_port->direction()->isAnyOutput()) {
                              original_port->libertyLibrary()->defaultMaxSlew(
                                  old_limit, old_exists);
                            }
                          }
                          check_limit(actual,
                                      starting,
                                      limit,
                                      exists,
                                      old_limit,
                                      old_exists,
                                      1e-15f);
                        };
                    check_slew(selected_ports.first,
                               original_ports.first,
                               std::max(0.0f, base.input_slew + parent.second),
                               base.input_slew);
                    check_slew(
                        selected_ports.second,
                        original_ports.second,
                        std::max(0.0f,
                                 base.output_slew + deltas[n][key].second),
                        base.output_slew);
                    if (weak) {
                      if (selected[n] != node.cell) {
                        relax.insert(n);
                      } else {
                        // A frozen anchor's waveform can change through its
                        // modeled parent or child loads. Relax those causes.
                        relax_ancestry(node.parent);
                        for (int child : node.children) {
                          if (selected[child] != nodes[child].cell) {
                            relax.insert(child);
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
            struct ProjectedSink
            {
              const SinkArrival* original;
              int parent;
              float latency;
            };
            std::vector<ProjectedSink> sinks;
            std::map<ClockKey, Envelope> projected;
            std::map<ClockKey, std::vector<int>> early_parents;
            // First finish every group's projected extrema. Using the old
            // early arrival here falsely consumes headroom when all paths move.
            for (const auto& arrival : certified.sink_arrivals) {
              int parent = -1;
              auto parent_it = sink_parents.find(arrival.pin);
              if (parent_it != sink_parents.end()) {
                parent = parent_it->second;
              }
              const float delta
                  = parent < 0 ? 0
                               : deltas[parent][{std::get<1>(arrival.key),
                                                 arrival.min_max,
                                                 std::get<3>(arrival.key)}]
                                     .first;
              const float latency = arrival.latency + delta;
              auto& envelope = projected[arrival.key];
              if (latency < envelope.early) {
                early_parents[arrival.key].clear();
                envelope.early = latency;
              }
              if (latency == envelope.early) {
                early_parents[arrival.key].push_back(parent);
              }
              envelope.late = std::max(envelope.late, latency);
              sinks.push_back({&arrival, parent, latency});
            }
            std::set<ClockKey> relaxed_early_groups;
            for (const auto& sink : sinks) {
              const auto& arrival = *sink.original;
              const auto& initial = baseline.clocks.at(arrival.key);
              const auto& group = projected.at(arrival.key);
              const auto [latency_limit, skew_limit]
                  = clock_limits(arrival.key, intermediate_budget);
              const bool changed_path
                  = sink.parent >= 0 && path_changed[sink.parent];
              const bool preserving = preserve_window(arrival.key);
              const float guardband
                  = preserving && changed_path
                        ? std::max(10e-12f,
                                   0.01f * (initial.late - initial.early))
                        : 0;
              const float upper
                  = std::min(latency_limit, group.early + skew_limit)
                    - guardband;
              if (preserving && changed_path
                  && below(sink.latency, group.late - skew_limit + guardband)) {
                // Load relief can accelerate an early path. Increasing only
                // its driver is not a monotonic remedy: freeze its Cin cone.
                if (sink.parent >= 0) {
                  freeze_early.insert(sink.parent);
                }
              }
              if (below(upper, sink.latency)) {
                // Only this late path's changed ancestors advance. Siblings
                // retain their proposals and participate in the next
                // projection.
                const bool changed_ancestor = relax_ancestry(sink.parent);
                if (!changed_ancestor
                    && below(group.early + skew_limit, sink.latency)) {
                  // The late path can be unchanged while another path became
                  // earlier through load reduction. Relax that early cause.
                  if (relaxed_early_groups.insert(arrival.key).second) {
                    for (int parent : early_parents.at(arrival.key)) {
                      relax_ancestry(parent);
                    }
                  }
                }
              }
            }
            bool advanced = false;
            expand_blocked(freeze_early);
            for (int n : freeze_early) {
              projection_frozen.insert(n);
              if (nodes[n].eligible) {
                early_frozen_total.insert(n);
              }
              if (selected[n] != nodes[n].cell) {
                choice[n] = options[n].size() - 1;
                selected[n] = nodes[n].cell;
                advanced = true;
              }
            }
            for (int n : relax) {
              if (choice[n] + 1 < options[n].size()) {
                selected[n] = options[n][++choice[n]];
                ++candidate_relaxations;
                advanced = true;
              }
            }
            if (!advanced) {
              break;
            }
          }
          std::vector<Move> moves;
          for (int n : order) {
            if (selected[n] != nodes[n].cell) {
              if (max_trials
                  && trials + static_cast<int>(moves.size()) >= max_trials) {
                break;
              }
              moves.push_back({n, selected[n]});
            }
          }
          proposed += moves.size();
          if (moves.empty()) {
            stop = max_trials && trials >= max_trials ? "max_trials"
                                                      : "no_improving_batch";
            break;
          }
          // The deadline may expire during a large proposal. Do not start a
          // native checkpoint then; final validation remains reserved.
          if (expired()) {
            stop = "max_seconds";
            break;
          }
          // Rank once using the command-start native instance powers. These
          // cached powers are proposal weights, never a replacement for the
          // outer native objective. No instance power read follows an edit.
          auto score = [&](const Move& move) {
            const auto& node = nodes[move.node];
            const auto* master = node.inst->getMaster();
            const auto* replacement
                = db_->findMaster(move.cell->name().c_str());
            const double area_fraction
                = 1.0 - double(replacement->getWidth()) / master->getWidth();
            const double power = baseline_instance_power[move.node];
            sta::LibertyPort *old_input, *old_output, *new_input, *new_output;
            node.cell->bufferPorts(old_input, old_output);
            move.cell->bufferPorts(new_input, new_output);
            const double cap_fraction
                = old_input->capacitance() > 0
                      ? std::max(0.0,
                                 1.0
                                     - double(new_input->capacitance())
                                           / old_input->capacitance())
                      : 0;
            return std::max(0.0, power) * (area_fraction + 0.1 * cap_fraction);
          };
          std::stable_sort(
              moves.begin(), moves.end(), [&](const Move& a, const Move& b) {
                const double as = score(a), bs = score(b);
                if (as != bs) {
                  return as > bs;
                }
                if (depth[a.node] != depth[b.node]) {
                  return depth[a.node] > depth[b.node];
                }
                return std::string(nodes[a.node].inst->getName())
                       < nodes[b.node].inst->getName();
              });
          std::vector<Original> originals;
          bool prefix_final_feasible = feasible_clock_slack(certified, false);
          const size_t accepted_before = accepted.size();
          journalBegin();
          journal_open = true;
          std::set<int> tentative_nodes;
          bool scan_stopped = false;
          constexpr size_t max_tentative_accepts = 512;
          constexpr size_t max_tentative_sizes = 3;
          // A second cheap scan allows a parent rejected before child load
          // relief to be reconsidered. Full native snapshots occur only
          // after these inner journals have settled into the outer journal.
          for (int scan = 0; scan < 2 && !scan_stopped; ++scan) {
            for (const auto& move : moves) {
              if (tentative_nodes.count(move.node)) {
                continue;
              }
              if (tentative_nodes.size() >= max_tentative_accepts) {
                scan_stopped = true;
                break;
              }
              auto& node = nodes[move.node];
              size_t candidate_index = choice[move.node];
              auto fallback_risk = [&](sta::LibertyCell* candidate) {
                double risk = 0;
                for (auto* scene : sta_->scenes()) {
                  for (auto* mm : sta::MinMax::range()) {
                    for (auto* rf : sta::RiseFall::range()) {
                      const auto base = bases[move.node].find(
                          {scene, mm->index(), rf->index()});
                      if (base == bases[move.node].end()) {
                        continue;
                      }
                      const auto& b = base->second;
                      const auto candidate_model = model(
                          candidate, b.load, b.input_slew, scene, mm, rf);
                      const double delay_penalty
                          = std::max(0.0,
                                     double(candidate_model.first - b.delay))
                            / std::max(1e-12, std::abs(double(b.delay)));
                      const double slew_penalty
                          = std::max(
                                0.0,
                                double(candidate_model.second - b.model_slew))
                            / std::max(1e-12, std::abs(double(b.model_slew)));
                      risk = std::max(risk, delay_penalty + slew_penalty);
                    }
                  }
                }
                return risk;
              };
              for (size_t attempt = 0;
                   attempt < max_tentative_sizes
                   && candidate_index + 1 < options[move.node].size();
                   ++attempt, ++candidate_index) {
                if (max_trials && trials >= max_trials) {
                  stop = "max_trials";
                  wave_budget_stop = scan_stopped = true;
                  break;
                }
                if (!time_for_tentative()) {
                  stop = "max_seconds";
                  wave_budget_stop = scan_stopped = true;
                  break;
                }
                if (attempt + 1 == max_tentative_sizes) {
                  double best_risk = std::numeric_limits<double>::infinity();
                  size_t best = candidate_index;
                  for (size_t alternative = candidate_index;
                       alternative + 1 < options[move.node].size();
                       ++alternative) {
                    const double risk
                        = fallback_risk(options[move.node][alternative]);
                    if (risk < best_risk) {
                      best_risk = risk;
                      best = alternative;
                    }
                  }
                  candidate_index = best;
                  ++strong_fallbacks;
                }
                auto* candidate = options[move.node][candidate_index];
                auto* replacement = db_->findMaster(candidate->name().c_str());
                auto* master = node.inst->getMaster();
                const Original original{
                    node.inst, node.cell, node.inst->getOrigin()};
                // Keep the pending inner move reachable even if timing throws
                // after its ECO was already undone and ownership was released.
                const size_t inner_before = accepted.size();
                accepted.push_back(original);
                journalBegin();
                inner_journal_open = true;
                ++trials;
                designAreaIncr(-area(master));
                sta_->replaceCell(db_network_->dbToSta(node.inst),
                                  db_network_->dbToSta(replacement));
                if (node.inst->getMaster() != replacement
                    || node.inst->getOrigin() != original.origin) {
                  throw std::runtime_error(
                      "tentative clock buffer replacement failed");
                }
                designAreaIncr(area(replacement));
                const auto wns_start = std::chrono::steady_clock::now();
                refresh_timing();
                ++tentative_checks;
                bool wns_feasible = true, final_wns_feasible = true;
                for (auto* scene : sta_->scenes()) {
                  for (auto* mm : sta::MinMax::range()) {
                    sta::Slack slack;
                    sta::Vertex* endpoint = nullptr;
                    sta_->worstSlack(scene, mm, slack, endpoint);
                    ++slack_queries;
                    const float value = sta::delayAsFloat(slack);
                    const bool valid
                        = endpoint && std::isfinite(value) && value < sta::INF;
                    auto initial = baseline.slacks.find({scene, mm->index()});
                    if (initial == baseline.slacks.end()) {
                      wns_feasible &= !valid;
                      final_wns_feasible &= !valid;
                    } else {
                      const float floor = slack_floor({scene, mm->index()},
                                                      intermediate_budget);
                      wns_feasible &= valid && !below(value, floor);
                      final_wns_feasible
                          &= valid
                             && !below(value,
                                       slack_floor({scene, mm->index()},
                                                   final_budget));
                    }
                  }
                }
                tentative_wns_seconds
                    += std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - wns_start)
                           .count();
                bool electrical_feasible = false;
                if (wns_feasible) {
                  ++local_electrical_checks;
                  const auto electrical_start
                      = std::chrono::steady_clock::now();
                  electrical_feasible = local_electrical_feasible(node);
                  local_electrical_seconds
                      += std::chrono::duration<double>(
                             std::chrono::steady_clock::now()
                             - electrical_start)
                             .count();
                  local_electrical_rejects += !electrical_feasible;
                }
                std::pair<bool, bool> clock_feasibility{false, false};
                if (wns_feasible && electrical_feasible) {
                  // WNS has settled all arrivals. Read only cached clock sink
                  // paths here; ERC and power remain outer checkpoint guards.
                  ++cached_clock_checks;
                  const auto clock_start = std::chrono::steady_clock::now();
                  clock_feasibility = cached_clocks_feasible();
                  cached_clock_seconds
                      += std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - clock_start)
                             .count();
                  cached_clock_rejects += !clock_feasibility.first;
                }
                if (wns_feasible && clock_feasibility.first
                    && electrical_feasible) {
                  const bool final_timing_feasible
                      = final_wns_feasible && clock_feasibility.second;
                  if (prefix_final_feasible && !final_timing_feasible
                      && !originals.empty()) {
                    // Certify the useful prefix before exploring beyond the
                    // final window. The deferred move can be tried next wave.
                    ++boundary_deferrals;
                    restore_inner();
                    accepted.resize(inner_before);
                    scan_stopped = true;
                    break;
                  }
                  prefix_final_feasible = final_timing_feasible;
                  // Committing this inner journal merges the tentative move
                  // into the still-uncertified batch.
                  odb::dbDatabase::commitEco(block_);
                  inner_journal_open = false;
                  RepairSetupProfiler::journalEnd(false);
                  originals.push_back(original);
                  tentative_nodes.insert(move.node);
                  ++tentative_accepts;
                  break;
                }
                ++tentative_rejects;
                restore_inner();
                accepted.resize(inner_before);
              }
              if (scan_stopped) {
                break;
              }
            }
          }
          if (originals.empty()) {
            restore_batch();
            accepted.resize(accepted_before);
            if (!wave_budget_stop) {
              stop = "no_improving_batch";
            }
            break;
          }
          ++batch_attempts;
          // The last accepted inner trial has already settled incremental STA.
          const Snapshot trial = snapshot(
              [&](const Snapshot& state) {
                return feasible_clock_slack(state, false, intermediate_budget);
              },
              feasible_electrical);
          if (verbose) {
            logger_->report(
                "recover_clock_power batch checkpoint: validation {}, native "
                "power calls {}, committed batches {}, replacements {}, power "
                "computed {}, power {:.9g} W.",
                evaluations,
                power_calls,
                accepted_batches,
                originals.size(),
                trial.power_computed,
                trial.power);
          }
          const bool improvement
              = trial.power_computed && std::isfinite(trial.power)
                && trial.power
                       < best.power
                             - std::max(1e-20f, std::abs(best.power) * 1e-6f);
          if (feasible(trial, intermediate_budget)) {
            // The ordinary journalEnd() also refreshes timing. This batch is
            // already settled: commit only the journal and close its profiler
            // frame, preserving the exact precommit native certificate.
            {
              const ProfileScope scope("journal");
              odb::dbDatabase::commitEco(block_);
              journal_open = false;
              RepairSetupProfiler::journalEnd(false);
            }
            committed += originals.size();
            ++accepted_batches;
            certified = trial;
            current_power = trial.power;
            if (improvement && feasible_clock_slack(trial, false)) {
              best = trial;
              best_boundary = accepted.size();
              ++best_updates;
              for (auto& state : master_states) {
                state.best = state.inst->getMaster();
              }
            }
            progress = true;
            break;
          }
          if (verbose) {
            feasible_clock_slack(trial, true, intermediate_budget);
            report_slacks("rejected", trial);
          }
          const auto diagnosed = diagnose(trial);
          const size_t blocked_before = blocked.size();
          blocked.insert(diagnosed.begin(), diagnosed.end());
          // Unsupported/nonlocal diagnostics get one deterministic, globally
          // less aggressive revision, never recursive single-cell validation.
          if (blocked.size() == blocked_before) {
            for (size_t i = 0; i < originals.size(); i += 2) {
              auto node
                  = node_by_inst.find(db_network_->dbToSta(originals[i].inst));
              if (node != node_by_inst.end()) {
                blocked.insert(node->second);
              }
            }
          }
          restore_batch();
          for (const auto& original : originals) {
            if (network_->libertyCell(db_network_->dbToSta(original.inst))
                    != original.cell
                || original.inst->getOrigin() != original.origin) {
              throw std::runtime_error(
                  "batch rollback failed structural verification");
            }
          }
          accepted.resize(accepted_before);
          stop = "no_improving_batch";
        }
        if (!progress || wave_budget_stop) {
          break;
        }
        if (wave + 1 == max_passes) {
          stop = "max_passes";
        }
      }
      // Always reserve and consume the final native certificate, including
      // baseline-only deadline/checkpoint exits. Native calls are
      // uninterruptible.
      // Keep the entire history until certification succeeds. If restoring
      // the best suffix or refreshing timing throws, restore_all can still
      // reverse every invocation edit without consuming a caller's ECO.
      for (size_t i = accepted.size(); i > best_boundary; --i) {
        const auto& original = accepted[i - 1];
        sta_->replaceCell(db_network_->dbToSta(original.inst),
                          db_network_->dbToSta(
                              db_->findMaster(original.cell->name().c_str())));
        original.inst->setOrigin(original.origin.x(), original.origin.y());
      }
      if (accepted.size() > best_boundary) {
        invalidateVertexOrdering();
        initDesignArea();
      }
      for (const auto& state : master_states) {
        if (state.inst->getMaster() != state.best) {
          throw std::runtime_error("batch best restoration failed");
        }
        if (state.best != state.initial) {
          changed.insert(state.inst->getId());
        }
      }
      refresh_timing();
      const auto final = snapshot();
      if (!feasible(final)
          || std::abs(final.power - best.power)
                 > std::max(1e-20f, std::abs(best.power) * 1e-5f)
          || (!changed.empty() && !(final.power < baseline.power))) {
        throw std::runtime_error("batch final feasibility/power check failed");
      }
      current_power = final.power;
      report_slacks("final", final);
    } catch (...) {
      // Close only our ECO. Pending originals remain available if a settled
      // timing update throws after undo, so restore_all needs no further undo.
      if (inner_journal_open) {
        try {
          restore_inner();
        } catch (...) {
          // The pending move remains in invocation structural restoration.
        }
      }
      if (journal_open) {
        try {
          restore_batch();
        } catch (...) {
          // Complete structural invocation restoration below before
          // propagating.
        }
      }
      restore_all();
      throw;
    }
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
    logger_->report(
        "recover_clock_power: initial {:.9g} W, final {:.9g} W; changed {}, "
        "trials {}, STA evaluations {}, elapsed {:.3f}s, stop {}.",
        baseline.power,
        current_power,
        changed.size(),
        trials,
        evaluations,
        elapsed,
        stop);
    logger_->report(
        "recover_clock_power batch summary: attempts {}, accepted {}, "
        "full validations {}, native power calls {}, proposed {}, "
        "committed {}, excluded {}, timing skipped power {}, electrical "
        "skipped power {}, projection rounds {}, candidate relaxations {}, "
        "tentative checks {}, accepts {}, rejects {}, WNS seconds {:.6f}, "
        "baseline instance power reads {}, critically frozen {}, early frozen "
        "{}, "
        "strong fallbacks {}, cached clock checks {}, rejections {}, seconds "
        "{:.6f}, local electrical checks {}, rejections {}, seconds {:.6f}, "
        "watched slew pins {}, exploratory batches {}, best updates {}, "
        "best history {}, restored suffix {}, boundary deferrals {}.",
        batch_attempts,
        accepted_batches,
        evaluations,
        power_calls,
        proposed,
        committed,
        excluded,
        skipped_power_trials,
        skipped_electrical_power_trials,
        projection_rounds,
        candidate_relaxations,
        tentative_checks,
        tentative_accepts,
        tentative_rejects,
        tentative_wns_seconds,
        baseline_instance_power_reads,
        critical_frozen_total.size(),
        early_frozen_total.size(),
        strong_fallbacks,
        cached_clock_checks,
        cached_clock_rejects,
        cached_clock_seconds,
        local_electrical_checks,
        local_electrical_rejects,
        local_electrical_seconds,
        watched_slew_pins.size(),
        accepted_batches,
        best_updates,
        best_boundary,
        accepted.size() - best_boundary,
        boundary_deferrals);
    report_profile();
    initDesignArea();
    return changed.size();
  }
  try {
    bool trial_limit_reached = false;
    for (int pass = 0; pass < max_passes; ++pass) {
      bool progress = false;
      for (const auto& buffer : buffers) {
        auto* inst = buffer.inst;
        auto* si = db_network_->dbToSta(inst);
        auto* original_cell = network_->libertyCell(si);
        auto* master = inst->getMaster();
        const odb::Rect footprint = inst->getBBox()->getBox();
        for (auto* candidate : candidates) {
          auto* replacement = db_->findMaster(candidate->name().c_str());
          auto* site = master->getSite();
          bool available
              = replacement && candidate->isBuffer() && !dontUse(candidate)
                && replacement->isCore() && site && site->getWidth() > 0
                && replacement->getSite() == site && replacement->getWidth() > 0
                && replacement->getHeight() > 0
                && replacement->getWidth() % site->getWidth() == 0
                && replacement->getHeight() == master->getHeight()
                && replacement->getWidth() < master->getWidth()
                && (!master->getSymmetryX() || replacement->getSymmetryX())
                && (!master->getSymmetryY() || replacement->getSymmetryY())
                && (!master->getSymmetryR90() || replacement->getSymmetryR90())
                && sta::equivCellPorts(original_cell, candidate);
          if (available) {
            available
                = master->getMTerms().size() == replacement->getMTerms().size();
            for (auto* term : master->getMTerms()) {
              auto* other = replacement->findMTerm(term->getName().c_str());
              available &= other && other->getIoType() == term->getIoType()
                           && other->getSigType() == term->getSigType();
            }
          }
          for (auto* scene : sta_->scenes()) {
            for (auto* mm : sta::MinMax::range()) {
              auto* original_model = scene_model(original_cell, scene, mm);
              auto* candidate_model = scene_model(candidate, scene, mm);
              available
                  &= original_model && candidate_model
                     && sta::equivCellPorts(original_model, candidate_model)
                     && sta::equivCellFuncs(original_model, candidate_model);
            }
          }
          if (!available) {
            continue;
          }
          odb::Rect box;
          replacement->getPlacementBoundary(box);
          inst->getTransform().apply(box);
          if (!contains(footprint, box)) {
            continue;
          }
          if (max_trials > 0 && trials >= max_trials) {
            trial_limit_reached = true;
            stop = "max_trials";
            break;
          }
          ++trials;
          const auto origin = inst->getOrigin();
          journalBegin();
          journal_open = true;
          designAreaIncr(-area(master));
          sta_->replaceCell(si, db_network_->dbToSta(replacement));
          if (inst->getMaster() != replacement) {
            throw std::runtime_error("clock buffer master replacement failed");
          }
          designAreaIncr(area(replacement));
          refresh_timing();
          Snapshot trial = snapshot([&](const Snapshot& timing) {
            return feasible_clock_slack(timing, false);
          });
          if (verbose) {
            if (trial.timing_rejected) {
              logger_->report(
                  "recover_clock_power: trial {} {}, power not evaluated "
                  "(clock/slack rejection).",
                  inst->getName(),
                  candidate->name());
              feasible_clock_slack(trial, true);
            } else {
              logger_->report(
                  "recover_clock_power: trial {} {}, power {:.9g} -> {:.9g} W.",
                  inst->getName(),
                  candidate->name(),
                  current_power,
                  trial.power);
            }
          }
          if (!trial.timing_rejected && trial.power < current_power
              && feasible(trial)
              && contains(footprint, inst->getBBox()->getBox())) {
            journalEnd();
            journal_open = false;
            refresh_timing();
            trial = snapshot();
            accepted.push_back({inst, original_cell, origin});
            if (!feasible(trial) || !(trial.power < current_power)) {
              throw std::runtime_error(
                  "post-commit feasibility/power check failed");
            }
            current_power = trial.power;
            changed.insert(inst->getId());
            progress = true;
            if (verbose) {
              logger_->report(
                  "recover_clock_power: {} {} -> {}, clock power {:.9g} W",
                  inst->getName(),
                  original_cell->name(),
                  candidate->name(),
                  current_power);
            }
            break;
          }
          journalRestore();
          journal_open = false;
          initDesignArea();
          refresh_timing();
          const auto restored = snapshot();
          if (!feasible(restored)
              || std::abs(restored.power - current_power)
                     > std::max(1e-20f, std::abs(current_power) * 1e-5f)) {
            throw std::runtime_error("trial rollback failed verification");
          }
        }
        if (trial_limit_reached) {
          break;
        }
      }
      if (trial_limit_reached) {
        break;
      }
      if (!progress) {
        stop = "no_improving_move";
        break;
      }
    }
    refresh_timing();
    const auto final = snapshot();
    if (!feasible(final)
        || (changed.size() && !(final.power < baseline.power))) {
      throw std::runtime_error("final feasibility/power check failed");
    }
    current_power = final.power;
  } catch (...) {
    restore_all();
    throw;
  }
  const double elapsed
      = std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  logger_->report(
      "recover_clock_power: initial {:.9g} W, final {:.9g} W; changed {}, "
      "trials {}, STA evaluations {}, elapsed {:.3f}s, stop {}.",
      baseline.power,
      current_power,
      changed.size(),
      trials,
      evaluations,
      elapsed,
      stop);
  report_profile();
  initDesignArea();
  return changed.size();
}
}  // namespace rsz
