// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018-2025, The OpenROAD Authors

#include "timingBase.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "grt/GlobalRouter.h"
#include "nesterovBase.h"
#include "placerBase.h"
#include "rsz/Resizer.hh"
#include "sta/Fuzzy.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace gpl {

using utl::GPL;

// TimingBase
TimingBase::TimingBase() = default;

TimingBase::TimingBase(std::shared_ptr<NesterovBaseCommon> nbc,
                       grt::GlobalRouter* grt,
                       rsz::Resizer* rs,
                       utl::Logger* log)
    : TimingBase()
{
  grt_ = grt;
  rs_ = rs;
  nbc_ = std::move(nbc);
  log_ = log;
}

void TimingBase::initTimingOverflowChk()
{
  timingOverflowChk_.clear();
  timingOverflowChk_.resize(timingNetWeightOverflow_.size(), false);
}

bool TimingBase::isTimingNetWeightOverflow(float overflow)
{
  int intOverflow = std::round(overflow * 100);
  // exception case handling
  if (timingNetWeightOverflow_.empty()
      || intOverflow > timingNetWeightOverflow_[0]) {
    return false;
  }

  bool needTdRun = false;
  for (int i = 0; i < timingNetWeightOverflow_.size(); i++) {
    if (timingNetWeightOverflow_[i] > intOverflow) {
      if (!timingOverflowChk_[i]) {
        timingOverflowChk_[i] = true;
        needTdRun = true;
      }
      continue;
    }
    return needTdRun;
  }
  return needTdRun;
}

void TimingBase::addTimingNetWeightOverflow(int overflow)
{
  std::vector<int>::iterator it
      = std::ranges::find(timingNetWeightOverflow_, overflow);

  // only push overflow when the overflow is not in vector.
  if (it == timingNetWeightOverflow_.end()) {
    timingNetWeightOverflow_.push_back(overflow);
  }

  // do sort in reverse order
  std::ranges::sort(timingNetWeightOverflow_, std::greater<int>());
}

void TimingBase::setTimingNetWeightOverflows(const std::vector<int>& overflows)
{
  // sort by decreasing order
  auto sorted = overflows;
  std::ranges::sort(sorted, std::greater<int>());
  for (auto& overflow : sorted) {
    addTimingNetWeightOverflow(overflow);
  }
  initTimingOverflowChk();
}

void TimingBase::deleteTimingNetWeightOverflow(int overflow)
{
  std::vector<int>::iterator it
      = std::ranges::find(timingNetWeightOverflow_, overflow);
  // only erase overflow when the overflow is in vector.
  if (it != timingNetWeightOverflow_.end()) {
    timingNetWeightOverflow_.erase(it);
  }
}

void TimingBase::clearTimingNetWeightOverflow()
{
  timingNetWeightOverflow_.clear();
}

size_t TimingBase::getTimingNetWeightOverflowSize() const
{
  return timingNetWeightOverflow_.size();
}

void TimingBase::setTimingNetWeightMax(float max)
{
  net_weight_max_ = max;
}

void TimingBase::setTimingNetsPercentage(float percentage)
{
  nets_percentage_ = percentage;
}

void TimingBase::setTimingDrivenUseRepairSetup(bool use_repair_setup)
{
  timing_driven_use_repair_setup_ = use_repair_setup;
}

void TimingBase::setVerbose(bool verbose)
{
  verbose_ = verbose;
}

void TimingBase::setTimingDrivenPinBased(bool pin_based)
{
  pin_based_ = pin_based;
}

bool TimingBase::executeTimingDriven(bool run_journal_restore,
                                     bool enable_repair_timing)
{
  rs_->findResizeSlacks(run_journal_restore,
                        verbose_,
                        timing_driven_use_repair_setup_,
                        (enable_repair_timing && repair_timing_),
                        repair_tns_end_percent_);

  if (!run_journal_restore) {
    nbc_->fixPointers();
  }

  rs_->setWorstSlackNetsPercent(nets_percentage_);
  if (pin_based_) {
    sta::PinSeq worst_slack_pins = rs_->resizeWorstSlackPins();

    if (worst_slack_pins.empty()) {
      log_->warn(
          GPL,
          8880,
          "Timing-driven: no pin slacks found. Timing-driven mode disabled.");
      return false;
    }

    // min/max slack for worst pins
    auto slack_min = rs_->resizePinSlack(worst_slack_pins[0]).value();
    auto slack_max
        = rs_->resizePinSlack(worst_slack_pins[worst_slack_pins.size() - 1])
              .value();

    log_->info(GPL, 8881, "Timing-driven: worst slack {}", slack_min);

    if (sta::fuzzyInf(slack_min)) {
      log_->warn(
          GPL,
          8882,
          "Timing-driven: no slacks found. Timing-driven mode disabled.");
      return false;
    }

    // Default weight
    for (auto& gNet : nbc_->getGNets()) {
      gNet->setTimingWeight(1.0);
      for (auto& gPin : gNet->getGPins()) {
        gPin->setTimingWeight(1.0);
      }
    }

    int weighted_pin_count = 0;
    for (auto& gPin : nbc_->getGPins()) {
      sta::Pin* sta_pin
          = rs_->dbNetwork()->dbToSta(gPin->getPbPin()->getDbITerm());
      auto pin_slack_opt = rs_->resizePinSlack(sta_pin);
      if (!pin_slack_opt) {
        continue;
      }
      auto pin_slack = pin_slack_opt.value();
      if (pin_slack < slack_max) {
        if (slack_max == slack_min) {
          gPin->setTimingWeight(1.0);
        } else {
          // weight(min_slack) = net_weight_max_
          // weight(max_slack) = 1
          const float weight = 1
                               + (net_weight_max_ - 1) * (slack_max - pin_slack)
                                     / (slack_max - slack_min);
          gPin->setTimingWeight(weight);
        }
        weighted_pin_count++;
      }
      debugPrint(log_,
                 GPL,
                 "timing",
                 1,
                 "pin:{} slack:{} weight:{}",
                 gPin->getPbPin()->getDbITerm()->getName(),
                 pin_slack,
                 gPin->getTotalWeight());
    }

    log_->info(
        GPL, 8883, "Timing-driven: weighted {} pins.", weighted_pin_count);

    return true;
  }
  // get worst resize nets
  sta::NetSeq worst_slack_nets = rs_->resizeWorstSlackNets();

  if (worst_slack_nets.empty()) {
    log_->warn(
        GPL,
        105,
        "Timing-driven: no net slacks found. Timing-driven mode disabled.");
    return false;
  }

  // min/max slack for worst nets
  auto slack_min = rs_->resizeNetSlack(worst_slack_nets[0]).value();
  auto slack_max
      = rs_->resizeNetSlack(worst_slack_nets[worst_slack_nets.size() - 1])
            .value();

  log_->info(GPL, 106, "Timing-driven: worst slack {}", slack_min);

  if (sta::fuzzyInf(slack_min)) {
    log_->warn(GPL,
               102,
               "Timing-driven: no slacks found. Timing-driven mode disabled.");
    return false;
  }

  int weighted_net_count = 0;
  for (auto& gNet : nbc_->getGNets()) {
    // default weight
    for (auto& gPin : gNet->getGPins()) {
      gPin->setTimingWeight(1.0);
    }
    gNet->setTimingWeight(1.0);
    if (gNet->getGPins().size() > 1) {
      auto net_slack_opt = rs_->resizeNetSlack(gNet->getPbNet()->getDbNet());
      if (!net_slack_opt) {
        continue;
      }
      auto net_slack = net_slack_opt.value();
      if (net_slack < slack_max) {
        if (slack_max == slack_min) {
          gNet->setTimingWeight(1.0);
        } else {
          // weight(min_slack) = net_weight_max_
          // weight(max_slack) = 1
          const float weight = 1
                               + (net_weight_max_ - 1) * (slack_max - net_slack)
                                     / (slack_max - slack_min);
          gNet->setTimingWeight(weight);
        }
        weighted_net_count++;
      }
      debugPrint(log_,
                 GPL,
                 "timing",
                 1,
                 "net:{} slack:{} weight:{}",
                 gNet->getPbNet()->getDbNet()->getConstName(),
                 net_slack,
                 gNet->getTotalWeight());
    }
  }

  debugPrint(log_,
             GPL,
             "timing",
             1,
             "Timing-driven: weighted {} nets.",
             weighted_net_count);
  return true;
}

}  // namespace gpl
