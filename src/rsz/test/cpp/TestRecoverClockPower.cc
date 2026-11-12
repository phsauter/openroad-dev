// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "RepairSetupProfiler.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "sta/ArcDelayCalc.hh"
#include "sta/Clock.hh"
#include "sta/FuncExpr.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Network.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/TableModel.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "tst/IntegratedFixture.h"

namespace rsz {
namespace {
class RecoverClockPowerTest : public tst::IntegratedFixture
{
 protected:
  RecoverClockPowerTest()
      : IntegratedFixture(Technology::kNangate45, "_main/src/rsz/test/")
  {
  }
  virtual const char* designFile() { return "TestRecoverClockPower.v"; }
  void SetUp() override
  {
    readVerilogAndSetup(designFile(), true, false);
    int x = 10000;
    block_->setDieArea(odb::Rect(0, 0, 200000, 200000));
    for (auto* inst : block_->getInsts()) {
      inst->setLocation(x, 10000);
      inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      x += 10000;
    }
    auto* sdc = sta_->cmdMode()->sdc();
    sta_->setPropagatedClock(sdc->findClock("clk"), sta_->cmdMode());
    sta_->setInputDelay(db_network_->findPin("data"),
                        sta::RiseFallBoth::riseFall(),
                        sdc->findClock("clk"),
                        sta::RiseFall::rise(),
                        nullptr,
                        false,
                        false,
                        sta::MinMaxAll::all(),
                        false,
                        0.1e-9,
                        sdc);
    ep_.estimateWireParasitics();
  }
  sta::LibertyLibrary* addFastScene(const char* omitted = nullptr)
  {
    // Replace the default scene with an explicitly modeled primary scene,
    // then add a second scene. The second creation retains the primary.
    sta_->makeScene("primary",
                    "default",
                    {"NangateOpenCellLibrary"},
                    {"NangateOpenCellLibrary"},
                    "",
                    "");
    sta_->makeScene("second", "default", {}, {}, "", "");
    auto* second = sta_->findScene("second");
    std::string path
        = getFilePath("_main/src/rsz/test/Nangate45/Nangate45_fast.lib");
    if (omitted) {
      std::ifstream input(path);
      std::string text((std::istreambuf_iterator<char>(input)), {});
      const std::string declaration
          = "\n  cell (" + std::string(omitted) + ") {";
      const auto offset = text.find(declaration);
      if (offset == std::string::npos) {
        throw std::runtime_error("test Liberty cell declaration not found");
      }
      text.replace(offset,
                   declaration.size(),
                   "\n  cell (unavailable_" + std::string(omitted) + ") {");
      path = std::string(std::getenv("TEST_TMPDIR")) + "/clock_recovery_"
             + omitted + ".lib";
      std::ofstream(path) << text;
    }
    return readLiberty(path, second);
  }
  float clockPower()
  {
    sta::PowerResult total, seq, comb, clock, macro, pad;
    sta_->power(sta_->cmdScene(), total, seq, comb, clock, macro, pad);
    return clock.total();
  }
  int recover(bool top_down = false, float latency = 1e-9)
  {
    return resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                      1e-9,
                                      latency,
                                      0,
                                      0,
                                      2,
                                      top_down,
                                      true);
  }
  void setCellDelay(sta::LibertyCell* cell, float delay)
  {
    for (auto* set : cell->timingArcSets()) {
      if (set->role()->isTimingCheck()) {
        continue;
      }
      for (auto* arc : set->arcs()) {
        *arc->gateTableModel()->delayModel()->table() = sta::Table(delay);
      }
    }
    sta_->delaysInvalid();
  }
  void setBufferDelay(const char* name, float delay)
  {
    setCellDelay(db_network_->findLibertyCell(name), delay);
  }
  int recoverBudget(float final_budget,
                    float intermediate_budget = -1,
                    int passes = 1,
                    int checkpoints = 6,
                    int trials = 0,
                    bool setup_given = false,
                    bool hold_given = false,
                    float setup_margin = 0,
                    float hold_margin = 0,
                    sta::LibertyCellSeq candidates = {},
                    float max_skew = -1,
                    float max_latency = -1)
  {
    if (candidates.empty()) {
      candidates.push_back(db_network_->findLibertyCell("BUF_X1"));
    }
    return resizer_.recoverClockPower(candidates,
                                      max_skew,
                                      max_latency,
                                      setup_margin,
                                      hold_margin,
                                      passes,
                                      false,
                                      true,
                                      trials,
                                      true,
                                      checkpoints,
                                      300,
                                      final_budget,
                                      intermediate_budget,
                                      setup_given,
                                      hold_given);
  }
  std::pair<float, float> reportedBudgets(const std::string& output)
  {
    float final_budget = NAN, intermediate_budget = NAN;
    const auto pos = output.find("recover_clock_power timing budgets:");
    if (pos != std::string::npos) {
      std::sscanf(
          output.c_str() + pos,
          "recover_clock_power timing budgets: final %f s, intermediate %f s",
          &final_budget,
          &intermediate_budget);
    }
    return {final_budget, intermediate_budget};
  }
  void checkSingleTrialBound(bool top_down)
  {
    auto* root = block_->findInst("clock_root");
    auto* leaf = block_->findInst("clock_leaf");
    const auto root_origin = root->getOrigin();
    const auto leaf_origin = leaf->getOrigin();
    auto* root_net = root->findITerm("Z")->getNet();
    auto* leaf_net = leaf->findITerm("Z")->getNet();
    const float before_power = clockPower();
    const float before_setup = sta_->worstSlack(sta::MinMax::max());
    const float before_hold = sta_->worstSlack(sta::MinMax::min());
    testing::internal::CaptureStdout();
    int changed = 0;
    EXPECT_NO_THROW(changed = resizer_.recoverClockPower(
                        {db_network_->findLibertyCell("BUF_X1")},
                        1e-9,
                        1e-9,
                        0,
                        0,
                        5,
                        top_down,
                        false,
                        1));
    const std::string output = testing::internal::GetCapturedStdout();
    EXPECT_NE(output.find("trials 1,"), std::string::npos);
    EXPECT_NE(output.find("stop max_trials."), std::string::npos);
    EXPECT_EQ(changed, 1);
    EXPECT_LT(clockPower(), before_power);
    EXPECT_GE(sta_->worstSlack(sta::MinMax::max()),
              std::min(0.0f, before_setup) - 1e-15f);
    EXPECT_GE(sta_->worstSlack(sta::MinMax::min()),
              std::min(0.0f, before_hold) - 1e-15f);
    EXPECT_EQ(root->getOrigin(), root_origin);
    EXPECT_EQ(leaf->getOrigin(), leaf_origin);
    EXPECT_EQ(root->findITerm("Z")->getNet(), root_net);
    EXPECT_EQ(leaf->findITerm("Z")->getNet(), leaf_net);
    EXPECT_EQ(std::string(root->getMaster()->getName()),
              top_down ? "BUF_X1" : "BUF_X16");
    EXPECT_EQ(std::string(leaf->getMaster()->getName()),
              top_down ? "BUF_X16" : "BUF_X1");
  }
};

TEST_F(RecoverClockPowerTest, DownsizingReducesActualClockPower)
{
  const float before = clockPower();
  auto* root = block_->findInst("clock_root");
  const auto origin = root->getOrigin();
  const auto inst_count = block_->getInsts().size();
  const auto net_count = block_->getNets().size();
  EXPECT_GT(recover(), 0);
  EXPECT_LT(clockPower(), before);
  EXPECT_EQ(block_->getInsts().size(), inst_count);
  EXPECT_EQ(block_->getNets().size(), net_count);
  EXPECT_EQ(root->getOrigin(), origin);
}

TEST_F(RecoverClockPowerTest, TopDownAndMirroredPlacement)
{
  auto* root = block_->findInst("clock_root");
  root->setOrient(odb::dbOrientType::MY);
  root->setLocation(10000, 10000);
  const auto box = root->getBBox()->getBox();
  EXPECT_GT(recover(true), 0);
  EXPECT_EQ(root->getOrient(), odb::dbOrientType::MY);
  const auto after = root->getBBox()->getBox();
  EXPECT_GE(after.xMin(), box.xMin());
  EXPECT_LE(after.xMax(), box.xMax());
}

TEST_F(RecoverClockPowerTest, InitialLimitRejectionPreservesExactDesign)
{
  auto* root = block_->findInst("clock_root");
  auto* leaf = block_->findInst("clock_leaf");
  const auto root_origin = root->getOrigin();
  const auto leaf_origin = leaf->getOrigin();
  auto* master = root->getMaster();
  EXPECT_ANY_THROW(recover(false, 0));
  EXPECT_EQ(root->getMaster(), master);
  EXPECT_EQ(leaf->getMaster(), master);
  EXPECT_EQ(root->getOrigin(), root_origin);
  EXPECT_EQ(leaf->getOrigin(), leaf_origin);
}

TEST_F(RecoverClockPowerTest, RejectedElectricalTrialsRollbackExactly)
{
  auto* root = block_->findInst("clock_root");
  auto* leaf = block_->findInst("clock_leaf");
  auto* master = root->getMaster();
  const auto root_origin = root->getOrigin();
  const auto leaf_origin = leaf->getOrigin();
  auto* input_net = root->findITerm("A")->getNet();
  auto* output_net = leaf->findITerm("Z")->getNet();
  // A replacement-specific cap limit forces actual ECO trials to fail even
  // though their area and power are attractive.
  sta_->setCapacitanceLimit(db_network_->dbToSta(db_->findMaster("BUF_X1")),
                            sta::MinMax::max(),
                            0,
                            sta_->cmdMode()->sdc());
  EXPECT_EQ(recover(), 0);
  EXPECT_EQ(root->getMaster(), master);
  EXPECT_EQ(leaf->getMaster(), master);
  EXPECT_EQ(root->getOrigin(), root_origin);
  EXPECT_EQ(leaf->getOrigin(), leaf_origin);
  EXPECT_EQ(root->findITerm("A")->getNet(), input_net);
  EXPECT_EQ(leaf->findITerm("Z")->getNet(), output_net);
}

TEST_F(RecoverClockPowerTest, SlackRejectedTrialSkipsPowerAndRollsBackExactly)
{
  auto* root = block_->findInst("clock_root");
  auto* leaf = block_->findInst("clock_leaf");
  auto* master = root->getMaster();
  const auto root_origin = root->getOrigin();
  const auto leaf_origin = leaf->getOrigin();
  auto* input_net = root->findITerm("A")->getNet();
  auto* output_net = leaf->findITerm("Z")->getNet();
  const auto inst_count = block_->getInsts().size();
  const auto net_count = block_->getNets().size();
  const float before_power = clockPower();
  const float before_hold = sta_->worstSlack(sta::MinMax::min());
  // Retain the initial hold slack rather than spending its positive margin.
  // Downsizing the leaf delays its capture clock and fails that floor.
  testing::internal::CaptureStdout();
  int changed = -1;
  EXPECT_NO_THROW(changed = resizer_.recoverClockPower(
                      {db_network_->findLibertyCell("BUF_X1")},
                      1e-9,
                      1e-9,
                      0,
                      1e-9,
                      1,
                      false,
                      true,
                      1));
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 0);
  EXPECT_NE(output.find("trial clock_leaf BUF_X1, power not evaluated "
                        "(clock/slack rejection)."),
            std::string::npos);
  EXPECT_NE(output.find(" below floor "), std::string::npos);
  EXPECT_NE(output.find("whole-design power 3 calls,"), std::string::npos);
  EXPECT_NE(output.find("clock/slack rejected trials skipped power 1."),
            std::string::npos);
  EXPECT_EQ(root->getMaster(), master);
  EXPECT_EQ(leaf->getMaster(), master);
  EXPECT_EQ(root->getOrigin(), root_origin);
  EXPECT_EQ(leaf->getOrigin(), leaf_origin);
  EXPECT_EQ(root->findITerm("A")->getNet(), input_net);
  EXPECT_EQ(leaf->findITerm("Z")->getNet(), output_net);
  EXPECT_EQ(block_->getInsts().size(), inst_count);
  EXPECT_EQ(block_->getNets().size(), net_count);
  EXPECT_NEAR(clockPower(), before_power, before_power * 1e-5f);
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::min()), before_hold, 1e-15f);
}

TEST_F(RecoverClockPowerTest, DontTouchAndDontUse)
{
  auto* root = block_->findInst("clock_root");
  auto* leaf = block_->findInst("clock_leaf");
  root->setDoNotTouch(true);
  resizer_.setDontUse(db_network_->findLibertyCell("BUF_X1"), true);
  EXPECT_EQ(recover(), 0);
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X16");
  EXPECT_EQ(std::string(leaf->getMaster()->getName()), "BUF_X16");
  resizer_.setDontUse(db_network_->findLibertyCell("BUF_X1"), false);
  EXPECT_GT(recover(), 0);
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X16");
}

TEST_F(RecoverClockPowerTest, InvalidCandidateAndPassesPreserveDesign)
{
  auto* root = block_->findInst("clock_root");
  auto* master = root->getMaster();
  EXPECT_ANY_THROW(
      resizer_.recoverClockPower({db_network_->findLibertyCell("INV_X1")},
                                 1e-9,
                                 1e-9,
                                 0,
                                 0,
                                 1,
                                 false,
                                 false));
  EXPECT_ANY_THROW(
      resizer_.recoverClockPower({}, 1e-9, 1e-9, 0, 0, 0, false, false));
  EXPECT_ANY_THROW(
      resizer_.recoverClockPower({}, 1e-9, 1e-9, 0, 0, 1, false, false, -1));
  EXPECT_EQ(root->getMaster(), master);
}

TEST_F(RecoverClockPowerTest, IdealClocksRejected)
{
  sta_->removePropagatedClock(sta_->cmdMode()->sdc()->findClock("clk"),
                              sta_->cmdMode());
  EXPECT_ANY_THROW(recover());
}

TEST_F(RecoverClockPowerTest, ClockPhasesDoNotEnterNetworkLatencyOrSkew)
{
  sta::PinSet pins(db_network_);
  pins.insert(db_network_->findPin("clk"));
  sta_->makeClock(
      "shifted", pins, true, 0.5e-9, {0.2e-9, 0.45e-9}, "", sta_->cmdMode());
  sta_->setPropagatedClock(sta_->cmdMode()->sdc()->findClock("shifted"),
                           sta_->cmdMode());
  sta_->setClockInsertion(sta_->cmdMode()->sdc()->findClock("clk"),
                          nullptr,
                          sta::RiseFallBoth::riseFall(),
                          sta::MinMaxAll::all(),
                          sta::EarlyLateAll::all(),
                          0.3e-9,
                          sta_->cmdMode()->sdc());
  // These bounds exceed physical network delay but are smaller than the
  // source phase shift and the separation between rising and falling edges.
  EXPECT_NO_THROW(
      resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                 0.15e-9,
                                 0.15e-9,
                                 0,
                                 0,
                                 1,
                                 false,
                                 false));
}

TEST_F(RecoverClockPowerTest, FixedBuffersAndDontTouchNetsAreProtected)
{
  auto* root = block_->findInst("clock_root");
  auto* leaf = block_->findInst("clock_leaf");
  root->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  leaf->findITerm("Z")->getNet()->setDoNotTouch(true);
  EXPECT_EQ(recover(), 0);
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X16");
  EXPECT_EQ(std::string(leaf->getMaster()->getName()), "BUF_X16");
}

TEST_F(RecoverClockPowerTest, MultipleScenesUseDistinctEnvelopes)
{
  addFastScene();
  ASSERT_EQ(sta_->scenes().size(), 2u);
  ep_.estimateWireParasitics();
  EXPECT_GT(recover(), 0);
}

TEST_F(RecoverClockPowerTest, ExplicitCornerCandidateUsesCanonicalLinkCell)
{
  auto* fast = addFastScene();
  auto* corner_copy = fast->findLibertyCell("BUF_X1");
  auto* primary = db_network_->findLibertyCell("BUF_X1");
  ASSERT_NE(corner_copy, primary);
  resizer_.setDontUse(primary, true);
  EXPECT_EQ(resizer_.recoverClockPower(
                {corner_copy}, 1e-9, 1e-9, 0, 0, 2, false, false),
            0);
  resizer_.setDontUse(primary, false);
  EXPECT_GT(resizer_.recoverClockPower(
                {corner_copy}, 1e-9, 1e-9, 0, 0, 2, false, false),
            0);
  EXPECT_EQ(std::string(block_->findInst("clock_leaf")->getMaster()->getName()),
            "BUF_X1");
}

TEST_F(RecoverClockPowerTest, MissingCandidateCornerModelRejectedBeforeChanges)
{
  addFastScene("BUF_X1");
  auto* master = block_->findInst("clock_root")->getMaster();
  EXPECT_ANY_THROW(recover());
  EXPECT_EQ(block_->findInst("clock_root")->getMaster(), master);
  EXPECT_EQ(block_->findInst("clock_leaf")->getMaster(), master);
}

TEST_F(RecoverClockPowerTest,
       MissingOriginalCornerModelRejectedBeforeEvaluation)
{
  addFastScene("BUF_X16");
  auto* master = block_->findInst("clock_root")->getMaster();
  EXPECT_ANY_THROW(recover());
  EXPECT_EQ(block_->findInst("clock_root")->getMaster(), master);
  EXPECT_EQ(block_->findInst("clock_leaf")->getMaster(), master);
}

TEST_F(RecoverClockPowerTest, IncompatibleCornerFunctionsRejected)
{
  auto* fast = addFastScene();
  auto invert = [](sta::LibertyCell* cell) {
    cell->findLibertyPort("Z")->setFunction(sta::FuncExpr::makeNot(
        sta::FuncExpr::makePort(cell->findLibertyPort("A"))));
  };
  auto* candidate = fast->findLibertyCell("BUF_X1");
  invert(candidate);
  EXPECT_ANY_THROW(recover());
  candidate->findLibertyPort("Z")->setFunction(
      sta::FuncExpr::makePort(candidate->findLibertyPort("A")));
  invert(fast->findLibertyCell("BUF_X16"));
  EXPECT_ANY_THROW(recover());
  EXPECT_EQ(std::string(block_->findInst("clock_root")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_EQ(std::string(block_->findInst("clock_leaf")->getMaster()->getName()),
            "BUF_X16");
}

TEST_F(RecoverClockPowerTest, FractionalSiteWidthCandidateIsRejected)
{
  auto* candidate = db_->findMaster("BUF_X1");
  const auto width = candidate->getWidth();
  ASSERT_GT(candidate->getSite()->getWidth(), 1u);
  candidate->setWidth(width + 1);
  EXPECT_EQ(recover(), 0);
  EXPECT_EQ(std::string(block_->findInst("clock_root")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_EQ(std::string(block_->findInst("clock_leaf")->getMaster()->getName()),
            "BUF_X16");
  candidate->setWidth(width);
}

TEST_F(RecoverClockPowerTest, IncompatiblePlacedOrientationIsRejected)
{
  auto* original = db_->findMaster("BUF_X16");
  auto* candidate = db_->findMaster("BUF_X1");
  ASSERT_FALSE(candidate->getSymmetryR90());
  original->setSymmetryR90();
  auto* root = block_->findInst("clock_root");
  root->setOrient(odb::dbOrientType::R90);
  root->setLocation(10000, 10000);
  const auto box = root->getBBox()->getBox();
  EXPECT_EQ(recover(), 0);
  EXPECT_EQ(root->getMaster(), original);
  EXPECT_EQ(root->getOrient(), odb::dbOrientType::R90);
  EXPECT_EQ(root->getBBox()->getBox(), box);
}

TEST_F(RecoverClockPowerTest, BottomUpTrialLimitRetainsFeasibleAcceptedState)
{
  checkSingleTrialBound(false);
}

TEST_F(RecoverClockPowerTest, TopDownTrialLimitRetainsFeasibleAcceptedState)
{
  checkSingleTrialBound(true);
}

class RecoverClockPowerBatchTest : public RecoverClockPowerTest
{
 protected:
  const char* designFile() override { return "TestRecoverClockPowerBatch.v"; }
  int recoverBatch(int checkpoints = 6,
                   double seconds = 300,
                   int trials = 0,
                   float hold_margin = 0)
  {
    return resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                      1e-9,
                                      1e-9,
                                      0,
                                      hold_margin,
                                      3,
                                      false,
                                      true,
                                      trials,
                                      true,
                                      checkpoints,
                                      seconds,
                                      1e-9,
                                      1e-9,
                                      true,
                                      true);
  }
};

TEST_F(RecoverClockPowerBatchTest, MultiCellBatchUsesOneNativeCertificate)
{
  const float before = clockPower();
  const float setup = sta_->worstSlack(sta::MinMax::max());
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(3), 2);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("attempts 1, accepted 1, full validations 3, native "
                        "power calls 3, proposed 2, committed 2"),
            std::string::npos);
  EXPECT_EQ(std::string(block_->findInst("clock_root")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_EQ(std::string(block_->findInst("clock_left")->getMaster()->getName()),
            "BUF_X1");
  EXPECT_EQ(
      std::string(block_->findInst("clock_right")->getMaster()->getName()),
      "BUF_X1");
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::max()),
            std::min(0.0f, setup) - 1e-15f);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()),
            std::min(0.0f, hold) - 1e-15f);
}

TEST_F(RecoverClockPowerBatchTest, AutomaticBudgetUsesTechnologyReference)
{
  setBufferDelay("BUF_X1", 20e-12);
  auto* second = addFastScene();
  setCellDelay(second->findLibertyCell("BUF_X1"), 6e-12);
  resizer_.setDontUse(db_network_->findLibertyCell("BUF_X1"), true);
  // Restricting replacements and dont_use do not change the technology scale.
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(-1,
                          -1,
                          1,
                          2,
                          0,
                          false,
                          false,
                          0,
                          0,
                          {db_network_->findLibertyCell("BUF_X4")}),
            0);
  const auto output = testing::internal::GetCapturedStdout();
  const auto [final_budget, intermediate_budget] = reportedBudgets(output);
  EXPECT_NEAR(final_budget, 3e-12f, 1e-16f) << output;
  EXPECT_NEAR(intermediate_budget, 6e-12f, 1e-16f) << output;
  EXPECT_NE(output.find("reference BUF_X1"), std::string::npos) << output;
}

TEST_F(RecoverClockPowerBatchTest, NonpositiveIntrinsicFallsBackToZeroBudget)
{
  setBufferDelay("BUF_X1", 8e-12);
  auto* cell = db_network_->findLibertyCell("BUF_X1");
  auto* arc = cell->timingArcSets().front()->arcs().front();
  *arc->gateTableModel()->delayModel()->table() = sta::Table(0);
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(-1, -1, 1, 2), 0);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(reportedBudgets(output), std::make_pair(0.0f, 0.0f));
  EXPECT_NE(output.find("No finite positive compatible buffer intrinsic delay"),
            std::string::npos)
      << output;
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(6e-12, 9e-12, 1, 2), 0);
  const auto explicit_output = testing::internal::GetCapturedStdout();
  EXPECT_NEAR(reportedBudgets(explicit_output).first, 6e-12f, 1e-16f);
  EXPECT_EQ(explicit_output.find("No finite positive"), std::string::npos);
}

TEST_F(RecoverClockPowerBatchTest, TimingAllowanceNeverRebasesAcrossWaves)
{
  setBufferDelay("BUF_X16", 10e-12);
  setBufferDelay("BUF_X4", 14e-12);
  setBufferDelay("BUF_X1", 18e-12);
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(6e-12,
                          6e-12,
                          3,
                          6,
                          0,
                          false,
                          false,
                          0,
                          0,
                          {db_network_->findLibertyCell("BUF_X1"),
                           db_network_->findLibertyCell("BUF_X4")}),
            2);
  const auto output = testing::internal::GetCapturedStdout();
  for (const char* name : {"clock_left", "clock_right"}) {
    EXPECT_EQ(std::string(block_->findInst(name)->getMaster()->getName()),
              "BUF_X4")
        << output;
  }
  EXPECT_NE(output.find("best updates 1"), std::string::npos) << output;
}

TEST_F(RecoverClockPowerBatchTest, IndependentMetricsEachReceiveTheirBudget)
{
  setBufferDelay("BUF_X16", 10e-12);
  setBufferDelay("BUF_X1", 14e-12);
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(5e-12, 5e-12, 1, 3, 1), 1);
  const auto output = testing::internal::GetCapturedStdout();
  const float hold_loss = hold - sta_->worstSlack(sta::MinMax::min());
  EXPECT_GT(hold_loss, 3.5e-12f) << output;
  EXPECT_LE(hold_loss, 5e-12f + 1e-15f) << output;
  EXPECT_GT(hold_loss + 4e-12f, 5e-12f) << output;
  // Latency and spread also increase by 4 ps; these are independent limits.
  EXPECT_NE(output.find("best updates 1"), std::string::npos) << output;
}

TEST_F(RecoverClockPowerBatchTest, ExplicitMarginAddsASeparateFloor)
{
  setBufferDelay("BUF_X16", 10e-12);
  setBufferDelay("BUF_X1", 14e-12);
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(5e-12, 5e-12, 1, 6, 0, false, true, 0, 1e-9), 0);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("cached clock checks 0"), std::string::npos) << output;
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::min()), hold, 1e-15f);
}

TEST_F(RecoverClockPowerBatchTest, TwoCheckpointsReserveBaselineAndFinal)
{
  const float before = clockPower();
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(2), 0);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("stop max_checkpoints."), std::string::npos);
  EXPECT_NE(output.find("attempts 0, accepted 0, full validations 2, native "
                        "power calls 2, proposed 0, committed 0"),
            std::string::npos);
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
  for (auto* inst : block_->getInsts()) {
    if (std::string(inst->getName()).starts_with("clock_")) {
      EXPECT_EQ(std::string(inst->getMaster()->getName()), "BUF_X16");
    }
  }
}

TEST_F(RecoverClockPowerBatchTest, ExpiredBaselineStillValidatesFinal)
{
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(6, 1e-12), 0);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("stop max_seconds."), std::string::npos);
  EXPECT_NE(output.find("full validations 2, native power calls 2"),
            std::string::npos);
}

TEST_F(RecoverClockPowerBatchTest,
       RejectedBatchRestoresAllInstancesAndSkipsPower)
{
  struct State
  {
    odb::dbInst* inst;
    odb::dbMaster* master;
    odb::Point origin;
    std::vector<odb::dbNet*> nets;
  };
  std::vector<State> states;
  int64_t area = 0;
  for (auto* inst : block_->getInsts()) {
    states.push_back({inst, inst->getMaster(), inst->getOrigin(), {}});
    for (auto* pin : inst->getITerms()) {
      states.back().nets.push_back(pin->getNet());
    }
    area += int64_t(inst->getMaster()->getWidth())
            * inst->getMaster()->getHeight();
  }
  const float before = clockPower();
  // A downstream sink limit is outside the inner move's local ERC guard.
  sta_->findRequireds();
  float initial_slew = 0;
  for (const char* name : {"sink1/CK", "sink2/CK"}) {
    auto* vertex = sta_->graph()->pinLoadVertex(db_network_->findPin(name));
    initial_slew
        = std::max(initial_slew,
                   sta::delayAsFloat(sta_->slew(vertex,
                                                sta::RiseFallBoth::riseFall(),
                                                {sta_->cmdScene()},
                                                sta::MinMax::max())));
  }
  db_network_->findLibertyCell("DFF_X1")->findLibertyPort("CK")->setSlewLimit(
      initial_slew + 1e-15f, sta::MinMax::max());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(3), 0);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("attempts 1, accepted 0, full validations 3, native "
                        "power calls 2, proposed 2, committed 0"),
            std::string::npos);
  int64_t after_area = 0;
  for (const auto& state : states) {
    EXPECT_EQ(state.inst->getMaster(), state.master);
    EXPECT_EQ(state.inst->getOrigin(), state.origin);
    size_t i = 0;
    for (auto* pin : state.inst->getITerms()) {
      EXPECT_EQ(pin->getNet(), state.nets[i++]);
    }
    after_area += int64_t(state.inst->getMaster()->getWidth())
                  * state.inst->getMaster()->getHeight();
  }
  EXPECT_EQ(after_area, area);
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
}

TEST_F(RecoverClockPowerBatchTest, TrialCapCountsConstituentReplacements)
{
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(6, 300, 1), 1);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("trials 1,"), std::string::npos);
  EXPECT_NE(output.find("stop max_trials."), std::string::npos);
  EXPECT_NE(
      output.find(
          "full validations 3, native power calls 3, proposed 1, committed 1"),
      std::string::npos);
}

TEST_F(RecoverClockPowerBatchTest,
       CandidateSdcCapLimitRejectsInnerMovesBeforeCheckpoint)
{
  const float before = clockPower();
  sta_->setCapacitanceLimit(db_network_->dbToSta(db_->findMaster("BUF_X1")),
                            sta::MinMax::max(),
                            0,
                            sta_->cmdMode()->sdc());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(3), 0);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("full validations 2, native power calls 2"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("local electrical checks 4, rejections 4"),
            std::string::npos)
      << output;
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
  for (const char* name : {"clock_left", "clock_right"}) {
    EXPECT_EQ(std::string(block_->findInst(name)->getMaster()->getName()),
              "BUF_X16");
  }
}

TEST_F(RecoverClockPowerBatchTest, PreexistingDownstreamSlewPinsAreWatched)
{
  db_network_->findLibertyCell("DFF_X1")->findLibertyPort("CK")->setSlewLimit(
      0, sta::MinMax::max());
  const float before = clockPower();
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBatch(3), 0);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("watched slew pins 2"), std::string::npos) << output;
  EXPECT_NE(output.find("local electrical checks 4, rejections 4"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("full validations 2, native power calls 2"),
            std::string::npos)
      << output;
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
}

TEST_F(RecoverClockPowerBatchTest, MultipleScenesAndProtectedInstances)
{
  addFastScene();
  ep_.estimateWireParasitics();
  auto* left = block_->findInst("clock_left");
  left->setDoNotTouch(true);
  EXPECT_EQ(recoverBatch(), 1);
  EXPECT_EQ(std::string(left->getMaster()->getName()), "BUF_X16");
  EXPECT_EQ(
      std::string(block_->findInst("clock_right")->getMaster()->getName()),
      "BUF_X1");
}

TEST_F(RecoverClockPowerBatchTest, InvalidBudgetsPreserveDesign)
{
  EXPECT_ANY_THROW(recoverBatch(1));
  EXPECT_ANY_THROW(recoverBatch(6, 0));
  EXPECT_ANY_THROW(recoverBatch(6, std::numeric_limits<double>::infinity()));
  EXPECT_ANY_THROW(recoverBatch(6, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(std::string(block_->findInst("clock_left")->getMaster()->getName()),
            "BUF_X16");
}

TEST_F(RecoverClockPowerTest, BatchSlackRejectedBatchSkipsPower)
{
  const float before = clockPower();
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   1e-9,
                                   1e-9,
                                   0,
                                   1e-9,
                                   3,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   1e-9,
                                   1e-9,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 0) << output;
  EXPECT_NE(
      output.find(
          "attempts 0, accepted 0, full validations 2, native power calls 2"),
      std::string::npos)
      << output;
  EXPECT_NE(output.find("tentative checks 2, accepts 0, rejects 2"),
            std::string::npos)
      << output;
  EXPECT_EQ(std::string(block_->findInst("clock_root")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_EQ(std::string(block_->findInst("clock_leaf")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
}

TEST_F(RecoverClockPowerBatchTest, BatchCommitClosesProfilerJournal)
{
  const std::string directory
      = std::string(std::getenv("TEST_TMPDIR")) + "/batch_profile";
  {
    RepairSetupProfiler profiler(
        resizer_, RepairSetupProfiler::Mode::kTiming, directory);
    EXPECT_EQ(recoverBatch(3), 2);
    profiler.complete(true);
  }
  std::ifstream input(directory + "/run_1/metadata.csv");
  const std::string metadata((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(metadata.find("\"open_journals\",\"0\"\n"), std::string::npos)
      << metadata;
}

TEST_F(RecoverClockPowerBatchTest, RejectedBatchPreservesCallerEco)
{
  auto* root = block_->findInst("clock_root");
  auto* original = root->getMaster();
  est::IncrementalParasiticsGuard parasitics(&ep_);
  resizer_.journalBeginTest();
  sta_->replaceCell(db_network_->dbToSta(root),
                    db_network_->dbToSta(db_->findMaster("BUF_X8")));
  sta_->setCapacitanceLimit(db_network_->dbToSta(db_->findMaster("BUF_X1")),
                            sta::MinMax::max(),
                            0,
                            sta_->cmdMode()->sdc());
  EXPECT_EQ(recoverBatch(3), 0);
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X8");
  EXPECT_EQ(std::string(block_->findInst("clock_left")->getMaster()->getName()),
            "BUF_X16");
  EXPECT_EQ(
      std::string(block_->findInst("clock_right")->getMaster()->getName()),
      "BUF_X16");
  resizer_.journalRestoreTest();
  EXPECT_EQ(root->getMaster(), original);
}

TEST_F(RecoverClockPowerBatchTest,
       LocalProjectionSelectsFeasibleIntermediateCells)
{
  auto* weakest = db_network_->findLibertyCell("BUF_X1");
  // The weakest cell's native Liberty max-cap limit rules it out. The next
  // candidate is feasible; local relaxation must keep the useful whole batch.
  weakest->findLibertyPort("Z")->setCapacitanceLimit(0, sta::MinMax::max());
  const float before = clockPower();
  const float setup = sta_->worstSlack(sta::MinMax::max());
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  const int changed = resizer_.recoverClockPower(
      {weakest, db_network_->findLibertyCell("BUF_X4")},
      1e-9,
      1e-9,
      0,
      0,
      3,
      false,
      true,
      0,
      true,
      3,
      300,
      1e-9,
      1e-9,
      true,
      true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 2) << output;
  EXPECT_EQ(std::string(block_->findInst("clock_left")->getMaster()->getName()),
            "BUF_X4");
  EXPECT_EQ(
      std::string(block_->findInst("clock_right")->getMaster()->getName()),
      "BUF_X4");
  EXPECT_NE(
      output.find(
          "attempts 1, accepted 1, full validations 3, native power calls 3"),
      std::string::npos)
      << output;
  EXPECT_NE(output.find("projection rounds 2, candidate relaxations 2"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::max()),
            std::min(0.0f, setup) - 1e-15f);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()),
            std::min(0.0f, hold) - 1e-15f);
}

class RecoverClockPowerWnsTest : public RecoverClockPowerTest
{
 protected:
  const char* designFile() override { return "TestRecoverClockPowerWns.v"; }
};

class RecoverClockPowerBudgetTest : public RecoverClockPowerTest
{
 protected:
  const char* designFile() override { return "TestRecoverClockPowerBudget.v"; }
  void SetUp() override
  {
    RecoverClockPowerTest::SetUp();
    setBufferDelay("BUF_X16", 10e-12);
    setBufferDelay("BUF_X8", 10e-12);
    setBufferDelay("BUF_X4", 14e-12);
  }
  int recoverChain(float final_budget, int checkpoints = 6, int passes = 3)
  {
    return recoverBudget(final_budget,
                         10e-12,
                         passes,
                         checkpoints,
                         0,
                         false,
                         false,
                         0,
                         0,
                         {db_network_->findLibertyCell("BUF_X4")});
  }
};

TEST_F(RecoverClockPowerBudgetTest,
       BoundarySavesUsefulPrefixAndRestoresExploratorySuffix)
{
  est::IncrementalParasiticsGuard parasitics(&ep_);
  auto* root = block_->findInst("clock_root");
  auto* root_master = root->getMaster();
  resizer_.journalBeginTest();
  sta_->replaceCell(db_network_->dbToSta(root),
                    db_network_->dbToSta(db_->findMaster("BUF_X8")));
  const float before = clockPower();
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverChain(5e-12), 1);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("exploratory batches 2, best updates 1"),
            std::string::npos)
      << output;
  EXPECT_NE(
      output.find("best history 1, restored suffix 1, boundary deferrals 1"),
      std::string::npos)
      << output;
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X8");
  EXPECT_LT(clockPower(), before);
  int changed = 0;
  for (const char* name : {"clock_first", "clock_second"}) {
    changed += std::string(block_->findInst(name)->getMaster()->getName())
               == "BUF_X4";
  }
  EXPECT_EQ(changed, 1);
  resizer_.journalRestoreTest();
  EXPECT_EQ(root->getMaster(), root_master);
  EXPECT_EQ(block_->findInst("clock_first")->getMaster(), root_master);
  EXPECT_EQ(block_->findInst("clock_second")->getMaster(), root_master);
}

TEST_F(RecoverClockPowerBudgetTest, ExhaustedBudgetReturnsBaselineBest)
{
  const float before = clockPower();
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverChain(0, 3, 1), 0);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("exploratory batches 1, best updates 0"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("best history 0, restored suffix 2"), std::string::npos)
      << output;
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
  for (const char* name : {"clock_root", "clock_first", "clock_second"}) {
    EXPECT_EQ(std::string(block_->findInst(name)->getMaster()->getName()),
              "BUF_X16");
  }
}

TEST_F(RecoverClockPowerBudgetTest,
       ExceptionDuringBestRestorePreservesCallerEco)
{
  est::IncrementalParasiticsGuard parasitics(&ep_);
  class ThrowOnRestore : public odb::dbBlockCallBackObj
  {
   public:
    void inDbInstSwapMasterAfter(odb::dbInst* inst) override
    {
      if (std::string(inst->getName()) != "clock_second") {
        return;
      }
      if (std::string(inst->getMaster()->getName()) == "BUF_X4") {
        resized = true;
      } else if (resized && !fired) {
        fired = true;
        throw std::runtime_error("injected best restoration failure");
      }
    }
    bool resized = false;
    bool fired = false;
  } callback;
  auto* root = block_->findInst("clock_root");
  auto* initial = root->getMaster();
  resizer_.journalBeginTest();
  sta_->replaceCell(db_network_->dbToSta(root),
                    db_network_->dbToSta(db_->findMaster("BUF_X8")));
  const auto before = clockPower();
  callback.addOwner(block_);
  EXPECT_ANY_THROW(recoverChain(0, 3, 1));
  EXPECT_TRUE(callback.fired);
  callback.removeOwner();
  EXPECT_EQ(std::string(root->getMaster()->getName()), "BUF_X8");
  EXPECT_EQ(block_->findInst("clock_first")->getMaster(), initial);
  EXPECT_EQ(block_->findInst("clock_second")->getMaster(), initial);
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
  resizer_.journalRestoreTest();
  EXPECT_EQ(root->getMaster(), initial);
}

class RecoverClockPowerRecoveryTest : public RecoverClockPowerBudgetTest
{
 protected:
  const char* designFile() override
  {
    return "TestRecoverClockPowerRecovery.v";
  }
  void SetUp() override
  {
    RecoverClockPowerBudgetTest::SetUp();
    setBufferDelay("BUF_X8", 18e-12);
  }
};

TEST_F(RecoverClockPowerRecoveryTest, LaterMoveRecoversALowerPowerFinalBest)
{
  const float before = clockPower();
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverChain(5e-12), 3);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("exploratory batches 2, best updates 2"),
            std::string::npos)
      << output;
  EXPECT_NE(
      output.find("best history 3, restored suffix 0, boundary deferrals 1"),
      std::string::npos)
      << output;
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()), hold - 5e-12f - 1e-15f);
  EXPECT_LT(clockPower(), before);
  for (const char* name : {"clock_first", "clock_second", "clock_third"}) {
    EXPECT_EQ(std::string(block_->findInst(name)->getMaster()->getName()),
              "BUF_X4");
  }
}

TEST_F(RecoverClockPowerWnsTest, BadTentativeWnsDoesNotPoisonUsefulBatch)
{
  const float before = clockPower();
  const float setup = sta_->worstSlack(sta::MinMax::max());
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   1e-9,
                                   1e-9,
                                   0,
                                   1e-9,
                                   1,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   1e-9,
                                   1e-9,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_GE(changed, 1) << output;
  EXPECT_NE(
      output.find(
          "attempts 1, accepted 1, full validations 3, native power calls 3"),
      std::string::npos)
      << output;
  EXPECT_EQ(output.find("rejects 0,"), std::string::npos) << output;
  EXPECT_NE(output.find("snapshot findRequireds+ensureClkArrivals 3 calls"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("graph clock/electrical traversal 6 calls"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::max()),
            std::min(0.0f, setup) - 1e-15f);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()), hold - 1e-15f);
  EXPECT_EQ(std::string(block_->findInst("clock_good")->getMaster()->getName()),
            "BUF_X1");
  EXPECT_EQ(std::string(block_->findInst("clock_bad")->getMaster()->getName()),
            "BUF_X16");
}

TEST_F(RecoverClockPowerTest, InnerTrialLimitRetainsCertifiedBaseline)
{
  const float before = clockPower();
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   1e-9,
                                   1e-9,
                                   0,
                                   1e-9,
                                   3,
                                   false,
                                   true,
                                   1,
                                   true,
                                   3,
                                   300,
                                   1e-9,
                                   1e-9,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 0) << output;
  EXPECT_NE(output.find("trials 1,"), std::string::npos) << output;
  EXPECT_NE(output.find("stop max_trials."), std::string::npos) << output;
  EXPECT_NE(output.find("full validations 2, native power calls 2"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("tentative checks 1, accepts 0, rejects 1"),
            std::string::npos)
      << output;
  EXPECT_NEAR(clockPower(), before, before * 1e-5f);
}

class RecoverClockPowerCriticalTest : public RecoverClockPowerTest
{
 protected:
  const char* designFile() override
  {
    return "TestRecoverClockPowerCritical.v";
  }
  void SetUp() override
  {
    RecoverClockPowerTest::SetUp();
    // Isolate path-depth extrema from placement wire-length differences.
    for (auto* inst : block_->getInsts()) {
      inst->setLocation(10000, 10000);
    }
    ep_.estimateWireParasitics();
  }
  std::vector<float> clockLatencies(const char* name)
  {
    sta_->findRequireds();
    sta_->ensureClkArrivals();
    auto* vertex = sta_->graph()->pinLoadVertex(db_network_->findPin(name));
    sta::VertexPathIterator paths(vertex, sta_.get());
    std::vector<float> result;
    while (paths.hasNext()) {
      auto* path = paths.next();
      if (path->isClock(sta_.get())) {
        sta::PathExpanded expanded(path, false, sta_.get());
        result.push_back(
            sta::delayAsFloat(path->arrival() - expanded.path(0)->arrival()));
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }
};

TEST_F(RecoverClockPowerCriticalTest, ZeroBudgetPreservesPositiveStartingWns)
{
  setBufferDelay("BUF_X16", 10e-12);
  setBufferDelay("BUF_X1", 6e-12);
  setBufferDelay("BUF_X2", 50e-12);
  const float setup = sta_->worstSlack(sta::MinMax::max());
  ASSERT_GT(setup, 0);
  testing::internal::CaptureStdout();
  EXPECT_EQ(recoverBudget(0, 0), 0);
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("tentative checks 2, accepts 0, rejects 2"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("cached clock checks 0"), std::string::npos) << output;
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()), setup, 1e-15f);
}

TEST_F(RecoverClockPowerCriticalTest,
       PreserveClockExtremaKeepsUnrelatedBranchUseful)
{
  const auto early = clockLatencies("sink_early/CK");
  const auto late = clockLatencies("sink_late/CK");
  ASSERT_FALSE(early.empty());
  ASSERT_FALSE(late.empty());
  const float before = clockPower();
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   -1,
                                   -1,
                                   0,
                                   0,
                                   1,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   0,
                                   0,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 1) << output;
  EXPECT_EQ(
      std::string(block_->findInst("clock_mid_anchor")->getMaster()->getName()),
      "BUF_X16");
  EXPECT_EQ(
      std::string(block_->findInst("clock_mid_leaf")->getMaster()->getName()),
      "BUF_X1");
  EXPECT_NE(output.find("critically frozen 4"), std::string::npos) << output;
  EXPECT_NE(output.find("full validations 3, native power calls 3"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  const auto after_early = clockLatencies("sink_early/CK");
  const auto after_late = clockLatencies("sink_late/CK");
  ASSERT_EQ(after_early.size(), early.size());
  ASSERT_EQ(after_late.size(), late.size());
  for (size_t i = 0; i < early.size(); ++i) {
    EXPECT_NEAR(after_early[i], early[i], 1e-15f);
  }
  for (size_t i = 0; i < late.size(); ++i) {
    EXPECT_NEAR(after_late[i], late[i], 1e-15f);
  }
}

TEST_F(RecoverClockPowerWnsTest, ThirdAttemptUsesStrongModelFallback)
{
  const float before = clockPower();
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1"),
                                    db_network_->findLibertyCell("BUF_X2"),
                                    db_network_->findLibertyCell("BUF_X4"),
                                    db_network_->findLibertyCell("BUF_X8")},
                                   1e-9,
                                   1e-9,
                                   0,
                                   1e-9,
                                   1,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   1e-9,
                                   1e-9,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_GT(changed, 0) << output;
  EXPECT_EQ(output.find("strong fallbacks 0"), std::string::npos) << output;
  EXPECT_NE(output.find("full validations 3, native power calls 3"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()), hold - 1e-15f);
  const std::string bad_master
      = block_->findInst("clock_bad")->getMaster()->getName();
  EXPECT_TRUE(bad_master == "BUF_X4" || bad_master == "BUF_X8")
      << bad_master << "\n"
      << output;
}

class RecoverClockPowerGeneratedTest : public RecoverClockPowerCriticalTest
{
 protected:
  const char* designFile() override
  {
    return "TestRecoverClockPowerGenerated.v";
  }
  void SetUp() override
  {
    RecoverClockPowerCriticalTest::SetUp();
    sta::PinSet pins(db_network_);
    pins.insert(db_network_->findPin("generator/Q"));
    sta_->makeGeneratedClock("generated",
                             pins,
                             false,
                             db_network_->findPin("generator/CK"),
                             sta_->cmdMode()->sdc()->findClock("clk"),
                             2,
                             0,
                             0,
                             false,
                             false,
                             {},
                             {},
                             "",
                             sta_->cmdMode());
    sta_->setPropagatedClock(sta_->cmdMode()->sdc()->findClock("generated"),
                             sta_->cmdMode());
  }
};

class RecoverClockPowerSpreadTest : public RecoverClockPowerCriticalTest
{
 protected:
  const char* designFile() override { return "TestRecoverClockPowerSpread.v"; }
  void SetUp() override
  {
    RecoverClockPowerCriticalTest::SetUp();
    block_->setDieArea(odb::Rect(0, 0, 3000000, 3000000));
    for (const char* name : {"clock_early_leaf", "sink_early"}) {
      block_->findInst(name)->setLocation(170000, 10000);
    }
    for (const char* name : {"clock_middle_leaf", "sink_middle"}) {
      block_->findInst(name)->setLocation(210000, 10000);
    }
    for (const char* name : {"clock_late_leaf", "sink_late"}) {
      block_->findInst(name)->setLocation(1010000, 10000);
    }
    block_->findInst("sink_useful")->setLocation(2010000, 10000);
    ep_.setHWireClkRC(nullptr, sta_->cmdScene(), 1e8, 1e-12);
    ep_.setVWireClkRC(nullptr, sta_->cmdScene(), 1e8, 1e-12);
    ep_.setHWireSignalRC(nullptr, sta_->cmdScene(), 1e8, 1e-12);
    ep_.setVWireSignalRC(nullptr, sta_->cmdScene(), 1e8, 1e-12);
    ep_.estimateWireParasitics();
  }
};

TEST_F(RecoverClockPowerSpreadTest,
       EarlySpeedupRejectsInnerMoveAndKeepsUsefulBranch)
{
  const auto early = clockLatencies("sink_early/CK");
  const auto middle = clockLatencies("sink_middle/CK");
  const auto late = clockLatencies("sink_late/CK");
  ASSERT_FALSE(early.empty());
  ASSERT_FALSE(middle.empty());
  ASSERT_FALSE(late.empty());
  // Long wire resistance makes the middle leaf's reduced input capacitance
  // advance its clock past the initial early anchor. The fixed-wire proposal
  // sees only the cell delay/load change and proposes this harmful resize.
  auto* middle_leaf = block_->findInst("clock_middle_leaf");
  auto* original_master = middle_leaf->getMaster();
  sta_->replaceCell(db_network_->dbToSta(middle_leaf),
                    db_network_->dbToSta(db_->findMaster("BUF_X1")));
  ep_.estimateWireParasitics();
  const auto accelerated = clockLatencies("sink_middle/CK");
  ASSERT_FALSE(accelerated.empty());
  EXPECT_LT(accelerated.front(), early.front());
  sta_->replaceCell(db_network_->dbToSta(middle_leaf),
                    db_network_->dbToSta(original_master));
  ep_.estimateWireParasitics();
  const float before = clockPower();
  const float setup = sta_->worstSlack(sta::MinMax::max());
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   -1,
                                   -1,
                                   0,
                                   0,
                                   1,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   0,
                                   0,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 1) << output;
  EXPECT_EQ(std::string(
                block_->findInst("clock_middle_leaf")->getMaster()->getName()),
            "BUF_X16")
      << output;
  EXPECT_EQ(std::string(
                block_->findInst("clock_useful_leaf")->getMaster()->getName()),
            "BUF_X1")
      << output;
  EXPECT_NE(output.find("cached clock checks 3, rejections 2"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("full validations 3, native power calls 3"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::max()), setup - 1e-15f);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()), hold - 1e-15f);
  EXPECT_EQ(clockLatencies("sink_early/CK"), early);
  EXPECT_EQ(clockLatencies("sink_middle/CK"), middle);
  EXPECT_EQ(clockLatencies("sink_late/CK"), late);
}

TEST_F(RecoverClockPowerGeneratedTest,
       PreserveGeneratedClockExtremaProtectsMasterAncestry)
{
  const auto master = clockLatencies("generator/CK");
  const auto generated = clockLatencies("sink_generated/CK");
  ASSERT_FALSE(master.empty());
  ASSERT_FALSE(generated.empty());
  const float before = clockPower();
  const float setup = sta_->worstSlack(sta::MinMax::max());
  const float hold = sta_->worstSlack(sta::MinMax::min());
  testing::internal::CaptureStdout();
  const int changed
      = resizer_.recoverClockPower({db_network_->findLibertyCell("BUF_X1")},
                                   -1,
                                   -1,
                                   0,
                                   0,
                                   1,
                                   false,
                                   true,
                                   0,
                                   true,
                                   3,
                                   300,
                                   0,
                                   0,
                                   true,
                                   true);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(changed, 1) << output;
  for (const char* name : {"clock_gen_anchor", "clock_gen_leaf"}) {
    EXPECT_EQ(std::string(block_->findInst(name)->getMaster()->getName()),
              "BUF_X16")
        << name << "\n"
        << output;
  }
  EXPECT_EQ(std::string(
                block_->findInst("clock_useful_leaf")->getMaster()->getName()),
            "BUF_X1")
      << output;
  EXPECT_NE(output.find("full validations 3, native power calls 3"),
            std::string::npos)
      << output;
  EXPECT_LT(clockPower(), before);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::max()), setup - 1e-15f);
  EXPECT_GE(sta_->worstSlack(sta::MinMax::min()), hold - 1e-15f);
  const auto after_master = clockLatencies("generator/CK");
  const auto after_generated = clockLatencies("sink_generated/CK");
  ASSERT_EQ(after_master.size(), master.size());
  ASSERT_EQ(after_generated.size(), generated.size());
  for (size_t i = 0; i < master.size(); ++i) {
    EXPECT_NEAR(after_master[i], master[i], 1e-15f);
  }
  for (size_t i = 0; i < generated.size(); ++i) {
    EXPECT_NEAR(after_generated[i], generated[i], 1e-15f);
  }
}
}  // namespace
}  // namespace rsz
