// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <memory>
#include <vector>

#include "PdnTest.h"
#include "connect.h"
#include "domain.h"
#include "grid.h"
#include "odb/db.h"
#include "shape.h"
#include "straps.h"
#include "techlayer.h"
#include "tst/physical_builder.h"
#include "via.h"

namespace pdn {
namespace {

class TestPDNPadHop : public PdnTest
{
 protected:
  void SetUp() override
  {
    PdnTest::SetUp();
    stack_ = makeRoutingStack(5);
    m4_ = stack_.routing[3];
    m5_ = stack_.routing[4];
    via_ = tst::addVia(tech(), "V45", m4_, stack_.cuts[3], m5_, 50, 10);
    block()->setDieArea(odb::Rect(0, 0, 10000, 10000));
    block()->setCoreArea(odb::Rect(1500, 1500, 8500, 8500));
    power_ = odb::dbNet::create(block(), "VDD");
    power_->setSigType(odb::dbSigType::POWER);
    auto* ground = odb::dbNet::create(block(), "VSS");
    ground->setSigType(odb::dbSigType::GROUND);
    auto* master = tst::makeMaster(*db_->getLibs().begin(),
                                   "PAD",
                                   900,
                                   1000,
                                   {{.name = "VDD",
                                     .layer = m4_,
                                     .rect = odb::Rect(700, 300, 900, 700),
                                     .sig_type = odb::dbSigType::POWER}},
                                   odb::dbMasterType::PAD_POWER);
    auto* inst = odb::dbInst::create(block(), master, "pad");
    inst->setLocation(100, 4000);
    inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
    auto* iterm = inst->findITerm("VDD");
    iterm->connect(power_);
    domain_ = std::make_unique<VoltageDomain>(nullptr,
                                              block(),
                                              power_,
                                              ground,
                                              std::vector<odb::dbNet*>{},
                                              getLogger());
    grid_ = std::make_unique<TestGrid>(
        domain_.get(), "grid", true, std::vector<odb::dbTechLayer*>{});
    auto connect = std::make_unique<Connect>(grid_.get(), m4_, m5_);
    connect->addFixedVia(via_);
    connect->setMaxRows(1);
    connect->setMaxColumns(1);
    grid_->addConnect(std::move(connect));
    auto counts = std::make_shared<odb::PtrMap<odb::dbNet, int>>();
    auto pad = std::make_unique<PadDirectConnectionStraps>(
        grid_.get(), iterm, std::vector<odb::dbTechLayer*>{m4_}, counts);
    pad_ = pad.get();
    grid_->addStrap(std::move(pad));
  }

  ShapePtr target(odb::dbTechLayer* layer, const odb::Rect& rect)
  {
    auto shape = std::make_shared<Shape>(
        layer, power_, rect, odb::dbWireShapeType::RING);
    shape->setShapeType(Shape::kFixed);
    shapes_[layer].insert(shape);
    return shape;
  }

  void build()
  {
    pad_->makeShapes(shapes_);
    pad_->cutShapes(obstructions_);
    auto local = obstructions_;
    grid_->makeVias(shapes_, obstructions_, local);
  }

  odb::dbSWire* write(Grid::ViaShapeMap* via_shapes = nullptr)
  {
    auto* wire = odb::dbSWire::create(power_, odb::dbWireType::ROUTED);
    grid_->writeToDb({{power_, wire}}, {}, obstructions_, via_shapes);
    return wire;
  }

  RoutingStack stack_;
  odb::dbTechLayer* m4_;
  odb::dbTechLayer* m5_;
  odb::dbTechVia* via_;
  odb::dbNet* power_;
  std::unique_ptr<VoltageDomain> domain_;
  std::unique_ptr<TestGrid> grid_;
  PadDirectConnectionStraps* pad_;
  Shape::ShapeTreeMap shapes_;
  Shape::ObstructionTreeMap obstructions_;
};

TEST_F(TestPDNPadHop, EscapesThenRoutesOnPreferredLayerToOwnerlessRing)
{
  const auto ring = target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  build();
  ASSERT_EQ(pad_->getShapeCount(), 2);
  const auto escape = *pad_->getShapes().at(m4_).begin();
  const auto route = *pad_->getShapes().at(m5_).begin();
  EXPECT_EQ(escape->getRect(), odb::Rect(800, 4300, 1100, 4700));
  EXPECT_EQ(route->getRect(), odb::Rect(1000, 4300, 2200, 4700));
  EXPECT_TRUE(escape->hasITermConnections());
  EXPECT_FALSE(route->hasITermConnections());
  EXPECT_TRUE(route->getRect().overlaps(ring->getRect()));
  EXPECT_TRUE(pad_->isPadHop(escape.get(), route.get()));
  Grid::ViaShapeMap via_shapes;
  auto* wire = write(&via_shapes);
  std::vector<odb::dbSBox*> live_boxes;
  int vias = 0;
  int metals = 0;
  for (auto* box : wire->getWires()) {
    live_boxes.push_back(box);
    if (box->isVia()) {
      ++vias;
      const auto point = box->getViaXY();
      EXPECT_TRUE(escape->getRect().intersects(point)
                  || ring->getRect().intersects(point));
      EXPECT_TRUE(route->getRect().intersects(point));
    } else {
      ++metals;
    }
  }
  EXPECT_EQ(via_shapes.size(), 2);
  int captured = 0;
  for (const auto& [via, boxes] : via_shapes) {
    EXPECT_TRUE(via->isValid());
    EXPECT_FALSE(via->isFailed());
    EXPECT_FALSE(boxes.empty());
    for (auto* box : boxes) {
      EXPECT_NE(std::find(live_boxes.begin(), live_boxes.end(), box),
                live_boxes.end());
      ++captured;
    }
  }
  EXPECT_EQ(captured, vias);
  EXPECT_EQ(vias, 2);
  EXPECT_EQ(metals, 2);
  EXPECT_EQ(pad_->getShapeCount(), 2);
}

TEST_F(TestPDNPadHop, LimitsHopWidthOnBothLayers)
{
  m4_->setMaxWidth(120);
  m5_->setMaxWidth(150);
  target(m4_, odb::Rect(2000, 2000, 2120, 8000));
  build();
  ASSERT_EQ(pad_->getShapeCount(), 2);
  EXPECT_EQ((*pad_->getShapes().at(m4_).begin())->getRect().dy(), 120);
  EXPECT_EQ((*pad_->getShapes().at(m5_).begin())->getRect().dy(), 120);
  auto* wire = write();
  int vias = 0;
  const auto check_width = [](const odb::dbBox* box) {
    const auto* layer = box->getTechLayer();
    if (layer->hasMaxWidth()) {
      EXPECT_LE(box->getBox().minDXDY(), layer->getMaxWidth());
    }
  };
  for (auto* box : wire->getWires()) {
    if (box->isVia()) {
      ++vias;
      const auto boxes = box->getBlockVia() != nullptr
                             ? box->getBlockVia()->getBoxes()
                             : box->getTechVia()->getBoxes();
      for (auto* via_box : boxes) {
        check_width(via_box);
      }
    } else {
      check_width(box);
    }
  }
  EXPECT_EQ(vias, 2);
  EXPECT_EQ(pad_->getShapeCount(), 2);
}

TEST_F(TestPDNPadHop, PreservesDirectConnection)
{
  target(m5_, odb::Rect(2000, 2000, 2200, 8000));
  build();
  ASSERT_EQ(pad_->getShapeCount(), 1);
  const auto direct = *pad_->getShapes().at(m4_).begin();
  EXPECT_EQ(direct->getRect(), odb::Rect(800, 4300, 2200, 4700));
  EXPECT_TRUE(direct->hasITermConnections());
  EXPECT_FALSE(write()->getWires().empty());
}

TEST_F(TestPDNPadHop, RejectsNearerMacroGridTarget)
{
  class MacroGrid : public TestGrid
  {
   public:
    using TestGrid::TestGrid;
    Type type() const override { return kInstance; }
  };
  MacroGrid macro(domain_.get(), "macro", true, {});
  TestGridComponent component(&macro);
  const auto near = target(m4_, odb::Rect(1600, 2000, 1800, 8000));
  near->setGridComponent(&component);
  target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  build();
  ASSERT_EQ(pad_->getShapeCount(), 2);
  EXPECT_EQ((*pad_->getShapes().at(m5_).begin())->getRect().xMax(), 2200);
}

TEST_F(TestPDNPadHop, PadObstructionBehindEdgeDoesNotCutEscape)
{
  target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  auto obs = std::make_shared<Shape>(
      m4_, odb::Rect(100, 4000, 1000, 5000), Shape::kPadObs);
  obs->generateObstruction();
  obstructions_[m4_].insert(obs);
  build();
  EXPECT_EQ(pad_->getShapeCount(), 2);
  EXPECT_FALSE(write()->getWires().empty());
}

TEST_F(TestPDNPadHop, FailedTargetViaRemovesEscapeRouteAndFirstVia)
{
  target(m4_, odb::Rect(2000, 2000, 2060, 8000));
  build();
  ASSERT_EQ(pad_->getShapeCount(), 2);
  EXPECT_TRUE(write()->getWires().empty());
  EXPECT_EQ(pad_->getShapeCount(), 0);
}

TEST_F(TestPDNPadHop, ObstructedTargetHopRemovesSurvivingFirstVia)
{
  target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  auto obs = std::make_shared<Shape>(
      stack_.cuts[3], odb::Rect(1900, 4200, 2300, 4800), Shape::kObs);
  obstructions_[stack_.cuts[3]].insert(obs);
  build();
  ASSERT_EQ(pad_->getShapeCount(), 2);
  Grid::ViaShapeMap via_shapes;
  EXPECT_TRUE(write(&via_shapes)->getWires().empty());
  EXPECT_TRUE(via_shapes.empty());
  EXPECT_EQ(pad_->getShapeCount(), 0);
}

TEST_F(TestPDNPadHop, CutRouteRemovesBothSegments)
{
  target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  pad_->makeShapes(shapes_);
  auto obs = std::make_shared<Shape>(
      m5_, odb::Rect(1500, 4000, 1600, 5000), Shape::kObs);
  obs->generateObstruction();
  obstructions_[m5_].insert(obs);
  pad_->cutShapes(obstructions_);
  EXPECT_EQ(pad_->getShapeCount(), 0);
}

TEST_F(TestPDNPadHop, ReplacedModifiableTargetKeepsConnectedHop)
{
  class TargetStraps : public Straps
  {
   public:
    using GridComponent::addShape;
    using Straps::Straps;
  };
  auto targets = std::make_unique<TargetStraps>(grid_.get(), m4_, 200, 0);
  auto* component = targets.get();
  const auto stripe = component->addShape(
      std::make_unique<Shape>(m4_,
                              power_,
                              odb::Rect(2000, 2000, 2200, 8000),
                              odb::dbWireShapeType::STRIPE));
  grid_->addStrap(std::move(targets));
  shapes_[m4_].insert(stripe);
  pad_->makeShapes(shapes_);
  pad_->cutShapes(obstructions_);
  grid_->makeVias(shapes_, obstructions_);
  ASSERT_EQ(pad_->getShapeCount(), 2);
  component->replaceShape(stripe.get(), stripe->copy());
  const auto replacement = *component->getShapes().at(m4_).begin();
  ASSERT_NE(replacement, stripe);
  auto* wire = write();
  int vias = 0;
  for (auto* box : wire->getWires()) {
    vias += box->isVia();
  }
  EXPECT_EQ(vias, 2);
  EXPECT_EQ(pad_->getShapeCount(), 2);
}

TEST_F(TestPDNPadHop, ReplacementAndResetPreservePairOwnership)
{
  target(m4_, odb::Rect(2000, 2000, 2200, 8000));
  build();
  auto* component = static_cast<GridComponent*>(pad_);
  const auto escape = *pad_->getShapes().at(m4_).begin();
  component->replaceShape(escape.get(), escape->copy());
  EXPECT_EQ(pad_->getShapeCount(), 2);
  EXPECT_FALSE(write()->getWires().empty());
  component->clearShapes();
  EXPECT_EQ(pad_->getShapeCount(), 0);
  EXPECT_FALSE(pad_->removeFailedPadHops());
}

}  // namespace
}  // namespace pdn
