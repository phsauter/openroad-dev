// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "PdnTest.h"
#include "connect.h"
#include "grid.h"
#include "pdn/PdnGen.hh"
#include "rings.h"
#include "straps.h"
#include "techlayer.h"

namespace pdn {
namespace {

class PinGrid : public InstanceGrid
{
 public:
  using InstanceGrid::getIntersections;
  using InstanceGrid::InstanceGrid;
};

class TestMacroGroup : public PdnTest
{
 protected:
  void SetUp() override
  {
    PdnTest::SetUp();
    m1_ = tst::addRoutingLayer(tech(), "M1");
    tst::addCutLayer(tech(), "V1");
    m2_ = tst::addRoutingLayer(
        tech(), "M2", {.direction = odb::dbTechLayerDir::VERTICAL});
    power_ = odb::dbNet::create(block(), "VDD");
    power_->setSigType(odb::dbSigType::POWER);
    ground_ = odb::dbNet::create(block(), "VSS");
    ground_->setSigType(odb::dbSigType::GROUND);
    block()->setDieArea(odb::Rect(0, 0, 12000, 12000));
    gen_ = std::make_unique<PdnGen>(db_.get(), getLogger());
    gen_->setCoreDomain(power_, nullptr, ground_, {});
    domain_ = gen_->getDomains().front();
    master_ = tst::makeMaster(db_->findLib("lib"),
                              "macro",
                              2000,
                              2000,
                              {{.name = "VDD",
                                .layer = m1_,
                                .rect = odb::Rect(0, 100, 2000, 200),
                                .sig_type = odb::dbSigType::POWER},
                               {.name = "VSS",
                                .layer = m1_,
                                .rect = odb::Rect(0, 300, 2000, 400),
                                .sig_type = odb::dbSigType::GROUND}},
                              odb::dbMasterType::BLOCK);
  }

  odb::dbInst* place(const char* name,
                     int x,
                     int y,
                     odb::dbOrientType orient = odb::dbOrientType::R0)
  {
    auto* inst = odb::dbInst::create(block(), master_, name);
    inst->setOrient(orient);
    inst->setLocation(x, y);
    inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
    inst->findITerm("VDD")->connect(power_);
    inst->findITerm("VSS")->connect(ground_);
    return inst;
  }

  std::unique_ptr<Rings> rings(InstanceGrid& grid, bool single_layer = false)
  {
    auto ring = std::make_unique<Rings>(
        &grid,
        Rings::Layer{m1_, 100, 100},
        Rings::Layer{single_layer ? m1_ : m2_, 100, 100});
    ring->setOffset({200, 200, 200, 200});
    ring->makeShapes({});
    return ring;
  }

  odb::dbInst* placeRect(const char* name, const odb::Rect& rect)
  {
    const std::string master_name = std::string(name) + "_master";
    auto* master = tst::makeMaster(db_->findLib("lib"),
                                   master_name.c_str(),
                                   rect.dx(),
                                   rect.dy(),
                                   {{.name = "VDD",
                                     .layer = m1_,
                                     .rect = odb::Rect(0, 100, rect.dx(), 200),
                                     .sig_type = odb::dbSigType::POWER},
                                    {.name = "VSS",
                                     .layer = m1_,
                                     .rect = odb::Rect(0, 300, rect.dx(), 400),
                                     .sig_type = odb::dbSigType::GROUND}},
                                   odb::dbMasterType::BLOCK);
    auto* inst = odb::dbInst::create(block(), master, name);
    inst->setLocation(rect.xMin(), rect.yMin());
    inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
    inst->findITerm("VDD")->connect(power_);
    inst->findITerm("VSS")->connect(ground_);
    return inst;
  }

  void expectPdnError(const std::string& code,
                      const std::function<void()>& action)
  {
    try {
      action();
      FAIL() << "Expected " << code;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find(code), std::string::npos);
    }
  }

  Grid* owner(odb::dbInst* inst) const
  {
    for (auto* grid : gen_->getGrids()) {
      if (grid->type() == Grid::kInstance
          && grid->getInstances().contains(inst)) {
        return grid;
      }
    }
    return nullptr;
  }

  void assign(const char* name,
              const std::vector<odb::dbInst*>& insts,
              bool replaceable = false)
  {
    gen_->makeInstanceGrid(
        domain_, name, kPower, insts, {}, true, replaceable, {}, false);
  }

  std::unique_ptr<PdnGen> gen_;
  VoltageDomain* domain_ = nullptr;
  odb::dbMaster* master_ = nullptr;
  odb::dbTechLayer* m1_ = nullptr;
  odb::dbTechLayer* m2_ = nullptr;
  odb::dbNet* power_ = nullptr;
  odb::dbNet* ground_ = nullptr;
};

class TestMacroGroupUOpening
    : public TestMacroGroup,
      public testing::WithParamInterface<std::pair<int, bool>>
{
};

TEST_P(TestMacroGroupUOpening, RingGapThresholds)
{
  const auto [width, rejects] = GetParam();
  auto* base = placeRect("base", odb::Rect(2000, 2000, 10000, 4000));
  auto* left = placeRect("left", odb::Rect(2000, 4000, 4000, 10000));
  auto* right = placeRect("right", odb::Rect(4000 + width, 4000, 10000, 10000));
  InstanceGrid group(domain_, "group", true, {base, left, right}, {});
  if (rejects) {
    expectPdnError("PDN-0246", [&] { rings(group); });
  } else {
    auto ring = rings(group);
    EXPECT_FALSE(ring->getShapes().at(m1_).empty());
    EXPECT_FALSE(ring->getShapes().at(m2_).empty());
  }
}

INSTANTIATE_TEST_SUITE_P(RingWidths,
                         TestMacroGroupUOpening,
                         testing::Values(std::pair{350, false},
                                         std::pair{400, false},
                                         std::pair{405, true},
                                         std::pair{650, true},
                                         std::pair{700, false},
                                         std::pair{795, false},
                                         std::pair{800, false},
                                         std::pair{805, true},
                                         std::pair{1000, true},
                                         std::pair{1095, true},
                                         std::pair{1100, false},
                                         std::pair{1105, false}));

TEST_F(TestMacroGroup, RejectsClosedRingHole)
{
  auto* left = placeRect("left", odb::Rect(2000, 2000, 4000, 10000));
  auto* right = placeRect("right", odb::Rect(8000, 2000, 10000, 10000));
  auto* bottom = placeRect("bottom", odb::Rect(4000, 2000, 8000, 4000));
  auto* top = placeRect("top", odb::Rect(4000, 8000, 8000, 10000));
  InstanceGrid group(domain_, "closed", true, {left, right, bottom, top}, {});
  expectPdnError("PDN-0245", [&] { rings(group); });
}

TEST_F(TestMacroGroup, RingOffsetCannotSealOpenHole)
{
  auto* left = placeRect("left", odb::Rect(2000, 2000, 4000, 10000));
  auto* bottom = placeRect("bottom", odb::Rect(4000, 2000, 10000, 4000));
  auto* top = placeRect("top", odb::Rect(4000, 8000, 10000, 10000));
  auto* bottom_right
      = placeRect("bottom_right", odb::Rect(8000, 4000, 10000, 5850));
  auto* top_right = placeRect("top_right", odb::Rect(8000, 6150, 10000, 8000));
  InstanceGrid group(domain_,
                     "offset_closed",
                     true,
                     {left, bottom, top, bottom_right, top_right},
                     {});
  try {
    rings(group);
    FAIL() << "Expected PDN-0245 or PDN-0246";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_TRUE(message.find("PDN-0245") != std::string::npos
                || message.find("PDN-0246") != std::string::npos);
  }
}

TEST_F(TestMacroGroup, RejectsNarrowInwardStep)
{
  auto* lower = placeRect("lower", odb::Rect(2000, 2000, 10000, 6000));
  auto* upper = placeRect("upper", odb::Rect(2000, 6000, 9995, 10000));
  InstanceGrid group(domain_, "step", true, {lower, upper}, {});
  expectPdnError("PDN-0246", [&] { rings(group); });
}

TEST_F(TestMacroGroup, RejectsTouchingWallsWithZeroConfiguredSpacing)
{
  auto* base = placeRect("base", odb::Rect(2000, 2000, 10000, 4000));
  auto* left = placeRect("left", odb::Rect(2000, 4000, 4000, 10000));
  auto* right = placeRect("right", odb::Rect(4600, 4000, 10000, 10000));
  InstanceGrid group(domain_, "zero_spacing", true, {base, left, right}, {});
  Rings ring(&group, Rings::Layer{m1_, 100, 0}, Rings::Layer{m2_, 100, 0});
  ring.setOffset({200, 200, 200, 200});
  expectPdnError("PDN-0246", [&] { ring.makeShapes({}); });
}

TEST_F(TestMacroGroup, RejectsGroupRingSegmentsBelowMinimumArea)
{
  m1_->setArea(40000);
  auto* base = placeRect("base", odb::Rect(2000, 2000, 10000, 4000));
  auto* left = placeRect("left", odb::Rect(2000, 4000, 4000, 10000));
  auto* right = placeRect("right", odb::Rect(5100, 4000, 10000, 10000));
  InstanceGrid group(domain_, "minimum_area", true, {base, left, right}, {});
  expectPdnError("PDN-0248", [&] { rings(group); });
}

TEST_F(TestMacroGroup, SingleLayerCoreRingIgnoresPadConnectionInstances)
{
  block()->setCoreArea(odb::Rect(3000, 3000, 9000, 9000));
  auto* pad_a = place("pad_a", 4000, 0);
  auto* pad_b = place("pad_b", 4000, 10000);
  CoreGrid core(domain_, "core", true, {});
  auto net_to_pin_count = std::make_shared<odb::PtrMap<odb::dbNet, int>>();
  for (auto* pad : {pad_a, pad_b}) {
    core.addStrap(std::make_unique<PadDirectConnectionStraps>(
        &core, pad->findITerm("VDD"), std::vector{m1_}, net_to_pin_count));
  }
  ASSERT_EQ(core.getInstances().size(), 2u);
  Rings ring(&core, Rings::Layer{m1_, 100, 100}, Rings::Layer{m1_, 100, 100});
  ring.setOffset({200, 200, 200, 200});
  EXPECT_NO_THROW(ring.makeShapes({}));
  EXPECT_EQ(ring.getShapes().at(m1_).size(), 8u);
}

TEST_F(TestMacroGroup, CornerOnlyMembersUseRectangularFallback)
{
  auto* lower = placeRect("lower", odb::Rect(2000, 2000, 4000, 4000));
  auto* upper = placeRect("upper", odb::Rect(4000, 4000, 6000, 6000));
  InstanceGrid group(domain_, "corners", true, {lower, upper}, {});
  EXPECT_EQ(group.getDomainArea(), odb::Rect(2000, 2000, 6000, 6000));
  EXPECT_EQ(group.getDomainRegion().getEdges().size(), 4u);
  auto ring = rings(group);
  EXPECT_EQ(ring->getShapes().at(m1_).size(), 4u);
  EXPECT_EQ(ring->getShapes().at(m2_).size(), 4u);
}

TEST_F(TestMacroGroup, RejectsSingleLayerRingForGroupButKeepsSingleton)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 6000, 2000);
  InstanceGrid group(domain_, "group", true, std::vector{a, b}, {});
  expectPdnError("PDN-0247", [&] { rings(group, true); });

  InstanceGrid singleton(domain_, "single", true, a, {});
  auto ring = rings(singleton, true);
  EXPECT_FALSE(ring->getShapes().at(m1_).empty());
}

TEST_F(TestMacroGroup, SeparatedMembersHaveOneRectangularPerimeter)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 6000, 2000);
  InstanceGrid group(domain_, "group", true, std::vector{a, b}, {});
  EXPECT_EQ(group.getDomainArea(), odb::Rect(2000, 2000, 8000, 4000));
  EXPECT_EQ(group.getDomainRegion().getEdges().size(), 4u);
  EXPECT_TRUE(group.getGridRegion()
                  .intersect(odb::Rect(4500, 2500, 5500, 3500))
                  .isEmpty());
  auto ring = rings(group);
  EXPECT_EQ(ring->getShapes().at(m1_).size(), 4u);
  EXPECT_EQ(ring->getShapes().at(m2_).size(), 4u);
  ring->clearShapes();
  group.addRing(std::move(ring));
  ShapeVectorMap obs;
  group.getGridLevelObstructions(obs);
  const auto tree = Shape::convertVectorToObstructionTree(obs);
  const odb::Rect gap(4500, 2500, 5500, 3500);
  bool ring_claimed = false;
  for (const auto& [layer, shapes] : tree) {
    for (const auto& shape : shapes) {
      EXPECT_FALSE(shape->getObstruction().overlaps(gap));
      ring_claimed
          |= shape->getRect().overlaps(odb::Rect(4500, 4220, 4600, 4280));
    }
  }
  EXPECT_TRUE(ring_claimed);
}

TEST_F(TestMacroGroup, TouchingMembersKeepConcaveRingsAndFreeHaloNotch)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 4000, 2000);
  auto* c = place("c", 2000, 4000);
  InstanceGrid group(domain_, "group", true, std::vector{a, b, c}, {});
  group.addHalo({100, 200, 300, 400});
  EXPECT_EQ(group.getDomainRegion().getEdges().size(), 6u);
  EXPECT_TRUE(group.getGridRegion().contains(
      Region(odb::Rect(1900, 1800, 6300, 4400))));
  const odb::Rect notch(5000, 5000, 5500, 5500);
  EXPECT_TRUE(group.getGridRegion().intersect(notch).isEmpty());
  auto* site = odb::dbSite::create(db_->findLib("lib"), "site");
  site->setWidth(100);
  site->setHeight(100);
  site->setClass(odb::dbSiteClass::CORE);
  odb::dbRow::create(block(),
                     "notch_row",
                     site,
                     5000,
                     5000,
                     odb::dbOrientType::R0,
                     odb::dbRowDir::HORIZONTAL,
                     5,
                     100);
  EXPECT_NO_THROW(group.checkSetup());
  auto ring = rings(group);
  bool concave_horizontal = false;
  bool concave_vertical = false;
  for (const auto& [layer, shapes] : ring->getShapes()) {
    for (const auto& shape : shapes) {
      const auto& rect = shape->getRect();
      EXPECT_FALSE(rect.overlaps(notch));
      concave_horizontal |= layer == m1_ && rect.yMin() == 4200;
      concave_vertical |= layer == m2_ && rect.xMin() == 4200;
    }
  }
  EXPECT_TRUE(concave_horizontal);
  EXPECT_TRUE(concave_vertical);
  group.addRing(std::move(ring));
  ShapeVectorMap obs;
  group.getGridLevelObstructions(obs);
  for (const auto& [layer, shapes] : obs) {
    for (const auto& shape : shapes) {
      EXPECT_FALSE(shape->getRect().overlaps(notch));
    }
  }
}

TEST_F(TestMacroGroup, EveryMemberContributesPinsNetsAndObstructionExemptions)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 6000, 2000);
  a->findITerm("VSS")->disconnect();
  b->findITerm("VDD")->disconnect();
  PinGrid group(domain_, "group", true, std::vector{a, b}, {});
  EXPECT_EQ(group.getNets(true), (std::vector{power_, ground_}));
  EXPECT_EQ(group.getInstances(), (odb::PtrSet<odb::dbInst>{a, b}));
  EXPECT_EQ(group.getObstructionExemptInstances(), group.getInstances());
  place("unowned", 9000, 2000);
  ShapeVectorMap obs;
  Grid::makeInitialObstructions(
      block(), obs, group.getObstructionExemptInstances(), {}, getLogger());
  ASSERT_FALSE(obs[m1_].empty());
  for (const auto& shape : obs[m1_]) {
    EXPECT_GE(shape->getRect().xMin(), 8900);
  }
  group.addConnect(std::make_unique<Connect>(&group, m1_, m2_));
  Shape::ShapeTreeMap shapes;
  shapes[m2_].insert(
      std::make_shared<Shape>(m2_, power_, odb::Rect(2500, 2000, 2600, 3000)));
  shapes[m2_].insert(
      std::make_shared<Shape>(m2_, ground_, odb::Rect(6500, 2000, 6600, 3000)));
  std::vector<ViaPtr> vias;
  group.getIntersections(vias, shapes);
  ASSERT_EQ(vias.size(), 2u);
  EXPECT_NE(vias[0]->getNet(), vias[1]->getNet());
}

TEST_F(TestMacroGroup, ConflictingMemberLeavesAllDefaultOwnershipIntact)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 6000, 2000);
  assign("default", {a}, true);
  assign("explicit", {b});
  auto* original = owner(a);
  assign("rejected", {a, b});
  EXPECT_EQ(owner(a), original);
  EXPECT_EQ(owner(b)->getName(), "explicit");
  owner(b)->getDomain()->removeGrid(owner(b));
  assign("group", {a, b, a});
  EXPECT_EQ(gen_->getGrids().size(), 1u);
  EXPECT_EQ(owner(a), owner(b));
  EXPECT_EQ(owner(a)->getInstances().size(), 2u);
}

TEST_F(TestMacroGroup, PartialDefaultGroupReplacementKeepsEveryMember)
{
  auto* a = place("a", 2000, 2000);
  auto* b = place("b", 6000, 2000);
  auto* c = place("c", 9000, 2000);
  assign("default", {a, b, c}, true);
  auto* original = owner(a);
  gen_->makeInstanceGrid(
      domain_, "singleton", kPower, a, {}, true, false, {}, false);
  EXPECT_EQ(owner(a), original);
  assign("partial_group", {a, b});
  for (auto* member : {a, b, c}) {
    EXPECT_EQ(owner(member), original);
  }
  EXPECT_EQ(gen_->getGrids().size(), 1u);
  assign("complete_group", {a, b, c});
  EXPECT_EQ(gen_->getGrids().size(), 1u);
  for (auto* member : {a, b, c}) {
    EXPECT_EQ(owner(member)->getName(), "complete_group");
  }
}

TEST_F(TestMacroGroup, NeighborMetalBlocksMacroEdgeConnections)
{
  auto* a = place("a", 2000, 2000);
  a->findITerm("VSS")->disconnect();
  auto* master = tst::makeMaster(db_->findLib("lib"),
                                 "blocker",
                                 2000,
                                 2000,
                                 {{.name = "VDD",
                                   .layer = m1_,
                                   .rect = odb::Rect(100, 1500, 1900, 1600),
                                   .sig_type = odb::dbSigType::POWER}},
                                 odb::dbMasterType::BLOCK);
  odb::dbBox::create(master, m1_, 0, 0, 2000, 1000);
  auto* b = odb::dbInst::create(block(), master, "b");
  b->findITerm("VDD")->connect(power_);
  b->setLocation(5000, 2000);
  b->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  InstanceGrid group(domain_, "group", true, std::vector{a, b}, {});
  auto connect = std::make_unique<Connect>(&group, m1_, m2_);
  connect->addFixedVia(
      tst::addVia(tech(), "via", m1_, tech()->findLayer("V1"), m2_, 50, 25));
  group.addConnect(std::move(connect));
  Shape::ShapeTreeMap targets;
  targets[m2_].insert(
      std::make_shared<Shape>(m2_, power_, odb::Rect(7200, 2000, 7300, 4000)));
  ShapeVectorMap initial;
  group.getGridLevelObstructions(initial);
  auto obs = Shape::convertVectorToObstructionTree(initial);
  MacroEdgeConnectionStraps::connectUnreachedPins(&group, a, targets, obs);
  EXPECT_TRUE(group.getStraps().empty());
  b->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  b->setLocation(5000, 6000);
  b->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  initial.clear();
  group.getGridLevelObstructions(initial);
  obs = Shape::convertVectorToObstructionTree(initial);
  MacroEdgeConnectionStraps::connectUnreachedPins(&group, a, targets, obs);
  EXPECT_FALSE(group.getStraps().empty());
}

TEST_F(TestMacroGroup, PinlessMemberKeepsItsOutlineWithoutAPinBoundary)
{
  auto* a = place("a", 2000, 2000);
  auto* master = tst::makeMaster(
      db_->findLib("lib"), "pinless", 2000, 2000, {}, odb::dbMasterType::BLOCK);
  auto* b = odb::dbInst::create(block(), master, "b");
  b->setLocation(5000, 2000);
  b->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  InstanceGrid group(domain_, "group", true, std::vector{a, b}, {});
  const odb::Rect powered_boundary(2000, 2100, 4000, 2400);
  EXPECT_EQ(group.getDomainBoundary(), powered_boundary);
  EXPECT_EQ(group.getDomainBoundaryRegion().getRects(),
            std::vector{powered_boundary});
  EXPECT_TRUE(group.getDomainRegion().contains(getInstanceOutline(b)));
  group.addHalo({100, 100, 100, 100});
  EXPECT_TRUE(group.getGridRegion().contains(
      getInstanceOutline(b).bloat({100, 100, 100, 100})));
}

TEST_F(TestMacroGroup, DefHalosStaySpecificToEachPlacedMember)
{
  auto* a = place("a", 2000, 2000, odb::dbOrientType::MY);
  auto* b = place("b", 6000, 2000, odb::dbOrientType::R90);
  a->setHalo(100, 200, 300, 400, false);
  b->setHalo(500, 600, 700, 800, false);
  InstanceGrid group(domain_, "group", true, std::vector{a, b}, {});
  EXPECT_EQ(group.getOrientation(), odb::dbOrientType::R0);
  const Region expected
      = getInstanceOutline(a)
            .bloat({300, 200, 100, 400})
            .unite(getInstanceOutline(b).bloat({800, 500, 600, 700}));
  EXPECT_TRUE(group.getGridRegion().contains(expected));
  EXPECT_TRUE(expected.contains(group.getGridRegion()));
  group.addHalo({10, 20, 30, 40});
  const Region user_halo
      = getInstanceOutline(a)
            .bloat({30, 20, 10, 40})
            .unite(getInstanceOutline(b).bloat({40, 10, 20, 30}));
  EXPECT_TRUE(group.getGridRegion().contains(user_halo));
  EXPECT_TRUE(user_halo.contains(group.getGridRegion()));
}

TEST_F(TestMacroGroup, SingletonAndBumpKeepTheirInstanceAndOrientation)
{
  auto* a = place("a", 2000, 2000, odb::dbOrientType::R90);
  InstanceGrid grid(domain_, "single", true, a, {});
  grid.addHalo({100, 200, 300, 400});
  EXPECT_EQ(grid.getInstance(), a);
  EXPECT_EQ(grid.getOrientation(), odb::dbOrientType::R90);
  EXPECT_EQ(grid.getGridRegion().getEnclosingRect(),
            Region(a->getBBox()->getBox())
                .bloat({400, 100, 200, 300})
                .getEnclosingRect());
  BumpGrid bump(domain_, "bump", a);
  EXPECT_EQ(bump.getInstance(), a);
  EXPECT_EQ(bump.getInstances(), grid.getInstances());
}

}  // namespace
}  // namespace pdn
