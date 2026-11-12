// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <memory>
#include <vector>

#include "PdnTest.h"
#include "connect.h"
#include "domain.h"
#include "gmock/gmock.h"
#include "grid.h"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "pdn/PdnGen.hh"
#include "rings.h"
#include "shape.h"
#include "straps.h"
#include "techlayer.h"
#include "tst/physical_builder.h"
#include "via.h"

namespace pdn {
namespace {

class FixedStrap : public Straps
{
 public:
  FixedStrap(Grid* grid,
             odb::dbTechLayer* layer,
             odb::dbNet* net,
             const odb::Rect& rect = {1000, 4500, 4000, 4700})
      : Straps(grid, layer, rect.minDXDY(), 0, 0), net_(net), rect_(rect)
  {
    setNets({net});
  }

  void makeShapes(const Shape::ShapeTreeMap&) override
  {
    addShape(std::make_unique<Shape>(
        getLayer(), net_, rect_, odb::dbWireShapeType::STRIPE));
  }

 private:
  odb::dbNet* net_;
  odb::Rect rect_;
};

class FixedRing : public Rings
{
 public:
  FixedRing(Grid* grid,
            odb::dbTechLayer* layer,
            odb::dbNet* net,
            const odb::Rect& rect)
      : Rings(grid, {layer, 200, 200}, {layer, 200, 200}),
        layer_(layer),
        net_(net),
        rect_(rect)
  {
    setNets({net});
  }

  void makeShapes(const Shape::ShapeTreeMap&) override
  {
    auto ring = std::make_unique<Shape>(
        layer_, net_, rect_, odb::dbWireShapeType::RING);
    ring->setLocked();
    addShape(std::move(ring));
  }

 private:
  odb::dbTechLayer* layer_;
  odb::dbNet* net_;
  odb::Rect rect_;
};

class TestPDNMacroRing : public PdnTest
{
 protected:
  void SetUp() override
  {
    PdnTest::SetUp();
    stack_ = makeBasicViaStack("", 50, 10, odb::dbTechLayerDir::VERTICAL);
    block()->setDieArea(odb::Rect(0, 0, 10000, 10000));
    block()->setCoreArea(odb::Rect(1000, 1000, 9000, 9000));
    power_ = odb::dbNet::create(block(), "VDD");
    power_->setSigType(odb::dbSigType::POWER);
    ground_ = odb::dbNet::create(block(), "VSS");
    ground_->setSigType(odb::dbSigType::GROUND);
    pdngen_ = std::make_unique<PdnGen>(db_.get(), getLogger());
    pdngen_->setCoreDomain(power_, nullptr, ground_, {});
    domain_ = pdngen_->getDomains().front();
    auto core = std::make_unique<CoreGrid>(
        domain_, "core", true, std::vector<odb::dbTechLayer*>{});
    core_ = core.get();
    auto strap = std::make_unique<FixedStrap>(core_, stack_.top, power_);
    strap_ = strap.get();
    core_->addStrap(std::move(strap));
    core_->addRing(std::make_unique<FixedRing>(
        core_, stack_.bottom, power_, odb::Rect(1000, 1000, 1200, 9000)));
    domain_->addGrid(std::move(core));
    auto* inst = makeMacro("macro", 2000);
    inst->setLocation(5000, 3000);
    inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
    auto macro = std::make_unique<InstanceGrid>(
        domain_, "macro", true, inst, std::vector<odb::dbTechLayer*>{});
    macro_ = macro.get();
    macro_->addHalo({1000, 1000, 1000, 1000});
    domain_->addGrid(std::move(macro));
  }

  void addMacroRing(odb::dbNet* net,
                    const odb::Rect& rect = {4500, 2500, 4700, 7500})
  {
    macro_->addRing(
        std::make_unique<FixedRing>(macro_, stack_.bottom, net, rect));
  }

  odb::dbInst* makeMacro(const char* name, int width)
  {
    const std::vector<tst::PinSpec> pins{{.name = "VDD",
                                          .io_type = odb::dbIoType::INOUT,
                                          .layer = stack_.bottom,
                                          .rect = odb::Rect(50, 50, 150, 150),
                                          .sig_type = odb::dbSigType::POWER},
                                         {.name = "VSS",
                                          .io_type = odb::dbIoType::INOUT,
                                          .layer = stack_.bottom,
                                          .rect = odb::Rect(250, 50, 350, 150),
                                          .sig_type = odb::dbSigType::GROUND}};
    auto* master = tst::makeMaster(*db_->getLibs().begin(),
                                   name,
                                   width,
                                   4000,
                                   pins,
                                   odb::dbMasterType::BLOCK);
    auto* inst = odb::dbInst::create(block(), master, name);
    inst->findITerm("VDD")->connect(power_);
    inst->findITerm("VSS")->connect(ground_);
    return inst;
  }

  void addConnect(Grid* grid)
  {
    grid->addConnect(
        std::make_unique<Connect>(grid, stack_.bottom, stack_.top));
  }

  static std::vector<ViaPtr> vias(Grid* grid)
  {
    std::vector<ViaPtr> result;
    grid->getVias(result);
    return result;
  }

  static std::vector<odb::dbSBox*> placedVias(odb::dbNet* net)
  {
    std::vector<odb::dbSBox*> result;
    for (auto* wire : net->getSWires()) {
      for (auto* box : wire->getWires()) {
        if (box->isVia()) {
          result.push_back(box);
        }
      }
    }
    return result;
  }

  void expectUnchanged()
  {
    EXPECT_THAT(strap_->getShapeRects(),
                testing::ElementsAre(odb::Rect(1000, 4500, 4000, 4700)));
    EXPECT_TRUE(vias(macro_).empty());
    const auto anchor = vias(core_);
    ASSERT_EQ(anchor.size(), 1u);
    EXPECT_EQ(anchor[0]->getGrid(), core_);
    EXPECT_EQ(anchor[0]->getArea(), odb::Rect(1000, 4500, 1200, 4700));
  }

  static void expectAttachments(const std::vector<Grid*>& grids)
  {
    std::vector<ViaPtr> connections;
    std::vector<ShapePtr> shapes;
    for (auto* grid : grids) {
      grid->getVias(connections);
      for (const auto& [layer, layer_shapes] : grid->getShapes()) {
        shapes.insert(shapes.end(), layer_shapes.begin(), layer_shapes.end());
      }
    }
    for (const auto& via : connections) {
      EXPECT_THAT(shapes, testing::Contains(via->getLowerShape()));
      EXPECT_THAT(shapes, testing::Contains(via->getUpperShape()));
    }
    for (const auto& shape : shapes) {
      std::vector<ViaPtr> expected;
      for (const auto& via : connections) {
        if (via->getLowerShape() == shape || via->getUpperShape() == shape) {
          expected.push_back(via);
        }
      }
      EXPECT_THAT(shape->getVias(),
                  testing::UnorderedElementsAreArray(expected));
    }
  }

  ViaStack stack_;
  odb::dbNet* power_;
  odb::dbNet* ground_;
  std::unique_ptr<PdnGen> pdngen_;
  VoltageDomain* domain_;
  CoreGrid* core_;
  InstanceGrid* macro_;
  FixedStrap* strap_;
};

TEST_F(TestPDNMacroRing, ConnectsThroughHaloAndSurvivesTrimming)
{
  addMacroRing(power_);
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(true);
  EXPECT_THAT(strap_->getShapeRects(),
              testing::ElementsAre(odb::Rect(1000, 4500, 4700, 4700)));
  const auto anchor = vias(core_);
  ASSERT_EQ(anchor.size(), 1u);
  EXPECT_EQ(anchor[0]->getGrid(), core_);
  EXPECT_EQ(anchor[0]->getArea(), odb::Rect(1000, 4500, 1200, 4700));
  const auto contacts = vias(macro_);
  ASSERT_EQ(contacts.size(), 1u);
  const auto& contact = contacts[0];
  ASSERT_TRUE(contact->isValid());
  EXPECT_EQ(contact->getGrid(), macro_);
  EXPECT_EQ(contact->getNet(), power_);
  EXPECT_EQ(contact->getLowerLayer(), stack_.bottom);
  EXPECT_EQ(contact->getUpperLayer(), stack_.top);
  EXPECT_EQ(contact->getLowerShape()->getGridComponent()->getGrid(), macro_);
  EXPECT_EQ(contact->getUpperShape()->getGridComponent()->getGrid(), core_);
  EXPECT_EQ(contact->getArea(), odb::Rect(4500, 4500, 4700, 4700));
  expectAttachments({core_, macro_});

  pdngen_->writeToDb(false);
  EXPECT_FALSE(contact->isFailed());
  EXPECT_FALSE(anchor[0]->isFailed());
  const auto emitted = placedVias(power_);
  ASSERT_EQ(emitted.size(), 2u);
  int macro_contacts = 0;
  for (auto* box : emitted) {
    if (!contact->getArea().intersects(box->getViaXY())) {
      continue;
    }
    ++macro_contacts;
    std::vector<odb::dbShape> boxes;
    box->getViaBoxes(boxes);
    odb::Rect cuts;
    cuts.mergeInit();
    odb::Rect bottom;
    bottom.mergeInit();
    odb::Rect top;
    top.mergeInit();
    for (const auto& shape : boxes) {
      if (shape.getTechLayer() == stack_.cut) {
        cuts.merge(shape.getBox());
      } else if (shape.getTechLayer() == stack_.bottom) {
        bottom.merge(shape.getBox());
      } else if (shape.getTechLayer() == stack_.top) {
        top.merge(shape.getBox());
      }
    }
    ASSERT_FALSE(cuts.isInverted());
    for (const auto& landing : {bottom, top}) {
      ASSERT_FALSE(landing.isInverted());
      EXPECT_LE(landing.xMin() + 10, cuts.xMin());
      EXPECT_LE(landing.yMin() + 10, cuts.yMin());
      EXPECT_GE(landing.xMax() - 10, cuts.xMax());
      EXPECT_GE(landing.yMax() - 10, cuts.yMax());
    }
    EXPECT_TRUE(contact->getLowerShape()->getRect().contains(bottom));
    EXPECT_TRUE(contact->getUpperShape()->getRect().contains(top));
    bool emitted_stripe = false;
    for (auto* wire : power_->getSWires()) {
      for (auto* metal : wire->getWires()) {
        if (!metal->isVia() && metal->getTechLayer() == stack_.top
            && metal->getBox().contains(odb::Rect(1000, 4500, 4700, 4700))) {
          emitted_stripe = true;
        }
      }
    }
    EXPECT_TRUE(emitted_stripe);
  }
  EXPECT_EQ(macro_contacts, 1);
}

TEST_F(TestPDNMacroRing, ConnectsBothEndsAndSurvivesTrimming)
{
  addMacroRing(power_);
  addConnect(core_);
  addConnect(macro_);
  auto* inst = makeMacro("left_macro", 400);
  inst->setLocation(200, 3000);
  inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  auto left = std::make_unique<InstanceGrid>(
      domain_, "left_macro", true, inst, std::vector<odb::dbTechLayer*>{});
  auto* left_macro = left.get();
  left->addHalo({100, 100, 100, 100});
  left->addRing(std::make_unique<FixedRing>(
      left_macro, stack_.bottom, power_, odb::Rect(700, 2500, 900, 7500)));
  addConnect(left_macro);
  domain_->addGrid(std::move(left));
  pdngen_->buildGrids(true);
  EXPECT_THAT(strap_->getShapeRects(),
              testing::ElementsAre(odb::Rect(700, 4500, 4700, 4700)));
  const auto left_contacts = vias(left_macro);
  ASSERT_EQ(left_contacts.size(), 1u);
  EXPECT_EQ(left_contacts[0]->getGrid(), left_macro);
  EXPECT_EQ(left_contacts[0]->getArea(), odb::Rect(700, 4500, 900, 4700));
  EXPECT_EQ(left_contacts[0]->getLowerShape()->getGridComponent()->getGrid(),
            left_macro);
  EXPECT_EQ(left_contacts[0]->getUpperShape()->getGridComponent()->getGrid(),
            core_);
  const auto right_contacts = vias(macro_);
  ASSERT_EQ(right_contacts.size(), 1u);
  EXPECT_EQ(right_contacts[0]->getGrid(), macro_);
  EXPECT_EQ(right_contacts[0]->getArea(), odb::Rect(4500, 4500, 4700, 4700));
  EXPECT_EQ(right_contacts[0]->getLowerShape()->getGridComponent()->getGrid(),
            macro_);
  EXPECT_EQ(right_contacts[0]->getUpperShape()->getGridComponent()->getGrid(),
            core_);
  expectAttachments({core_, macro_, left_macro});
  pdngen_->writeToDb(false);
  const auto emitted = placedVias(power_);
  ASSERT_EQ(emitted.size(), 3u);
  for (const auto& area : {odb::Rect(700, 4500, 900, 4700),
                           odb::Rect(1000, 4500, 1200, 4700),
                           odb::Rect(4500, 4500, 4700, 4700)}) {
    EXPECT_EQ(std::count_if(emitted.begin(),
                            emitted.end(),
                            [&area](odb::dbSBox* via) {
                              return area.intersects(via->getViaXY());
                            }),
              1);
  }
  EXPECT_FALSE(left_contacts[0]->isFailed());
  EXPECT_FALSE(right_contacts[0]->isFailed());
}

TEST_F(TestPDNMacroRing, CoreConnectAloneDoesNotExtend)
{
  addMacroRing(power_);
  addConnect(core_);
  pdngen_->buildGrids(false);
  expectUnchanged();
}

TEST_F(TestPDNMacroRing, SameLayerRingDoesNotExtend)
{
  macro_->addRing(std::make_unique<FixedRing>(
      macro_, stack_.top, power_, odb::Rect(4500, 2500, 4700, 7500)));
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(false);
  expectUnchanged();
}

TEST_F(TestPDNMacroRing, UnrelatedMacroConnectDoesNotExtendUpperStripe)
{
  addMacroRing(power_);
  macro_->addRing(std::make_unique<FixedRing>(
      macro_, stack_.top, power_, odb::Rect(4500, 2500, 7500, 2700)));
  addConnect(core_);
  addConnect(macro_);
  const auto upper_stack = addStackedLevel(stack_);
  auto upper = std::make_unique<CoreGrid>(
      domain_, "upper", true, std::vector<odb::dbTechLayer*>{});
  auto* upper_grid = upper.get();
  auto stripe = std::make_unique<FixedStrap>(
      upper_grid, upper_stack.top, power_, odb::Rect(4500, 1000, 4700, 2000));
  auto* upper_stripe = stripe.get();
  upper->addStrap(std::move(stripe));
  domain_->addGrid(std::move(upper));

  pdngen_->buildGrids(false);
  EXPECT_THAT(upper_stripe->getShapeRects(),
              testing::ElementsAre(odb::Rect(4500, 1000, 4700, 2000)));
  EXPECT_TRUE(vias(upper_grid).empty());
  const auto contacts = vias(macro_);
  ASSERT_FALSE(contacts.empty());
  for (const auto& via : contacts) {
    EXPECT_EQ(via->getLowerLayer(), stack_.bottom);
    EXPECT_EQ(via->getUpperLayer(), stack_.top);
  }
  expectAttachments({core_, macro_, upper_grid});
}

TEST_F(TestPDNMacroRing, DifferentNetDoesNotExtend)
{
  addMacroRing(ground_);
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(false);
  expectUnchanged();
}

TEST_F(TestPDNMacroRing, DifferentDomainDoesNotExtend)
{
  auto other_domain
      = std::make_unique<VoltageDomain>(pdngen_.get(),
                                        block(),
                                        power_,
                                        ground_,
                                        std::vector<odb::dbNet*>{},
                                        getLogger());
  macro_->setDomain(other_domain.get());
  addMacroRing(power_);
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(false);
  expectUnchanged();
  macro_->setDomain(domain_);
}

TEST_F(TestPDNMacroRing, DoesNotCrossMacroBodyToFarRing)
{
  addMacroRing(power_, {7300, 2500, 7500, 7500});
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(false);
  expectUnchanged();
}

TEST_F(TestPDNMacroRing, RoutingObstructionStopsExtension)
{
  addMacroRing(power_);
  addConnect(core_);
  addConnect(macro_);
  odb::dbObstruction::create(block(), stack_.top, 4200, 4000, 4300, 5000);
  pdngen_->buildGrids(false);
  expectUnchanged();
}

TEST_F(TestPDNMacroRing, CutObstructionPreventsEmittedContact)
{
  addMacroRing(power_);
  addConnect(core_);
  addConnect(macro_);
  odb::dbObstruction::create(block(), stack_.cut, 4490, 4490, 4710, 4710);
  pdngen_->buildGrids(false);
  EXPECT_THAT(strap_->getShapeRects(),
              testing::ElementsAre(odb::Rect(1000, 4500, 4700, 4700)));
  EXPECT_TRUE(vias(macro_).empty());
  pdngen_->writeToDb(false);
  const auto emitted = placedVias(power_);
  ASSERT_EQ(emitted.size(), 1u);
  EXPECT_TRUE(
      odb::Rect(1000, 4500, 1200, 4700).intersects(emitted[0]->getViaXY()));
}

TEST_F(TestPDNMacroRing, LockedRingTooNarrowForViaEmitsNoContact)
{
  addMacroRing(power_, {4500, 2500, 4540, 7500});
  addConnect(core_);
  addConnect(macro_);
  pdngen_->buildGrids(false);
  const auto contacts = vias(macro_);
  ASSERT_EQ(contacts.size(), 1u);
  pdngen_->writeToDb(false);
  EXPECT_TRUE(contacts[0]->isFailed());
  const auto emitted = placedVias(power_);
  ASSERT_EQ(emitted.size(), 1u);
  EXPECT_TRUE(
      odb::Rect(1000, 4500, 1200, 4700).intersects(emitted[0]->getViaXY()));
}

TEST_F(TestPDNMacroRing, PreservesOwnedContactsAndAttachmentsAcrossBuilds)
{
  addMacroRing(power_);
  macro_->addRing(std::make_unique<FixedRing>(
      macro_, stack_.top, power_, odb::Rect(4500, 2500, 7500, 2700)));
  addMacroRing(ground_, {7300, 2500, 7500, 7500});
  macro_->addRing(std::make_unique<FixedRing>(
      macro_, stack_.top, ground_, odb::Rect(7000, 7300, 7500, 7500)));
  addConnect(core_);
  addConnect(macro_);
  const auto upper_stack = addStackedLevel(stack_);
  auto foreign = std::make_unique<CoreGrid>(
      domain_, "foreign", true, std::vector<odb::dbTechLayer*>{});
  auto* foreign_grid = foreign.get();
  foreign->addRing(
      std::make_unique<FixedRing>(foreign_grid,
                                  upper_stack.top,
                                  power_,
                                  odb::Rect(2500, 1000, 2700, 9000)));
  foreign->addConnect(std::make_unique<Connect>(
      foreign_grid, upper_stack.bottom, upper_stack.top));
  domain_->addGrid(std::move(foreign));

  for (int build = 0; build < 2; ++build) {
    SCOPED_TRACE(build);
    pdngen_->buildGrids(false);
    EXPECT_THAT(strap_->getShapeRects(),
                testing::ElementsAre(odb::Rect(1000, 4500, 4700, 4700)));
    const auto anchor = vias(core_);
    ASSERT_EQ(anchor.size(), 1u);
    EXPECT_EQ(anchor[0]->getGrid(), core_);
    EXPECT_EQ(anchor[0]->getArea(), odb::Rect(1000, 4500, 1200, 4700));
    const auto contacts = vias(macro_);
    ASSERT_EQ(contacts.size(), 4u);
    for (const auto& via : contacts) {
      ASSERT_TRUE(via->isValid());
      EXPECT_EQ(via->getGrid(), macro_);
      EXPECT_EQ(via->getLowerShape()->getGridComponent()->getGrid(), macro_);
      if (via->getArea() == odb::Rect(4500, 4500, 4700, 4700)) {
        EXPECT_EQ(via->getUpperShape()->getGridComponent()->getGrid(), core_);
        EXPECT_EQ(via->getNet(), power_);
      } else {
        EXPECT_EQ(via->getUpperShape()->getGridComponent()->getGrid(), macro_);
        if (via->getArea() == odb::Rect(5050, 2500, 5150, 2700)) {
          EXPECT_EQ(via->getLowerShape()->getRect(),
                    odb::Rect(5050, 2500, 5150, 3150));
          EXPECT_EQ(via->getUpperShape()->getRect(),
                    odb::Rect(4500, 2500, 7500, 2700));
        }
      }
    }
    for (const auto& area : {odb::Rect(4500, 4500, 4700, 4700),
                             odb::Rect(4500, 2500, 4700, 2700),
                             odb::Rect(5050, 2500, 5150, 2700)}) {
      EXPECT_EQ(std::count_if(contacts.begin(),
                              contacts.end(),
                              [this, &area](const ViaPtr& via) {
                                return via->getNet() == power_
                                       && via->getArea() == area;
                              }),
                1);
    }
    EXPECT_EQ(std::count_if(contacts.begin(),
                            contacts.end(),
                            [this](const ViaPtr& via) {
                              return via->getNet() == ground_
                                     && via->getArea()
                                            == odb::Rect(
                                                7300, 7300, 7500, 7500);
                            }),
              1);
    const auto foreign_contacts = vias(foreign_grid);
    ASSERT_EQ(foreign_contacts.size(), 1u);
    EXPECT_EQ(foreign_contacts[0]->getGrid(), foreign_grid);
    EXPECT_EQ(foreign_contacts[0]->getArea(),
              odb::Rect(2500, 4500, 2700, 4700));
    EXPECT_EQ(
        foreign_contacts[0]->getLowerShape()->getGridComponent()->getGrid(),
        core_);
    EXPECT_EQ(
        foreign_contacts[0]->getUpperShape()->getGridComponent()->getGrid(),
        foreign_grid);
    expectAttachments({core_, macro_, foreign_grid});
  }
  pdngen_->writeToDb(false);
  EXPECT_EQ(placedVias(power_).size(), 5u);
  EXPECT_EQ(placedVias(ground_).size(), 1u);
  for (auto* grid : {static_cast<Grid*>(core_),
                     static_cast<Grid*>(macro_),
                     static_cast<Grid*>(foreign_grid)}) {
    for (const auto& via : vias(grid)) {
      EXPECT_FALSE(via->isFailed());
    }
  }
}

}  // namespace
}  // namespace pdn
