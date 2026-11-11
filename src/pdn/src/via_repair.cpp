// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#include "via_repair.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "boost/geometry/geometry.hpp"
#include "grid.h"
#include "odb/PtrSetMap.h"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "odb/geom_boost.h"
#include "polygon.h"
#include "utl/Logger.h"
#include "via.h"

namespace pdn {

namespace {

struct ViaTransition
{
  Via* via;
  odb::dbTechLayer* cut_layer;
  odb::PtrSet<odb::dbSBox> boxes;
  odb::PtrMap<odb::dbTechLayer, std::vector<odb::Rect>> metals;
  std::vector<odb::Rect> cuts;
};

class ViaConflictIndex
{
  using Key = std::pair<odb::dbNet*, odb::dbTechLayer*>;
  using Value = std::pair<odb::Rect, size_t>;
  using Tree = bgi::rtree<Value, bgi::quadratic<16>>;

 public:
  explicit ViaConflictIndex(const std::vector<ViaTransition*>& transitions)
      : transitions_(transitions), bounds_(transitions.size())
  {
    for (size_t i = 0; i < transitions.size(); i++) {
      const ViaTransition* transition = transitions[i];
      odb::Rect& bounds = bounds_[i];
      bounds = transition->cuts.front();
      for (const odb::Rect& cut : transition->cuts) {
        bounds.merge(cut);
      }
      trees_[{transition->via->getNet(), transition->cut_layer}].insert(
          {bounds, i});
    }
  }

  const std::vector<size_t>& candidates(size_t i)
  {
    candidates_.clear();
    const ViaTransition* transition = transitions_[i];
    odb::Rect search;
    bounds_[i].bloat(std::max(0, transition->cut_layer->getSpacing() - 1),
                     search);
    const auto& tree
        = trees_.at({transition->via->getNet(), transition->cut_layer});
    for (auto it = tree.qbegin(bgi::intersects(search)); it != tree.qend();
         ++it) {
      if (it->second > i) {
        candidates_.push_back(it->second);
      }
    }
    // Preserve the exhaustive loop's order for conflict and winner processing.
    std::sort(candidates_.begin(), candidates_.end());
    return candidates_;
  }

 private:
  const std::vector<ViaTransition*>& transitions_;
  std::vector<odb::Rect> bounds_;
  std::map<Key, Tree> trees_;
  std::vector<size_t> candidates_;
};

// Preserve the metal union while retiring cut connections.
class ViaRepairRouting
{
  using Key = std::pair<odb::dbNet*, odb::dbTechLayer*>;
  using Value = std::pair<odb::Rect, size_t>;
  using Tree = bgi::rtree<Value, bgi::quadratic<16>>;
  struct LayerRouting
  {
    Tree tree;
    std::vector<int> components;
    int next_component = 0;
  };

 public:
  explicit ViaRepairRouting(
      const odb::PtrMap<odb::dbNet, odb::dbSWire*>& net_map)
  {
    std::map<Key, std::vector<odb::Rect>> ordinary;
    for (const auto& [net, wire] : net_map) {
      for (auto* swire : net->getSWires()) {
        for (auto* box : swire->getWires()) {
          if (box->isVia()) {
            std::vector<odb::dbShape> shapes;
            box->getViaBoxes(shapes);
            for (const auto& shape : shapes) {
              if (shape.getTechLayer()->getType()
                  == odb::dbTechLayerType::ROUTING) {
                insert(net, shape.getTechLayer(), shape.getBox());
              }
            }
          } else {
            insert(net, box->getTechLayer(), box->getBox());
            ordinary[{net, box->getTechLayer()}].push_back(box->getBox());
          }
        }
      }
    }
    for (const auto& [key, rects] : ordinary) {
      ordinary_.emplace(key, Region(rects));
    }
  }

  bool preserves(const ViaTransition& loser, const ViaTransition& winner)
  {
    if (winner.metals.size() != loser.metals.size()) {
      return false;
    }
    for (const auto& [layer, sources] : loser.metals) {
      const auto target = winner.metals.find(layer);
      if (target == winner.metals.end()) {
        return false;
      }
      const Key key{loser.via->getNet(), layer};
      const int component = componentOf(key, target->second.front());
      for (const auto* rects : {&sources, &target->second}) {
        for (const auto& rect : *rects) {
          if (componentOf(key, rect) != component) {
            return false;
          }
        }
      }
    }
    return true;
  }

  void preserveLandings(const ViaTransition& transition,
                        odb::dbSWire* wire) const
  {
    for (const auto& [layer, rects] : transition.metals) {
      const auto ordinary = ordinary_.find({transition.via->getNet(), layer});
      for (const auto& rect : rects) {
        if (ordinary != ordinary_.end()
            && ordinary->second.contains(Region(rect))) {
          continue;
        }
        odb::dbSBox::create(wire,
                            layer,
                            rect.xMin(),
                            rect.yMin(),
                            rect.xMax(),
                            rect.yMax(),
                            odb::dbWireShapeType::DRCFILL);
      }
    }
  }

 private:
  int componentOf(const Key& key, const odb::Rect& seed)
  {
    auto& routing = metals_.at(key);
    const auto first = routing.tree.qbegin(
        bgi::intersects(seed) && bgi::satisfies([&](const Value& value) {
          return seed.overlaps(value.first);
        }));
    const int previous = routing.components[first->second];
    if (previous >= 0) {
      return previous;
    }
    const int component = routing.next_component++;
    std::vector<odb::Rect> connected;
    auto visit = [&](const odb::Rect& rect) {
      for (auto it = routing.tree.qbegin(bgi::intersects(rect));
           it != routing.tree.qend();
           ++it) {
        if (rect.overlaps(it->first) && routing.components[it->second] < 0) {
          routing.components[it->second] = component;
          connected.push_back(it->first);
        }
      }
    };
    visit(first->first);
    for (size_t i = 0; i < connected.size(); i++) {
      // visit can reallocate connected, so keep the query rectangle alive.
      const odb::Rect rect = connected[i];
      visit(rect);
    }
    return component;
  }

  void insert(odb::dbNet* net, odb::dbTechLayer* layer, const odb::Rect& rect)
  {
    auto& routing = metals_[{net, layer}];
    routing.tree.insert({rect, routing.components.size()});
    routing.components.push_back(-1);
  }

  std::map<Key, LayerRouting> metals_;
  std::map<Key, Region> ordinary_;
};

using ViaTransitionMap
    = std::map<std::pair<Via*, odb::dbTechLayer*>, ViaTransition>;

ViaTransitionMap collectViaTransitions(
    const std::map<Via*, odb::PtrSet<odb::dbSBox>>& via_shapes)
{
  ViaTransitionMap transition_map;
  for (const auto& [via, boxes] : via_shapes) {
    for (auto* box : boxes) {
      if (!box->isVia()) {
        continue;
      }
      odb::dbTechLayer* cut_layer = nullptr;
      std::vector<odb::dbShape> shapes;
      box->getViaBoxes(shapes);
      for (const auto& shape : shapes) {
        if (shape.getTechLayer()->getType() == odb::dbTechLayerType::CUT) {
          if (cut_layer != nullptr && cut_layer != shape.getTechLayer()) {
            cut_layer = nullptr;
            break;
          }
          cut_layer = shape.getTechLayer();
        }
      }
      if (cut_layer == nullptr) {
        continue;
      }
      const auto it
          = transition_map
                .try_emplace(std::make_pair(via, cut_layer),
                             ViaTransition{via, cut_layer, {}, {}, {}})
                .first;
      ViaTransition& transition = it->second;
      transition.boxes.insert(box);
      for (const auto& shape : shapes) {
        auto* layer = shape.getTechLayer();
        if (layer->getType() == odb::dbTechLayerType::CUT) {
          transition.cuts.push_back(shape.getBox());
        } else if (layer->getType() == odb::dbTechLayerType::ROUTING) {
          transition.metals[layer].push_back(shape.getBox());
        }
      }
    }
  }
  for (auto& entry : transition_map) {
    std::ranges::sort(entry.second.cuts);
  }
  return transition_map;
}

}  // namespace

void ViaRepair::repairConflicts(
    const std::map<Via*, odb::PtrSet<odb::dbSBox>>& via_shapes,
    const odb::PtrMap<odb::dbNet, odb::dbSWire*>& net_map)
{
  auto transition_map = collectViaTransitions(via_shapes);

  std::vector<ViaTransition*> transitions;
  for (auto& [key, transition] : transition_map) {
    transitions.push_back(&transition);
  }
  const auto box_id = [](const ViaTransition* transition) {
    return (*transition->boxes.begin())->getId();
  };
  std::ranges::sort(transitions, [&](const auto* lhs, const auto* rhs) {
    return box_id(lhs) < box_id(rhs);
  });
  auto ring_via = [](const ViaTransition* transition) {
    return transition->via->getLowerShape()->getType()
               == odb::dbWireShapeType::RING
           || transition->via->getUpperShape()->getType()
                  == odb::dbWireShapeType::RING;
  };
  auto better = [&](const ViaTransition* lhs, const ViaTransition* rhs) {
    const auto score = [&](const ViaTransition* transition) {
      int64_t area = 0;
      for (const odb::Rect& cut : transition->cuts) {
        area += cut.area();
      }
      return std::make_tuple(transition->cuts.size(),
                             area,
                             ring_via(transition),
                             transition->via->getArea().area());
    };
    const auto lhs_score = score(lhs);
    const auto rhs_score = score(rhs);
    if (lhs_score != rhs_score) {
      return lhs_score > rhs_score;
    }
    if (lhs->via->getArea() != rhs->via->getArea()) {
      return lhs->via->getArea() < rhs->via->getArea();
    }
    return lhs->cuts != rhs->cuts ? lhs->cuts < rhs->cuts
                                  : box_id(lhs) < box_id(rhs);
  };
  auto conflicts_with = [](const ViaTransition* lhs, const ViaTransition* rhs) {
    if (lhs->cut_layer != rhs->cut_layer
        || lhs->via->getNet() != rhs->via->getNet()) {
      return false;
    }
    const int spacing = lhs->cut_layer->getSpacing();
    for (const odb::Rect& lhs_cut : lhs->cuts) {
      odb::Rect obstruction;
      lhs_cut.bloat(std::max(0, spacing - 1), obstruction);
      for (const odb::Rect& rhs_cut : rhs->cuts) {
        if (lhs_cut != rhs_cut && obstruction.intersects(rhs_cut)) {
          return true;
        }
      }
    }
    return false;
  };
  std::map<ViaTransition*, ViaTransition*> replacements;
  ViaConflictIndex conflict_index(transitions);
  for (size_t i = 0; i < transitions.size(); i++) {
    for (const size_t j : conflict_index.candidates(i)) {
      if (conflicts_with(transitions[i], transitions[j])) {
        ViaTransition* winner = better(transitions[i], transitions[j])
                                    ? transitions[i]
                                    : transitions[j];
        ViaTransition* loser
            = winner == transitions[i] ? transitions[j] : transitions[i];
        auto replacement = replacements.find(loser);
        if (replacement == replacements.end()
            || better(winner, replacement->second)) {
          replacements[loser] = winner;
        }
      }
    }
  }

  if (replacements.empty()) {
    return;
  }
  ViaRepairRouting routing(net_map);
  auto resolve = [&](ViaTransition* transition) {
    while (replacements.contains(transition)) {
      transition = replacements.at(transition);
    }
    return transition;
  };
  for (auto* loser : transitions) {
    const auto replacement = replacements.find(loser);
    if (replacement == replacements.end()) {
      continue;
    }
    const ViaTransition* winner = resolve(replacement->second);
    if (!routing.preserves(*loser, *winner)) {
      continue;
    }
    routing.preserveLandings(*loser, net_map.at(loser->via->getNet()));
    for (auto* box : loser->boxes) {
      odb::dbSBox::destroy(box);
    }
  }
}

ViaRepair::ViaRepair(utl::Logger* logger, const odb::PtrSet<odb::dbNet>& nets)
    : logger_(logger), nets_(nets)
{
}

void ViaRepair::repair()
{
  LayerViaTree vias = collectVias();
  removal_count_.clear();

  ObsRect combined_obs;
  if (use_obs_) {
    for (const auto& [layer, obs] :
         collectBlockObstructions((*nets_.begin())->getBlock())) {
      combined_obs[layer].insert(obs.begin(), obs.end());
    }
  }
  if (use_nets_) {
    for (const auto& [layer, obs] :
         collectInstanceObstructions((*nets_.begin())->getBlock())) {
      combined_obs[layer].insert(obs.begin(), obs.end());
    }
  }
  if (use_inst_) {
    for (const auto& [layer, obs] :
         collectNetObstructions((*nets_.begin())->getBlock())) {
      combined_obs[layer].insert(obs.begin(), obs.end());
    }
  }

  // find via violations
  odb::PtrMap<odb::dbTechLayer, odb::PtrSet<odb::dbSBox>> tech_vias_to_remove;
  odb::PtrMap<odb::dbTechLayer, odb::PtrSet<odb::dbSBox>> block_vias_to_remove;
  for (const auto& [layer, layer_obs] : combined_obs) {
    odb::geom::BoostPolygon90Set layer_obstructions;
    for (const odb::Rect& obs : layer_obs) {
      layer_obstructions.insert(odb::geom::toPolygon90(obs));
    }

    const auto& layer_vias = vias[layer];
    auto& tech_vias = tech_vias_to_remove[layer];
    auto& block_vias = block_vias_to_remove[layer];

    for (const odb::Rect& obs_rect :
         odb::geom::extractRectangles(layer_obstructions)) {
      for (auto itr = layer_vias.qbegin(bgi::intersects(obs_rect));
           itr != layer_vias.qend();
           itr++) {
        odb::dbSBox* box = itr->second;
        if (box->getTechVia() != nullptr) {
          tech_vias.insert(box);
        } else {
          block_vias.insert(box);
        }
      }
    }
  }

  // delete offending vias
  for (const auto& [layer, vias] : tech_vias_to_remove) {
    auto& removed = removal_count_[layer];
    for (auto* via : vias) {
      removed++;
      odb::dbSBox::destroy(via);
    }
  }
  for (const auto& [layer, vias] : block_vias_to_remove) {
    if (!vias.empty()) {
      logger_->warn(
          utl::PDN,
          226,
          "{} contains block vias to be removed, which is not supported."),
          layer->getName();
    }
  }
}

ViaRepair::LayerViaTree ViaRepair::collectVias()
{
  LayerViaTree vias;
  via_count_.clear();

  // collect vias
  for (auto* net : nets_) {
    for (auto* swire : net->getSWires()) {
      for (auto* wire : swire->getWires()) {
        if (!wire->isVia()) {
          continue;
        }

        odb::dbTechLayer* cut_layer = nullptr;
        std::vector<odb::dbShape> via_boxes;
        wire->getViaBoxes(via_boxes);
        for (const auto& via_box : via_boxes) {
          auto* layer = via_box.getTechLayer();
          if (layer->getType() == odb::dbTechLayerType::CUT) {
            cut_layer = layer;
            via_count_[layer]++;
          }
        }

        auto* tech_via = wire->getTechVia();
        if (tech_via != nullptr) {
          const odb::Point pt = wire->getViaXY();
          for (const auto& obs : TechViaGenerator::getViaObstructionRects(
                   logger_, tech_via, pt)) {
            vias[cut_layer].insert({obs, wire});
          }
        } else {
          // TODO: implement generate via
        }
      }
    }
  }

  return vias;
}

ViaRepair::ObsRect ViaRepair::collectBlockObstructions(odb::dbBlock* block)
{
  ObsRect obstructions;

  for (auto* obs : block->getObstructions()) {
    auto* box = obs->getBBox();
    if (box->getTechLayer()->getType() != odb::dbTechLayerType::CUT) {
      continue;
    }
    obstructions[box->getTechLayer()].insert(box->getBox());
  }

  return obstructions;
}

ViaRepair::ObsRect ViaRepair::collectNetObstructions(odb::dbBlock* block)
{
  ObsRect obstructions;

  for (auto* net : block->getNets()) {
    auto* wire = net->getWire();
    if (wire == nullptr) {
      continue;
    }

    odb::dbWireShapeItr itr;
    odb::dbShape shape;

    for (itr.begin(wire); itr.next(shape);) {
      if (!shape.isVia()) {
        continue;
      }

      std::vector<odb::dbShape> via_boxes;
      odb::dbShape::getViaBoxes(shape, via_boxes);
      for (const auto& via_box : via_boxes) {
        auto* layer = via_box.getTechLayer();
        if (layer->getType() != odb::dbTechLayerType::CUT) {
          continue;
        }
        obstructions[layer].insert(via_box.getBox());
      }
    }
  }

  return obstructions;
}

ViaRepair::ObsRect ViaRepair::collectInstanceObstructions(odb::dbBlock* block)
{
  ObsRect obstructions;

  for (auto* inst : block->getInsts()) {
    if (!inst->getPlacementStatus().isPlaced()) {
      continue;
    }

    const odb::dbTransform xform = inst->getTransform();

    odb::dbMaster* master = inst->getMaster();
    for (auto* obs : master->getObstructions()) {
      auto* layer = obs->getTechLayer();
      if (layer->getType() != odb::dbTechLayerType::CUT) {
        continue;
      }
      odb::Rect obs_rect = obs->getBox();
      xform.apply(obs_rect);
      obstructions[layer].insert(obs_rect);
    }

    for (const auto& [layer, shapes] : InstanceGrid::getInstancePins(inst)) {
      if (layer->getType() != odb::dbTechLayerType::CUT) {
        continue;
      }
      auto& layer_obs = obstructions[layer];
      for (const auto& shape : shapes) {
        layer_obs.insert(shape->getRect());
      }
    }
  }

  return obstructions;
}

void ViaRepair::report() const
{
  std::string nets;
  for (auto* net : nets_) {
    if (!nets.empty()) {
      nets += ", ";
    }
    nets += net->getName();
  }
  logger_->report("Via repair on {}", nets);
  bool removed_vias = false;
  for (const auto& [layer, removals] : removal_count_) {
    if (removals == 0) {
      continue;
    }
    const int total = via_count_.at(layer);
    double percent = static_cast<double>(removals) / total * 100;
    logger_->report("{} removed {} vias out of {} vias ({:.2f}%).",
                    layer->getName(),
                    removals,
                    total,
                    percent);
    removed_vias = true;
  }

  if (!removed_vias) {
    logger_->report("No vias removed.");
  }
}

}  // namespace pdn
