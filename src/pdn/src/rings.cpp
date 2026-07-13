// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#include "rings.h"

#include <algorithm>
#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "boost/polygon/polygon.hpp"
#include "domain.h"
#include "grid.h"
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "shape.h"
#include "techlayer.h"
#include "utl/Logger.h"

namespace pdn {

namespace {

using Polygon90 = boost::polygon::polygon_90_with_holes_data<int>;
using Polygon90Set = boost::polygon::polygon_90_set_data<int>;
using Rectangle = boost::polygon::rectangle_data<int>;

}  // namespace

Rings::Rings(Grid* grid, const Layer& layer0, const Layer& layer1)
    : GridComponent(grid), layer0_(layer0), layer1_(layer1)
{
}

void Rings::checkLayerSpecifications() const
{
  for (const auto& layer : {layer0_, layer1_}) {
    checkLayerWidth(layer.layer, layer.width, layer.layer->getDirection());
    checkLayerSpacing(
        layer.layer, layer.width, layer.spacing, layer.layer->getDirection());
    const TechLayer techlayer(layer.layer);
    techlayer.checkIfManufacturingGrid(layer.width, getLogger(), "Width");
    techlayer.checkIfManufacturingGrid(layer.spacing, getLogger(), "Spacing");
    for (const auto& off : offset_) {
      techlayer.checkIfManufacturingGrid(off, getLogger(), "Core offset");
    }
  }

  checkDieArea();
}

void Rings::checkDieArea() const
{
  int hor_width;
  int ver_width;
  getTotalWidth(hor_width, ver_width);

  odb::Rect ring_outline = getInnerRingOutline();
  ring_outline.set_xlo(ring_outline.xMin() - hor_width);
  ring_outline.set_xhi(ring_outline.xMax() + hor_width);
  ring_outline.set_ylo(ring_outline.yMin() - ver_width);
  ring_outline.set_yhi(ring_outline.yMax() + ver_width);

  const odb::Rect die_area = getBlock()->getDieArea();

  if (!die_area.contains(ring_outline)) {
    if (allow_outside_die_) {
      getLogger()->warn(
          utl::PDN, 239, "Ring shape falls outside the die bounds.");
    } else {
      const double dbus = getBlock()->getDbUnitsPerMicron();

      int xbounds = std::max(die_area.xMin() - ring_outline.xMin(),
                             ring_outline.xMax() - die_area.xMax());
      xbounds = std::max(xbounds, 0);
      int ybounds = std::max(die_area.yMin() - ring_outline.yMin(),
                             ring_outline.yMax() - die_area.yMax());
      ybounds = std::max(ybounds, 0);
      getLogger()->error(
          utl::PDN,
          351,
          "PDN rings do not fit inside the die area by {} um in X and {} um in "
          "Y. Either reduce the ring area or increase the core to die spacing "
          "to accommodate. Use -allow_out_of_die if this is intentional.",
          xbounds / dbus,
          ybounds / dbus);
    }
  }
}

void Rings::setOffset(const std::array<int, 4>& offset)
{
  offset_ = offset;
}

void Rings::setPadOffset(const std::array<int, 4>& offset)
{
  odb::Rect die_area = getBlock()->getDieArea();
  odb::Rect core = getBlock()->getCoreArea();

  odb::Rect pads_inner = die_area;

  // look for placed pads
  for (auto* inst : getBlock()->getInsts()) {
    if (!inst->getPlacementStatus().isPlaced()) {
      continue;
    }

    auto type = inst->getMaster()->getType();
    // only looking for pads
    if (!type.isPad()) {
      continue;
    }

    if (type == odb::dbMasterType::PAD_AREAIO) {
      continue;
    }

    odb::Rect box = inst->getBBox()->getBox();

    const bool is_ns_with_core
        = box.xMin() >= core.xMin() && box.xMax() <= core.xMax();
    const bool is_ew_with_core
        = box.yMin() >= core.yMin() && box.yMax() <= core.yMax();
    const bool is_north = box.yMin() > core.yMax() && is_ns_with_core;
    const bool is_south = box.yMax() < core.yMin() && is_ns_with_core;
    const bool is_west = box.xMax() < core.xMin() && is_ew_with_core;
    const bool is_east = box.xMin() > core.xMax() && is_ew_with_core;

    // find the inner edge of the pad outline
    if (is_north) {
      pads_inner.set_yhi(std::min(pads_inner.yMax(), box.yMin()));
    } else if (is_south) {
      pads_inner.set_ylo(std::max(pads_inner.yMin(), box.yMax()));
    } else if (is_west) {
      pads_inner.set_xlo(std::max(pads_inner.xMin(), box.xMax()));
    } else if (is_east) {
      pads_inner.set_xhi(std::min(pads_inner.xMax(), box.xMin()));
    }
  }

  if (core == pads_inner) {
    getLogger()->warn(utl::PDN,
                      105,
                      "Unable to determine location of pad offset, using die "
                      "boundary instead.");
    pads_inner = getBlock()->getDieArea();
  }

  int hor_width;
  int ver_width;
  getTotalWidth(hor_width, ver_width);

  debugPrint(getLogger(),
             utl::PDN,
             "PadOffset",
             1,
             "Core area: {}",
             Shape::getRectText(core, getBlock()->getDbUnitsPerMicron()));
  debugPrint(getLogger(),
             utl::PDN,
             "PadOffset",
             1,
             "Pads inner: {}",
             Shape::getRectText(pads_inner, getBlock()->getDbUnitsPerMicron()));

  std::array<int, 4> core_offset{
      core.xMin() - pads_inner.xMin() - offset[0] - ver_width,
      core.yMin() - pads_inner.yMin() - offset[1] - hor_width,
      pads_inner.xMax() - core.xMax() - offset[2] - ver_width,
      pads_inner.yMax() - core.yMax() - offset[3] - hor_width};

  // apply pad offset as core offset
  setOffset(core_offset);
}

void Rings::getTotalWidth(int& hor, int& ver) const
{
  const int rings = getNetCount();
  hor = layer0_.width * rings + layer0_.spacing * (rings - 1);
  ver = layer1_.width * rings + layer1_.spacing * (rings - 1);
  if (layer0_.layer->getDirection() != odb::dbTechLayerDir::HORIZONTAL) {
    std::swap(hor, ver);
  }
}

void Rings::setExtendToBoundary(bool value)
{
  extend_to_boundary_ = value;
}

odb::Rect Rings::getInnerRingOutline() const
{
  odb::Rect core;
  core.mergeInit();
  for (const auto& rect : getInnerRingOutlines()) {
    core.merge(rect);
  }

  return core;
}

std::vector<odb::Rect> Rings::getInnerRingOutlines() const
{
  std::vector<odb::Rect> cores = getGrid()->getDomainAreaRects();
  for (auto& core : cores) {
    core.set_xlo(core.xMin() - offset_[0]);
    core.set_ylo(core.yMin() - offset_[1]);
    core.set_xhi(core.xMax() + offset_[2]);
    core.set_yhi(core.yMax() + offset_[3]);
  }
  return cores;
}

void Rings::makeShapes(const Shape::ShapeTreeMap& other_shapes)
{
  debugPrint(getLogger(),
             utl::PDN,
             "Make",
             1,
             "Ring start of make shapes on layers {} and {}",
             layer0_.layer->getName(),
             layer1_.layer->getName());
  clearShapes();

  auto* grid = getGrid();

  const auto nets = getNets();

  const auto cores = getInnerRingOutlines();

  const bool single_layer_ring = layer0_.layer == layer1_.layer;
  auto lock_shapes = [&]() {
    if (!single_layer_ring) {
      return;
    }
    for (const auto& [layer, shapes] : getShapes()) {
      for (const auto& shape : shapes) {
        shape->setLocked();
      }
    }
  };

  using LayerPair = std::pair<Layer*, Layer*>;
  const std::array<LayerPair, 2> build_layers{LayerPair{&layer0_, &layer1_},
                                              LayerPair{&layer1_, &layer0_}};

  if (!extend_to_boundary_) {
    using boost::polygon::operators::operator+=;

    Polygon90Set core_set;
    for (const auto& core_rect : cores) {
      core_set += Rectangle(core_rect.xMin(),
                            core_rect.yMin(),
                            core_rect.xMax(),
                            core_rect.yMax());
    }
    int hor_width;
    int ver_width;
    getTotalWidth(hor_width, ver_width);
    const int space = std::max(layer0_.spacing, layer1_.spacing);
    const int x_gate = ver_width + space;
    const int y_gate = hor_width + space;
    core_set.bloat(x_gate, x_gate, y_gate, y_gate);
    core_set.shrink(x_gate, x_gate, y_gate, y_gate);

    std::vector<Polygon90> polygons;
    core_set.get_polygons(polygons);

    auto add_edge_shapes = [&](const auto& polygon,
                               Layer* layer_def,
                               Layer* layer_other,
                               bool make_horizontal) {
      const int width = layer_def->width;
      const int pitch = layer_def->spacing + width;
      const int other_width = layer_other->width;
      const int other_pitch = layer_other->spacing + other_width;
      const bool ccw = boost::polygon::winding(polygon)
                       == boost::polygon::COUNTERCLOCKWISE;

      std::vector<Polygon90::point_type> points(polygon.begin(), polygon.end());
      for (int edge = 0; edge < points.size(); edge++) {
        const auto& pt0 = points[edge];
        const auto& pt1 = points[(edge + 1) % points.size()];
        const bool horizontal = pt0.y() == pt1.y();
        if (horizontal != make_horizontal) {
          continue;
        }

        for (int idx = 0; idx < nets.size(); idx++) {
          const int offset = idx * pitch;
          const int end_offset = other_width + idx * other_pitch;
          odb::Rect rect;
          if (horizontal) {
            const int dir = pt1.x() > pt0.x() ? 1 : -1;
            const int outward = ccw ? -dir : dir;
            const int y0 = pt0.y();
            const int y1 = outward < 0 ? y0 - offset - width : y0 + offset;
            const int y2 = outward < 0 ? y0 - offset : y0 + offset + width;
            rect = odb::Rect(std::min(pt0.x(), pt1.x()) - end_offset,
                             y1,
                             std::max(pt0.x(), pt1.x()) + end_offset,
                             y2);
          } else {
            const int dir = pt1.y() > pt0.y() ? 1 : -1;
            const int outward = ccw ? dir : -dir;
            const int x0 = pt0.x();
            const int x1 = outward < 0 ? x0 - offset - width : x0 + offset;
            const int x2 = outward < 0 ? x0 - offset : x0 + offset + width;
            rect = odb::Rect(x1,
                             std::min(pt0.y(), pt1.y()) - end_offset,
                             x2,
                             std::max(pt0.y(), pt1.y()) + end_offset);
          }
          addShape(std::make_unique<Shape>(
              layer_def->layer, nets[idx], rect, odb::dbWireShapeType::RING));
        }
      }
    };

    bool processed_horizontal = false;
    for (const auto& [layer_def, layer_other] : build_layers) {
      const bool make_horizontal
          = (single_layer_ring && !processed_horizontal)
            || (!single_layer_ring
                && layer_def->layer->getDirection()
                       == odb::dbTechLayerDir::HORIZONTAL);
      processed_horizontal |= make_horizontal;
      for (const auto& polygon : polygons) {
        add_edge_shapes(polygon, layer_def, layer_other, make_horizontal);
        for (auto itr = polygon.begin_holes(); itr != polygon.end_holes();
             itr++) {
          add_edge_shapes(*itr, layer_def, layer_other, make_horizontal);
        }
      }
    }

    lock_shapes();
    return;
  }

  const odb::Rect boundary = grid->getGridBoundary();
  const odb::Rect core = getInnerRingOutline();

  bool processed_horizontal = false;
  for (auto* layer_def : {&layer0_, &layer1_}) {
    auto* layer = layer_def->layer;
    const int width = layer_def->width;
    const int pitch = layer_def->spacing + width;

    if ((single_layer_ring && !processed_horizontal)
        || (!single_layer_ring
            && layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL)) {
      processed_horizontal = true;

      // bottom
      const int x_start = boundary.xMin();
      const int x_end = boundary.xMax();
      int y_start = core.yMin() - width;
      int y_end = core.yMin();
      for (auto net : nets) {
        addShape(
            std::make_unique<Shape>(layer,
                                    net,
                                    odb::Rect(x_start, y_start, x_end, y_end),
                                    odb::dbWireShapeType::RING));
        y_start -= pitch;
        y_end -= pitch;
      }
      // top
      y_start = core.yMax();
      y_end = y_start + width;
      for (auto net : nets) {
        addShape(
            std::make_unique<Shape>(layer,
                                    net,
                                    odb::Rect(x_start, y_start, x_end, y_end),
                                    odb::dbWireShapeType::RING));
        y_start += pitch;
        y_end += pitch;
      }
    } else {
      // left
      int x_start = core.xMin() - width;
      int x_end = core.xMin();
      const int y_start = boundary.yMin();
      const int y_end = boundary.yMax();
      for (auto net : nets) {
        addShape(
            std::make_unique<Shape>(layer,
                                    net,
                                    odb::Rect(x_start, y_start, x_end, y_end),
                                    odb::dbWireShapeType::RING));
        x_start -= pitch;
        x_end -= pitch;
      }
      // right
      x_start = core.xMax();
      x_end = x_start + width;
      for (auto net : nets) {
        addShape(
            std::make_unique<Shape>(layer,
                                    net,
                                    odb::Rect(x_start, y_start, x_end, y_end),
                                    odb::dbWireShapeType::RING));
        x_start += pitch;
        x_end += pitch;
      }
    }
  }

  lock_shapes();
}

std::vector<odb::dbTechLayer*> Rings::getLayers() const
{
  std::vector<odb::dbTechLayer*> layers;
  layers.reserve(2);
  for (const auto& layer_def : {layer0_, layer1_}) {
    layers.push_back(layer_def.layer);
  }
  return layers;
}

void Rings::report() const
{
  auto* logger = getLogger();

  const double dbu_per_micron = getBlock()->getDbUnitsPerMicron();

  logger->report("  Core offset:");
  logger->report("    Left: {:.4f}", offset_[0] / dbu_per_micron);
  logger->report("    Bottom: {:.4f}", offset_[1] / dbu_per_micron);
  logger->report("    Right: {:.4f}", offset_[2] / dbu_per_micron);
  logger->report("    Top: {:.4f}", offset_[3] / dbu_per_micron);

  for (const auto& layer : {layer0_, layer1_}) {
    logger->report("  Layer: {}", layer.layer->getName());
    logger->report("    Width: {:.4f}", layer.width / dbu_per_micron);
    logger->report("    Spacing: {:.4f}", layer.spacing / dbu_per_micron);
  }
}

}  // namespace pdn
