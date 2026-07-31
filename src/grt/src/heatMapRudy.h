// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2025, The OpenROAD Authors

#pragma once

#include "grt/GlobalRouter.h"
#include "grt/Rudy.h"
#include "gui/heatMap.h"
#include "odb/dbBlockCallBackObj.h"
#include "odb/util.h"

namespace odb {
class dbDatabase;
}

namespace grt {

class RUDYDataSource : public gui::GlobalRoutingDataSource,
                       public odb::dbBlockCallBackObj
{
 public:
  RUDYDataSource(utl::Logger* logger,
                 grt::GlobalRouter* grouter,
                 odb::dbDatabase* db);

  void onShow() override;
  void onHide() override;

  // from dbBlockCallBackObj API
  void inDbInstCreate(odb::dbInst*) override;
  void inDbInstDestroy(odb::dbInst*) override;
  void inDbInstPlacementStatusBefore(odb::dbInst*,
                                     const odb::dbPlacementStatus&) override;
  void inDbInstSwapMasterAfter(odb::dbInst*) override;
  void inDbPostMoveInst(odb::dbInst*) override;
  void inDbITermPostDisconnect(odb::dbITerm*, odb::dbNet*) override;
  void inDbITermPostConnect(odb::dbITerm*) override;
  void inDbBTermPostConnect(odb::dbBTerm*) override;
  void inDbBTermPostDisConnect(odb::dbBTerm*, odb::dbNet*) override;

 protected:
  void populateXYGrid() override;
  bool populateMap() override;
  void combineMapData(bool base_has_value,
                      double& base,
                      double new_data,
                      double data_area,
                      double intersection_area,
                      double rect_area) override;

 private:
  grt::GlobalRouter* grouter_;
  odb::dbDatabase* db_;
  grt::Rudy* rudy_;
  bool selection_only_;
  bool show_total_rudy_;
  int rudy_range_;
  float max_net_aspect_ratio_ = -1.0f;
  int aspect_ratio_max_pins_ = std::numeric_limits<int>::max();
  int gaussian_blur_radius_ = 0;
};

gui::HeatMapSourceHandle registerRudyHeatMapSource(utl::Logger* logger,
                                                   grt::GlobalRouter* grouter,
                                                   odb::dbDatabase* db);

}  // namespace grt
