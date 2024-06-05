//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#pragma once
#ifndef FIELDS2COVER_ROUTE_PLANNING_ORTOOLS_ROUTE_PLANNER_H_
#define FIELDS2COVER_ROUTE_PLANNING_ORTOOLS_ROUTE_PLANNER_H_

#include "fields2cover/route_planning/route_planner_base.h"


namespace f2c::rp {

class OrtoolsRoutePlanner : public RoutePlannerBase  {
 public:
   using RoutePlannerBase::RoutePlannerBase;

   OrtoolsRoutePlanner() = default;
    ~OrtoolsRoutePlanner() override = default;
 protected:
  /// Use the optimizer to generate the index of the points of the best
  ///   coverage route.
  std::vector<int64_t> computeBestRoute(
      const F2CGraph2D& cov_graph, bool show_log) const override;
};



}  // namespace f2c::rp

#endif  // FIELDS2COVER_ROUTE_PLANNING_ORTOOLS_ROUTE_PLANNER_H_

