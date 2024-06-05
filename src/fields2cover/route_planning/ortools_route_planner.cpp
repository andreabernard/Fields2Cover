//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include "fields2cover/route_planning/ortools_route_planner.h"

namespace f2c::rp {

namespace ortools = operations_research;

std::vector<int64_t> OrtoolsRoutePlanner::computeBestRoute(
    const F2CGraph2D& cov_graph, bool show_log) const {
  int depot_id = static_cast<int>(cov_graph.numNodes()-1);
  const ortools::RoutingIndexManager::NodeIndex depot{depot_id};
  ortools::RoutingIndexManager manager(cov_graph.numNodes(), 1, depot);
  ortools::RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&cov_graph, &manager] (int64_t from, int64_t to) -> int64_t {
        auto from_node = manager.IndexToNode(from).value();
        auto to_node = manager.IndexToNode(to).value();
        return cov_graph.getCostFromEdge(from_node, to_node);
      });
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  ortools::RoutingSearchParameters searchParameters =
    ortools::DefaultRoutingSearchParameters();
  searchParameters.set_use_full_propagation(false);
  searchParameters.set_first_solution_strategy(
    ortools::FirstSolutionStrategy::AUTOMATIC);
  //  searchParameters.set_local_search_metaheuristic(
  //   ortools::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  searchParameters.set_local_search_metaheuristic(
    ortools::LocalSearchMetaheuristic::AUTOMATIC);
  searchParameters.mutable_time_limit()->set_seconds(1);
  searchParameters.set_log_search(show_log);
  const ortools::Assignment* solution =
    routing.SolveWithParameters(searchParameters);

  int64_t index = routing.Start(0);
  std::vector<int64_t> v_id;

  index = solution->Value(routing.NextVar(index));
  while (!routing.IsEnd(index)) {
    v_id.emplace_back(manager.IndexToNode(index).value());
    index = solution->Value(routing.NextVar(index));
  }
  return v_id;
}

}
