#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
struct DataModel {
  std::vector<std::vector<double>> distance_matrix;
  int num_vehicles = 1;
  RoutingIndexManager::NodeIndex depot{0};
};




std::vector<std::vector<double>> PrintSolution(const RoutingIndexManager& manager,
              const RoutingModel& routing, const Assignment& solution, const std::vector<std::vector<double>>& points) {
  double index = routing.Start(0);
  std::stringstream route;

  std::vector<std::vector<double>> waypoints;

  while (!routing.IsEnd(index)) {
    int i =  manager.IndexToNode(index).value();
    waypoints.emplace_back(points[i]);
    index = solution.Value(routing.NextVar(index));
  }
  int i =  manager.IndexToNode(index).value();
  waypoints.emplace_back(points[i]);
  return waypoints;
}



double distance(const std::vector<double>& pt1,const std::vector<double> pt2){
  double square_sum = 0;
  for(int i = 0;i<3;i++) square_sum += (pt1[i]-pt2[i])*(pt1[i]-pt2[i]);
  return square_sum;
}




DataModel process_points(const std::vector<std::vector<double>> &points){
  
  DataModel data;
  
  int n_points = points.size(); 
  
  data.distance_matrix = std::vector<std::vector<double>>(n_points, std::vector<double>(n_points,0));
  
  for(int i = 0 ; i<n_points;i++)
    for(int j = 0; j<n_points;j++)
      data.distance_matrix[i][j] = distance(points[i], points[j]);
    
  return data;
}




std::vector<std::vector<double>> Tsp(std::vector<std::vector<double>> &points) {
  DataModel data = process_points(points);

  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const double from_index,
                        const double to_index) -> double {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  return  PrintSolution(manager, routing, *solution, points);
}

}  

int main(int /*argc*/, char* /*argv*/[]) {
/*
 std::vector<std::vector<double>> store = {
     {1,1,1},
     {2,2,2},
     {25,25,25},
     {1,2,1},
 };
 */

  std::vector<std::vector<double>> store;
  double x, y, z;
  std::string line;
  std::ifstream rfile;
  rfile.open("/waypoints.csv");
  if (rfile.is_open()) {
      while (std::getline(rfile, line)) {
          rfile >> x >> y >> z;
          store.push_back({x, y, z});
      }
      rfile.close();
  }

  /*
  for(int i = 0; i < store.size(); i++){
    for(int j = 0; j < store[0].size(); j++){
      std::cout << store[i][j] << " ";
    }
    std::cout << std::endl;
  }
  */

  std::vector<std::vector<double>> waypoints  = operations_research::Tsp(store);

  std::ofstream wfile;
  wfile.open("/waypoints_tsp.csv");
  for(auto& wp : waypoints){
    if(wfile.is_open()){
      wfile << wp[0] << " " << wp[1] << " " << wp[2] << "\n";
    }
    // std::cout<<wp[0]<<" "<<wp[1]<<" "<<wp[2]<<std::endl;
  }
  wfile.close();

  return EXIT_SUCCESS;
}
