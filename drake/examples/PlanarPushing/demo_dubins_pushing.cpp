#include "dubins_interface.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main() {
  // Limit surface A_11. 
  double ls_a = 1.1; 
  // Limit surface A_33 / rho^2
  double ls_b = 4500;
  // Coefficient of contact friction
  double mu = 0.5;
  Eigen::Vector2d pt(0,-1);
  Eigen::Vector2d normal(0,1);
  DubinsPushPlanner planner(pt, normal, mu, ls_a, ls_b);
  // double min_radius_turn = 0.1;
  // planner.SetRadiusTurn(min_radius_turn);
  // double pose_start[3] = {0,0,0};
  // double pose_goal[3] = {0.05,0,0};
  // std::vector<double> cart_x;
  // std::vector<double> cart_y;
  // std::vector<double> cart_theta;
  // // Plan a path from start to goal. 
  // planner.PlanPath(pose_start, pose_goal, &cart_x, &cart_y, &cart_theta);
  // // Get the control vectors in local frames.
  // std::vector<double> u_x;
  // std::vector<double> u_y;
  // planner.GetLocalFramePushVectors(cart_x, cart_y, cart_theta, &u_x, &u_y);
  // for (uint i = 0; i < cart_x.size(); ++i) {
  //   std::cout << cart_x[i] << "," << cart_y[i] << "," << cart_theta[i] << std::endl;
  // }
  // std::cout << cart_x.size() << std::endl;
  // for (uint i = 0; i < u_x.size(); ++i) {
  //   std::cout << u_x[i] << "," << u_y[i] << std::endl;
  // }
  // std::cout << u_x.size() << std::endl;
}
