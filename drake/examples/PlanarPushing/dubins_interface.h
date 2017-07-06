#include "dubins.h"
#include "math.h"
#include <vector>
#include <iostream>

class DubinsPlanner {
 public:
  // Construct the class and set parameters for reduction.
  DubinsPlanner(double ls_a, double ls_b, double r, double mu);
  // Construct the class with additional information on number of way points on the path.
  DubinsPlanner(double ls_a, double ls_b, double r, double mu, int num_horizons);
  ~DubinsPlanner();

  void SetRadiusTurn(double radius);
  // Given a starting pose, end pose in flat space and delta path length, return the planned path in flat space.
  void GetDubinsPathFlatOutput(double flat_pose_start[3], double flat_pose_goal[3], std::vector<double> *flat_path_x, std::vector<double> *flat_path_y);

  // Given a path (length N) in flat space, map it to cartesian space (length N-1).
  void GetCartesianPathGivenFlatOutputPath(const std::vector<double>& flat_path_x, const std::vector<double>& flat_path_y, int num_pts, std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta);

  // Plan a path from cartesian starting pose to cartesian end pose. 
  void PlanPath(double cart_pose_start[3], double cart_pose_end[3], std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta);
  
  void FlatSpaceToCartesianSpace(double z_x, double v_zx, double z_y, double v_zy, double* x, double* y, double* theta);

  // This is for converting the start and end pose to flat space. Assuming instantaneous velocity will follow the positive local y axis. 
  void CartesianSpaceToFlatSpace(double x, double y, double theta, double* z_x, double* z_y, double* z_theta);

  // Get the pushing point vectors from the cartesian path of the object pose. 
  void GetLocalFramePushVectors(const std::vector<double> & cart_x, const std::vector<double> & cart_y, const std::vector<double>& cart_theta, std::vector<double> *push_vec_x, std::vector<double> *push_vec_y);

  // Dubin's path data structure from 3rd party. 
  // DubinsPath path;
  // A11 of the diagonal ellipsoid limit surface.
  double ls_a; 
  // A33 of the diagonal ellipsoid limit surface divided by rho^2.  A33/rho^2. 
  double ls_b;
  // Distance from the pushing point to the local frame center.
  double dist_r;
  // Coefficient of friction.
  double mu;
  // Turning radius
  double radius_turning;
  // step size for planner dubin's path in flat output space.
  double step_size;
  // number of horizons. (if specified, it equals the number of points on the path) 
  int num_horizons;

 
};
