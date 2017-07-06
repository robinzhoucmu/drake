#include "dubins_interface.h"

int num_steps = 1;
int ind_data = 0;
int StoreData(double q[3], double x, void* user_data) {
  
  if (x == 0)
    ind_data = 0; // Reset index counter when distance is equal to 0

  ((double*)user_data)[4 * ind_data + 0] = q[0]; // Write to the output matrix
  ((double*)user_data)[4 * ind_data + 1] = q[1];
  ((double*)user_data)[4 * ind_data + 2] = q[2];
  ((double*)user_data)[4 * ind_data + 3] = x; // Save distance along the path

  if (++ind_data >= num_steps) // Prevent buffer overflow
    return 1; // Stop sampling
  return 0;
}

DubinsPlanner::DubinsPlanner(double a_, double b_, double r_, double mu_) {
  ls_a = a_;
  ls_b = b_;
  dist_r = r_;
  mu = mu_;
  radius_turning = ls_a / (ls_b * dist_r * mu);
  num_horizons = -1;
  // If num_horizons is not specified, then we fix step size.
  step_size = radius_turning / 100;
}

DubinsPlanner::DubinsPlanner(double a_, double b_, double r_, double mu_, int num_horizons_) {
  ls_a = a_;
  ls_b = b_;
  dist_r = r_;
  mu = mu_;
  radius_turning = ls_a / (ls_b * dist_r * mu);
  num_horizons = num_horizons_;
  // Norminal value since it will later be determined from the total length and fixed number of steps/horizons.
  step_size = radius_turning / 100;
}

DubinsPlanner::~DubinsPlanner() {}

void DubinsPlanner::SetRadiusTurn(double radius_) {
  radius_turning = radius_;
}

void DubinsPlanner::GetDubinsPathFlatOutput(double flat_pose_start[3], double flat_pose_goal[3],  std::vector<double> *flat_path_x, std::vector<double> *flat_path_y) {
  DubinsPath path;
  dubins_init(flat_pose_start, flat_pose_goal, radius_turning, &path);
  
  double length = dubins_path_length(&path);
  if (num_horizons == -1) {
    num_steps = (int)floor(length/step_size);
    if (num_steps < 1) {
      num_steps = 1;
    }
  } else {
    num_steps = num_horizons;
    // Also change step size accordingly here.
    step_size = length / num_steps;
  }
  // Create output array.
  double* output_z = new double[4*num_steps];
  
  dubins_path_sample_many(&path, StoreData, step_size, output_z);
  
  // Extract from output_z. 
  for (int i = 0; i<num_steps; ++i) {
    flat_path_x->push_back(*(output_z + 4 * i));
    flat_path_y->push_back(*(output_z + 4 * i + 1));
  }
  
  delete [] output_z;
}

// Given a path in flat space, map it to cartesian space.
void DubinsPlanner::GetCartesianPathGivenFlatOutputPath(const std::vector<double>& flat_path_x, const std::vector<double>& flat_path_y, int num_pts, std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta) { 
  double dt = 1.0 / num_pts;
  // Starting from the second point, use difference w.r.t. previous point.
  for (int i = 1; i < num_pts; ++i) {
    double v_zx = (flat_path_x[i] - flat_path_x[i-1]) / dt;
    double v_zy = (flat_path_y[i] - flat_path_y[i-1]) / dt;
    double x,y,theta;
    FlatSpaceToCartesianSpace(flat_path_x[i], v_zx, flat_path_y[i], v_zy, &x, &y, &theta);    
    cart_x->push_back(x);
    cart_y->push_back(y);
    cart_theta->push_back(theta);
  }
}

// Plan a path from cartesian starting pose to cartesian end pose. 
void DubinsPlanner::PlanPath(double cart_pose_start[3], double cart_pose_goal[3], std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta) {
  double flat_pose_start[3];
  double flat_pose_goal[3];
  // Convert start and goal to flat space.
  CartesianSpaceToFlatSpace(cart_pose_start[0], cart_pose_start[1], cart_pose_start[2], &flat_pose_start[0], &flat_pose_start[1], &flat_pose_start[2]);
  CartesianSpaceToFlatSpace(cart_pose_goal[0], cart_pose_goal[1], cart_pose_goal[2], &flat_pose_goal[0], &flat_pose_goal[1], &flat_pose_goal[2]);
  
  // Get the planning result in flat space.
  std::vector<double> flat_path_x;
  std::vector<double> flat_path_y;
  GetDubinsPathFlatOutput(flat_pose_start, flat_pose_goal, &flat_path_x, &flat_path_y);
  
  int num_pts = flat_path_x.size();
  // Push in the start pose at the first element. 
  cart_x->push_back(cart_pose_start[0]);
  cart_y->push_back(cart_pose_start[1]);
  cart_theta->push_back(cart_pose_start[2]);
  // Convert the flat space output to cartesian space.
  GetCartesianPathGivenFlatOutputPath(flat_path_x, flat_path_y, num_pts, cart_x, cart_y, cart_theta);
}

// Get the pushing point vectors from the cartesian path of the object pose. 
void DubinsPlanner::GetLocalFramePushVectors(const std::vector<double> & cart_x, const std::vector<double> & cart_y, const std::vector<double>& cart_theta, std::vector<double> *push_vec_x, std::vector<double> *push_vec_y) {
  for (uint i = 0; i < cart_x.size() - 1; ++i) {
    double theta = cart_theta[i];
    // Global frame.
    double pt_x = -sin(cart_theta[i]) * (-dist_r) + cart_x[i];
    double pt_y = cos(cart_theta[i]) * (-dist_r) + cart_y[i];
    double pt_x_nxt = -sin(cart_theta[i+1]) * (-dist_r) + cart_x[i+1];
    double pt_y_nxt = cos(cart_theta[i+1]) * (-dist_r) + cart_y[i+1];
    double delta_pt_x = pt_x_nxt - pt_x;
    double delta_pt_y = pt_y_nxt - pt_y;
    // Convert to local frame.
    double u_x = cos(theta) * delta_pt_x + sin(theta) * delta_pt_y;
    double u_y = -sin(theta) * delta_pt_x + cos(theta) * delta_pt_y;
    push_vec_x->push_back(u_x);
    push_vec_y->push_back(u_y);
  }
}



void DubinsPlanner::FlatSpaceToCartesianSpace(double z_x, double v_zx, double z_y, double v_zy, double* x, double* y, double* theta) {
  *theta = atan2(-v_zx, v_zy);
  *x = z_x + ls_a / (ls_b * dist_r) * sin(*theta);
  *y = z_y - ls_a / (ls_b * dist_r) * cos(*theta);
}


void DubinsPlanner::CartesianSpaceToFlatSpace(double x, double y, double theta, double* z_x, double* z_y, double* z_theta) {
  *z_x = x + ls_a / (ls_b * dist_r) * (-sin(theta));
  *z_y = y + ls_a / (ls_b * dist_r) * (cos(theta));
  *z_theta = fmod(theta + M_PI / 2.0 + 2 * M_PI, 2 * M_PI);
}



