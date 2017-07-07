#include "dubins_interface.h"
#include <limits>
int num_segments = 1;
int ind_data = 0;
int StoreData(double q[3], double x, void* user_data) {
  
  if (x == 0)
    ind_data = 0; // Reset index counter when distance is equal to 0

  ((double*)user_data)[4 * ind_data + 0] = q[0]; // Write to the output matrix
  ((double*)user_data)[4 * ind_data + 1] = q[1];
  ((double*)user_data)[4 * ind_data + 2] = q[2];
  ((double*)user_data)[4 * ind_data + 3] = x; // Save distance along the path

  if (++ind_data >= num_segments) // Prevent buffer overflow
    return 1; // Stop sampling
  return 0;
}

DubinsPushPlanner::DubinsPushPlanner(Eigen::Vector2d contact_point, 
  Eigen::Vector2d contact_normal, double mu, double ls_a, double ls_b) {
  
  contact_point_ = contact_point;
  // In case the vector input is not normalized.
  contact_normal_ = contact_normal.normalized();
  
  ls_a_ = ls_a;
  ls_b_ = ls_b;
  unit_length_ = ls_a_ / ls_b_;
  mu_ = mu;
  
  // Compute all relevant parameters through the reduction logic.
  DubinsReduction();
}

DubinsPushPlanner::~DubinsPushPlanner() {}

void DubinsPushPlanner::DubinsPushPlanner::DubinsReduction() {
  ComputeFrictionCone();
  ComputeCenterOfRearAxleAndTurningRadius();
  ComputeDubinsFrame();
}

void DubinsPushPlanner::ComputeCenterOfRearAxleAndTurningRadius() {
  // Compute distances of the line of forces to the COM. 
  double dist_fl_to_com = abs(contact_point_(2) * friction_cone_left_(1) - 
    contact_point_(1) * friction_cone_left_(2));
  double dist_fr_to_com = abs(contact_point_(2) * friction_cone_right_(1) - 
    contact_point_(1) * friction_cone_right_(2));
  
  // The distance of the line of CORs to COM is inverse proportion to the 
  // distance from the contact point to COM. 
  double dist_line_to_com = unit_length_ / (contact_point_.norm() + 
    std::numeric_limits<double>::epsilon());

  // Compute the horizontal offset of the two CORs (dual points of the edges 
  // of the friction cone) along the line of CORs. 
  double cor_fl_dx = sqrt((unit_length_ / dist_fl_to_com) * 
    (unit_length_ / dist_fl_to_com) - dist_line_to_com * dist_line_to_com);
  
  double cor_fr_dx = -1.0 * sqrt((unit_length_ / dist_fr_to_com) * 
    (unit_length_ / dist_fr_to_com) - dist_line_to_com * dist_line_to_com);

  center_rear_axle_(0) = (cor_fl_dx + cor_fr_dx) / 2.0;
  center_rear_axle_(1) = dist_line_to_com;

  r_turn_ = (cor_fl_dx - cor_fr_dx) / 2.0;

}

void DubinsPushPlanner::ComputeDubinsFrame() {
  // The dubins push frame (rc_x, rc_y, theta): origin -> center of rear axle
  // +y axis -> the vector pointing from the contact point to COM.
  double angle_frame = atan2(contact_point_(1), -contact_point_(2));
  Eigen::Rotation2D<double> rotmat(angle_frame); 
  tf_pusher_frame_.linear() = rotmat.toRotationMatrix();
  tf_pusher_frame_.translation() = rotmat * center_rear_axle_;
}

void DubinsPushPlanner::ComputeFrictionCone() {
  double fc_half_angle = atan2(mu_, 1);
  Eigen::Rotation2D<double> rotmat_l(fc_half_angle);
  Eigen::Rotation2D<double> rotmat_r(-fc_half_angle);
  friction_cone_left_ = rotmat_l * contact_normal_;
  friction_cone_right_ = rotmat_r * contact_normal_;
}


void DubinsPushPlanner::GetDubinsPathFlatOutput(const Eigen::Vector3d 
  flat_augmented_z_start, const Eigen::Vector3d flat_augmented_z_goal, 
  const int num_way_points, Eigen::Matrix<double, Eigen::Dynamic, 2>* flat_traj_z) {
  
  DubinsPath path;
  // Call third party dubins curve generator. Return a path structure.
  dubins_init(flat_augmented_z_start.data(), flat_augmented_z_goal.data(), 
    r_turn_, &path);
  // Get path length to determine the sampling segment length.
  double path_length = dubins_path_length(&path);

  int num_segments = num_way_points - 1;
  double step_size = path_length / num_segments;
  
  // Create output array.
  double* output_z = new double[4*num_segments];
  
  dubins_path_sample_many(&path, StoreData, step_size, output_z);
  
  // Extract from output_z. 
  flat_traj_z->resize(num_segments + 1, 2);
  for (int i = 0; i<num_segments; ++i) {
    (*flat_traj_z)(i, 0) = *(output_z + 4 * i);
    (*flat_traj_z)(i, 1) = *(output_z + 4 * i + 1);
   }
  delete [] output_z;

  // Append flat_z_goal. 
  (*flat_traj_z)(num_segments, 0) = flat_augmented_z_goal(0); 
  (*flat_traj_z)(num_segments, 1) = flat_augmented_z_goal(1); 
}

// // Given a path in flat space, map it to cartesian space.
// void DubinsPushPlanner::GetCartesianPathGivenFlatOutputPath(const std::vector<double>& flat_path_x, const std::vector<double>& flat_path_y, int num_pts, std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta) { 
//   double dt = 1.0 / num_pts;
//   // Starting from the second point, use difference w.r.t. previous point.
//   for (int i = 1; i < num_pts; ++i) {
//     double v_zx = (flat_path_x[i] - flat_path_x[i-1]) / dt;
//     double v_zy = (flat_path_y[i] - flat_path_y[i-1]) / dt;
//     double x,y,theta;
//     FlatSpaceToCartesianSpace(flat_path_x[i], v_zx, flat_path_y[i], v_zy, &x, &y, &theta);    
//     cart_x->push_back(x);
//     cart_y->push_back(y);
//     cart_theta->push_back(theta);
//   }
// }

// // Plan a path from cartesian starting pose to cartesian end pose. 
// void DubinsPushPlanner::PlanPath(double cart_pose_start[3], double cart_pose_goal[3], std::vector<double> *cart_x, std::vector<double> *cart_y, std::vector<double>* cart_theta) {
//   double flat_pose_start[3];
//   double flat_pose_goal[3];
//   // Convert start and goal to flat space.
//   CartesianSpaceToFlatSpace(cart_pose_start[0], cart_pose_start[1], cart_pose_start[2], &flat_pose_start[0], &flat_pose_start[1], &flat_pose_start[2]);
//   CartesianSpaceToFlatSpace(cart_pose_goal[0], cart_pose_goal[1], cart_pose_goal[2], &flat_pose_goal[0], &flat_pose_goal[1], &flat_pose_goal[2]);
  
//   // Get the planning result in flat space.
//   std::vector<double> flat_path_x;
//   std::vector<double> flat_path_y;
//   GetDubinsPathFlatOutput(flat_pose_start, flat_pose_goal, &flat_path_x, &flat_path_y);
  
//   int num_pts = flat_path_x.size();
//   // Push in the start pose at the first element. 
//   cart_x->push_back(cart_pose_start[0]);
//   cart_y->push_back(cart_pose_start[1]);
//   cart_theta->push_back(cart_pose_start[2]);
//   // Convert the flat space output to cartesian space.
//   GetCartesianPathGivenFlatOutputPath(flat_path_x, flat_path_y, num_pts, cart_x, cart_y, cart_theta);
// }

// // Get the pushing point vectors from the cartesian path of the object pose. 
// void DubinsPushPlanner::GetLocalFramePushVectors(const std::vector<double> & cart_x, const std::vector<double> & cart_y, const std::vector<double>& cart_theta, std::vector<double> *push_vec_x, std::vector<double> *push_vec_y) {
//   for (uint i = 0; i < cart_x.size() - 1; ++i) {
//     double theta = cart_theta[i];
//     // Global frame.
//     double pt_x = -sin(cart_theta[i]) * (-dist_r) + cart_x[i];
//     double pt_y = cos(cart_theta[i]) * (-dist_r) + cart_y[i];
//     double pt_x_nxt = -sin(cart_theta[i+1]) * (-dist_r) + cart_x[i+1];
//     double pt_y_nxt = cos(cart_theta[i+1]) * (-dist_r) + cart_y[i+1];
//     double delta_pt_x = pt_x_nxt - pt_x;
//     double delta_pt_y = pt_y_nxt - pt_y;
//     // Convert to local frame.
//     double u_x = cos(theta) * delta_pt_x + sin(theta) * delta_pt_y;
//     double u_y = -sin(theta) * delta_pt_x + cos(theta) * delta_pt_y;
//     push_vec_x->push_back(u_x);
//     push_vec_y->push_back(u_y);
//   }
// }



// void DubinsPushPlanner::FlatSpaceToCartesianSpace(double z_x, double v_zx, double z_y, double v_zy, double* x, double* y, double* theta) {
//   *theta = atan2(-v_zx, v_zy);
//   *x = z_x + ls_a / (ls_b * dist_r) * sin(*theta);
//   *y = z_y - ls_a / (ls_b * dist_r) * cos(*theta);
// }


// void DubinsPushPlanner::CartesianSpaceToFlatSpace(double x, double y, double theta, double* z_x, double* z_y, double* z_theta) {
//   *z_x = x + ls_a / (ls_b * dist_r) * (-sin(theta));
//   *z_y = y + ls_a / (ls_b * dist_r) * (cos(theta));
//   *z_theta = fmod(theta + M_PI / 2.0 + 2 * M_PI, 2 * M_PI);
// }



