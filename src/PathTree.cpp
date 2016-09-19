#include <iostream>
#include <jacobian_path_planner/PathTree.hpp>

const double DELTA = 0.001;  // Delta for Calculating jacobians 

// Constructor
Link::Link(double length, double angle) {
  // Initialize the matrix 4x4
  tf_mat_ = Eigen::MatrixXd::Zero(4, 4);
  tf_mat_(3, 3) = tf_mat_(2, 2) = 1; // (3,3) = 1  for theta 

  // Set the length and the angle of the curve/link
  setParam(length, angle);
}

// Set the length and angle to different values and return the TF
Eigen::MatrixXd Link::getTF(double length, double angle) {
  setParam(length, angle);
  return getTF();
}

// Get the TF with current values of length and angle
Eigen::MatrixXd Link::getTF() {
  // Fill the rotation block in the TF matrix
  tf_mat_.block(0, 0, 2, 2)<< std::cos(angle_), -1*std::sin(angle_), std::sin(angle_), std::cos(angle_);

  // Fille the translation vector in the TF matrix
  if(std::fabs(angle_) < 0.001) tf_mat_.block(0, 3, 3, 1) << length_, angle_/2, angle_;
  else tf_mat_.block(0, 3, 3, 1) << length_*std::sin(angle_)/angle_, 2*length_*pow(std::sin(angle_/2), 2)/angle_, angle_;

  return tf_mat_;
}

// returns the derivative of the endlocation of the link with respect to its state (length and angle). 
Eigen::MatrixXd Link::getDiff() {
  Eigen::MatrixXd diff(3, 2);
  Link delta_link(0, 0);
  delta_link.setParam(length_+DELTA, angle_);
  diff.block(0, 0, 3, 1) = (delta_link.getTF() - getTF()).block(0, 3, 3, 1) / DELTA;
  delta_link.setParam(length_, angle_+DELTA);
  diff.block(0, 1, 3, 1) = (delta_link.getTF() - getTF()).block(0, 3, 3, 1) / DELTA;

  return diff;
}

// Default Constructor
PathTree::PathTree() {
}

// Add curve/link to the Path Chain
void PathTree::add_link(Link link) {
  linkChain_.push_back(link);
}

// Get the TF for end link of the chain with respect to the fist link
Eigen::MatrixXd PathTree::getTF() {
  Eigen::MatrixXd result = Eigen::MatrixXd::Identity(4, 4);
  for (int i = linkChain_.size()-1; i >= 0; --i) {
    result = linkChain_[i].getTF() * result;
  }
  return result;
}

// Calculates the Jacobian for the whole chain where configuration space is a vector of lengths and angles of
// all the links and the task space is (x, y, theta)
Eigen::MatrixXd PathTree::getJacobian() {

  Eigen::MatrixXd jacobian(3, 2 * linkChain_.size());
  Eigen::VectorXd current_state = getTF().block(0, 3, 3, 1);

  for (unsigned int i = 0; i < linkChain_.size(); i++) {
    linkChain_[i].setParamDiff(DELTA, 0);
    jacobian.block(0, 2*i, 3, 1) = (getTF().block(0, 3, 3, 1) - current_state) / DELTA;
    linkChain_[i].setParamDiff(-1 * DELTA, DELTA);
    jacobian.block(0, 2*i+1, 3, 1) = (getTF().block(0, 3, 3, 1) - current_state) / DELTA;
    linkChain_[i].setParamDiff(0, -1 * DELTA);
  }

  return jacobian;
}

// Advances each link in the chain from the input vector of deltas
void PathTree::setLinkDiff(Eigen::VectorXd diff_vec) {
  for (unsigned int i = 0; i < linkChain_.size(); i++) {
    linkChain_[i].setParamDiff(diff_vec[2*i], diff_vec[2*i+1]);
  }
}

// Returns the vector of lengths and angles of all the links in the chain
Eigen::MatrixXd PathTree::printTree() {
  Eigen::MatrixXd result(2, linkChain_.size());
  for (unsigned int i = 0; i < linkChain_.size(); i++) {
    result.block(0, i, 2, 1) << linkChain_[i].getState();
  }
  return result;
}

// Returns (x,y) location of the point at a perticular distace from the start point of calculated path.
Eigen::VectorXd PathTree::getPoint(double length) {
  int link_number = 0;
  for (auto i : linkChain_) {
    auto link_length = i.getState()[0];
    if(std::fabs(link_length) < length) {
      length -= std::fabs(link_length);
      link_number++;
    } else break;
  }
  auto end_link = linkChain_[link_number];
  auto dir = end_link.getState()[0]/std::fabs(end_link.getState()[0]); // Direction of the curve based on sign of the length of that curve.
  end_link.setParam(length*dir, length*dir/ linkChain_[link_number].getRCurve());

  Eigen::MatrixXd result = end_link.getTF();
  for (int i = link_number-1; i >= 0; --i) {
    result = linkChain_[i].getTF() * result;
  }
  return result.block(0, 3, 2, 1);
}

// Returns the total path length of the calculated path plan
double PathTree::getPathLength() {
  double length = 0;
  for (auto& i : linkChain_) {
    length += std::fabs(i.getState()[0]);
  }
  return length;
}
