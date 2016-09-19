#include <iostream>
#include <jacobian_path_planner/PathTree.hpp>

int main(int argc, char *argv[]) {
  PathTree p;

  // Add 5 links to the chain
  for (unsigned int i = 0; i < 5; i++) {
    p.add_link(Link(0.1, 0));
  }

  // Set the goal for the problem.
  Eigen::VectorXd goal(3), error(3);
  goal << -5, 0, 1.57;

  // Iterate to reduce the error
  do {
    error = goal - p.getTF().block(0, 3, 3, 1);
    auto jac = p.getJacobian();
    auto diff = jac.transpose()*(jac * jac.transpose()).inverse() * error * 0.1;
    p.setLinkDiff(diff);
  } while (error.norm() > 0.001);

  // Print the calculated path
  auto total_length = p.getPathLength();
  for (unsigned int i = 0; i < 200; i++) {
    auto pt = p.getPoint(i* total_length / 200);
    std::cout << pt[0] << " " << pt[1] << std::endl;
  }

  //std::cout << p.getTF() << std::endl;
  //std::cout << p.printTree() << std::endl;

  return 0;
}
