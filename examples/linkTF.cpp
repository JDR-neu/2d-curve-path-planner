#include <iostream>
#include <jacobian_path_planner/PathTree.hpp>

int main(int argc, char *argv[]) {
  PathTree p;
  p.add_link(Link(1, 0));
  p.add_link(Link(1, 0));
  p.add_link(Link(1, 0));

  std::cout << p.getTF() << std::endl;

  auto jac = p.getJacobian();
  std::cout << "Jacobian:" << std::endl << jac << std::endl;
  std::cout << "Jacobian inverse:" << std::endl << jac.transpose()*(jac * jac.transpose()).inverse() << std::endl;
  return 0;
}
