#include <vector>
#include <Eigen/Dense>

class Link
{
  public:
    // Constructor
    Link(double length, double angle);

    // Getters for TF
    Eigen::MatrixXd getTF(double length, double angle);
    Eigen::MatrixXd getTF();

    // returns the derivative of the endlocation of the link with respect to its state (length and angle). 
    Eigen::MatrixXd getDiff();

    // increaments the length and angle of the link
    inline void setParamDiff(double lengthDiff, double angleDiff) {
      this->length_ += lengthDiff;
      this->angle_ += angleDiff;
    }

    // sets the length and angle of the link
    inline void setParam(double length, double angle) {
      this->length_ = length;
      this->angle_ = angle;
    }

    // Return the radius of curvature of the link
    inline double getRCurve() {
      //if (angle_ < 0.0001) return 1000;
      return length_/angle_;
    }

    // Returns the length and the angle of the link
    inline Eigen::VectorXd getState() {
      Eigen::VectorXd state(2);
      state << length_, angle_;
      return state;
    }

  private:
    // each curve is defined by length and angle of the curve
    double length_, angle_;

    // Transformaation matrix for the link
    Eigen::MatrixXd tf_mat_;
};

class PathTree
{
  public:
    // Constructor
    PathTree();

    // Add curve/link to the Path Chain
    void add_link(Link link);

    // Get the TF for end link of the chain with respect to the fist link
    Eigen::MatrixXd getTF();

    // Calculates the Jacobian for the whole chain where configuration space is a vector of lengths and angles of
    // all the links and the task space is (x, y, theta)
    Eigen::MatrixXd getJacobian();

    // Advances each link in the chain from the input vector of deltas
    void setLinkDiff(Eigen::VectorXd diff_vec);

    // Returns (x,y) location of the point at a perticular distace from the start point of calculated path.
    Eigen::MatrixXd printTree();

    // Returns the total path length of the calculated path plan
    double getPathLength();

    Eigen::VectorXd getPoint(double length);

  private:
    // Chain of all the links`
    std::vector<Link> linkTree;
};
