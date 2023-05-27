#ifndef Costfuntion_H_
#define Costfuntion_H_
#include "Eigen/Eigen"
#include <ros/ros.h>
#include "ControlPoints.h"
namespace air_pilot
{
  class CostFunction // Control point on trajectory
  {
  public:
    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    double bspline_interval_; // B-spline knot span
    std::vector<Eigen::Vector3d> ref_pts_;

    ControlPoints cps_;

    void setParam(ros::NodeHandle &nh);
  private:
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting
    int order_;                    // bspline degree
 
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver

  };
} // namespace air_pilot
#endif