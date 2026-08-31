#ifndef PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
#define PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"

namespace plansys2
{

class OPTICPlanSolver : public PlanSolverBase
{
private:
  std::string command_parameter_name_, parameter_name_;
  std::shared_ptr<ros::lifecycle::ManagedNode> lc_node_;

public:
  OPTICPlanSolver();

  void configure(std::shared_ptr<ros::lifecycle::ManagedNode> &, const std::string &);

  std::optional<plansys2_msgs::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  std::string check_domain(
    const std::string & domain,
    const std::string & node_namespace = "");
};

}  // namespace plansys2

#endif  // PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
