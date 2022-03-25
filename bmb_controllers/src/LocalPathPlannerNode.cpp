#include "bmb_controllers/LocalPathPlannerNode.h"
#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PIDFFController.h>
#include <bmb_controllers/PosVelState.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_math/Vector3.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/ControllerGains.h>
#include <bmb_utilities/MathUtils.h>
#include <bmb_world_model/AppliedLoads.h>
#include <bmb_world_model/Constants.h>
#include <ros/ros.h>
#include <cmath>

static constexpr ControllerGains ALTITUDE_GAIN{0.05, 0.005, 0.04};

LocalPathPlannerNode::LocalPathPlannerNode(ros::NodeHandle& nh,
                                           const double& update_frequency)
    : update_frequency(update_frequency),
      altitude_pid(
          PIDFFController<double>{ALTITUDE_GAIN, 1 / update_frequency}) {
  // initialize subscribers
  aircraft_state_sub_ = nh.subscribe(
      "aircraft_state", 1, &LocalPathPlannerNode::aircraftStateCallback, this);
  reference_command_sub_ =
      nh.subscribe("reference_command", 1,
                   &LocalPathPlannerNode::referenceCommandCallback, this);

  // initialize publishers
  state_command_pub_ = nh.advertise<bmb_msgs::StateCommand>("state_command", 1);
}

void LocalPathPlannerNode::aircraftStateCallback(
    const bmb_msgs::AircraftState& msg) {
  latest_aircraft_state = msg;
}

void LocalPathPlannerNode::referenceCommandCallback(
    const bmb_msgs::ReferenceCommand& msg) {
  latest_reference_command = msg;
  update_dubins_path = true;
}

double pitchFromLift(const Vector3<double>& body_vel, const double& lift) {
  const double speed_xz_squared =
      body_vel.x * body_vel.x + body_vel.z * body_vel.z;
  return std::asin(
      (lift / speed_xz_squared - bmb_world_model::BODY_B_WRENCH.force.z) /
      bmb_world_model::BODY_M_WRENCH.force.z);
}

double velocityXZFromLift(const double& sin_aoa_xz, const double& lift) {
  return std::sqrt(lift / (bmb_world_model::BODY_M_WRENCH.force.z * sin_aoa_xz +
                           bmb_world_model::BODY_B_WRENCH.force.z));
}

bmb_msgs::StateCommand LocalPathPlannerNode::getStateCommand() {
  static constexpr double SIN6 = 0.10452846326;
  static constexpr double COS6 = 0.99452189536;
  static constexpr double RAD6 = 6 * M_PI / 180;
  static constexpr double MIN_RADIUS_CURVATURE =
      30;  // calculated to be 23.5 m, at 50 km/h

  const PosVelState<double> current_state{latest_aircraft_state};
  const PosVelState<double> goal{latest_reference_command};

  if (update_dubins_path) {
    path =
        DubinsPath<double>::create(current_state, goal, MIN_RADIUS_CURVATURE);
    pursuer.updatePath(path);
    update_dubins_path = false;
  }
  auto [should_replan, angular_vel] = pursuer.pursue(current_state);
  if (should_replan) {
    path =
        DubinsPath<double>::create(current_state, goal, MIN_RADIUS_CURVATURE);
    pursuer.updatePath(path);
    std::tie(should_replan, angular_vel) = pursuer.pursue(current_state);
#if DEBUG
    assert(!should_replan);
#endif
  }

  static geometry_msgs::Vector3 b_vel = latest_aircraft_state.twist.linear;
  const Vector3<double> body_vel = {b_vel.x, b_vel.y, b_vel.z};
  const double cur_world_vel = std::hypot(body_vel.x, body_vel.z);
  const double vertical_force =
      altitude_pid.update(-latest_aircraft_state.pose.position.z,
                          latest_reference_command.altitude) +
      WEIGHT.z;  // always lift weight
  const double horizontal_force =
      MASS * cur_world_vel *
      angular_vel;  // Centripetal force, CW ang vel results in positive force
  const double net_force = std::hypot(vertical_force, horizontal_force);
  // calculate max lift, verify feasibility, calculate StateCommand
  const Wrench<double> max_wrench_at_cur_vel =
      bmb_world_model::wrenchFromAOA(body_vel, SIN6);
  const double max_vertical_force_at_cur_vel =
      -max_wrench_at_cur_vel.force.z * COS6 +
      max_wrench_at_cur_vel.force.x * SIN6;
  double required_pitch = pitchFromLift(body_vel, vertical_force);
  double required_roll = std::atan(vertical_force / horizontal_force);
  double max_roll = std::asin(WEIGHT.z / max_vertical_force_at_cur_vel);

  double commanded_vel = std::hypot(goal.vel[0], goal.vel[1]);
  double commanded_roll = required_roll;
  double commanded_pitch = bmb_utilities::saturation(required_pitch, RAD6);
  if (std::abs(required_roll) > max_roll) {
    double required_vel_xz = velocityXZFromLift(
        SIN6, net_force / COS6);  // you have to really make sure altitude pid
                                  // doesnt give very large forces
    commanded_vel = required_vel_xz * COS6;
  }
  bmb_msgs::StateCommand state_command;
  state_command.speed = commanded_vel;
  state_command.roll = commanded_roll;
  state_command.pitch = commanded_pitch;
  return state_command;
}

void LocalPathPlannerNode::spin() {
  ros::Rate rate{update_frequency};
  while (ros::ok()) {
    ros::spinOnce();
    // TODO: what happens during first few iterations when
    //  latest_reference_command and latest_aircraft_state have not yet been set
    //  for the first time
    state_command_pub_.publish(getStateCommand());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_path_planner_node");
  ros::NodeHandle nh;
  LocalPathPlannerNode node{nh, 1};
  node.spin();
}
