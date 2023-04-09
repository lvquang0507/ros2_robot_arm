#include <math.h>
#include <string.h>
#include <chrono>

#include "builtin_interfaces/msg/duration.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.h"

using JointState = sensor_msgs::msg::JointState;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using Duration = builtin_interfaces::msg::Duration;

class TestTrajectory : public rclcpp::Node
{
public:
  auto get_sub_param(std::string sub_param, std::string name)
  {
    struct get_sub_param_return
    {
      bool goals_ok;
      std::vector<double> goal_values;
    };

    std::string param_name = name + '.' + sub_param;
    this->declare_parameter(param_name, std::vector<double>{});
    auto param_value = this->get_parameter(param_name).as_double_array();
    if (param_value.size() != joints_.size()) {
      return get_sub_param_return{false, std::vector<double>{}};
    }
    return get_sub_param_return{true, param_value};
  };

  TestTrajectory() : Node("test_joint_trajectory_control")
  {
    this->declare_parameter("controller_name", "position_trajectory_control");
    this->declare_parameter("wait_sec_between_publish", 6);
    this->declare_parameter("goal_names", std::vector<std::string>{"pos1", "pos2"});
    this->declare_parameter("joints", std::vector<std::string>{""});
    this->declare_parameter("check_starting_point", false);

    controller_name_ = this->get_parameter("controller_name").as_string();
    wait_sec_between_publish_ = this->get_parameter("wait_sec_between_publish").as_int();
    goal_names_ = this->get_parameter("goal_names").as_string_array();
    joints_ = this->get_parameter("joints").as_string_array();
    check_starting_point_ = this->get_parameter("check_starting_point").as_bool();

    RCLCPP_INFO_STREAM(this->get_logger(), wait_sec_between_publish_);

    this->starting_point_ = {};

    if (controller_name_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Controller is empty");
    }
    if (this->check_starting_point_) {
      for (auto & joint : joints_) {
        std::string param_name_temp = "starting_point_limits" + '.' + joint;
        this->declare_parameter(param_name_temp, std::vector<double>{-2 * M_PI, 2 * M_PI});
        starting_point_[joint] = this->get_parameter(param_name_temp).as_double_array();
      }
      for (auto & joint : joints_) {
        if (starting_point_[joint].size() != 2) {
          throw std::runtime_error("Starting_point parameter is not set correctly!");
        }
      }
      joint_state_sub_ = this->create_subscription<JointState>(
        "joint_state", 10,
        std::bind(&TestTrajectory::joint_state_callback, this, std::placeholders::_1));
    }
    this->starting_point_ok_ = !this->check_starting_point_;
    this->joint_state_msg_received_ = false;

    for (auto & name : goal_names_) {
      //   auto param_desc = ParameterDescriptor{dynamic_typing = true};
      this->declare_parameter(name, "");
      // Check if input goals is format as goal1: {"joint1_position","joint2_position",...} aka vector of strings
      if (this->get_parameter(name).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        auto goal = this->get_parameter(name).as_double_array();
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Goal " << name << "defined type is deprecated. Please use the others way.");
        auto duration = Duration();
        duration.sec = 4;
        auto point = JointTrajectoryPoint();
        point.positions = goal;
        point.time_from_start = duration;

        goals_.push_back(point);
      } else {
        auto point = JointTrajectoryPoint();
        bool one_ok = false;

        auto positions = this->get_sub_param("positions", name);

        RCLCPP_INFO_STREAM(this->get_logger(), positions.goals_ok);

        if (positions.goals_ok) {
          point.positions = positions.goal_values;
          one_ok = true;
        }
        auto velocities = this->get_sub_param("velocities", name);
        if (velocities.goals_ok) {
          point.velocities = velocities.goal_values;
          one_ok = true;
        }
        auto accelerations = this->get_sub_param("accelerations", name);
        if (accelerations.goals_ok) {
          point.accelerations = accelerations.goal_values;
          one_ok = true;
        }
        auto effort = this->get_sub_param("effort", name);
        if (effort.goals_ok) {
          point.effort = effort.goal_values;
          one_ok = true;
        }
        if (one_ok) {
          auto duration = Duration();
          duration.sec = 4;
          point.time_from_start = duration;
          goals_.push_back(point);
          RCLCPP_INFO_STREAM(this->get_logger(), "Goal " << name << " has definition");
          for (auto & pos : point.positions) {
            RCLCPP_INFO_STREAM(this->get_logger(), pos);
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Goals are wrong");
        }
      }
    }
    auto publish_topic = "/" + controller_name_ + "/" + "joint_trajectory";
    RCLCPP_INFO(
      this->get_logger(), "Publishing on topic %s every %ds", publish_topic.c_str(),
      wait_sec_between_publish_);
    joint_trajectory_pub_ = this->create_publisher<JointTrajectory>(publish_topic, 1);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(wait_sec_between_publish_),
      std::bind(&TestTrajectory::timer_callback, this));
    index_ = 0;
  }

private:
  void joint_state_callback(JointState::SharedPtr msg)
  {
    if (!this->joint_state_msg_received_) {
      int number_of_joints = (msg->name).size();

      std::vector<bool> limit_exceed(number_of_joints, false);
      for (int i = 0; i < number_of_joints; i++) {
        std::string joint_name = msg->name[i];
        double joint_value_0 = (this->starting_point_.at(joint_name))[0];
        double joint_value_1 = (this->starting_point_.at(joint_name))[1];

        if (msg->position[i] < joint_value_0 || msg->position[i] > joint_value_1) {
          RCLCPP_WARN(this->get_logger(), "Starting point limits exceed for joint %s", joint_name);
          limit_exceed.at(i) = true;
        }
        for (int i = 0; i < (int)limit_exceed.size(); i++) {
          if (limit_exceed[i] == true) {
            this->starting_point_ok_ = false;
            break;
          } else {
            this->starting_point_ok_ = true;
          }
        }
        this->joint_state_msg_received_ = true;
      }
    } else {
      return;
    };
  }
  void timer_callback()
  {
    if (this->starting_point_ok_) {
      RCLCPP_INFO(this->get_logger(), "Sending goal");  //TODO log all goals
      auto traj = JointTrajectory();
      traj.joint_names = this->joints_;
      traj.points.push_back(goals_[index_]);
      joint_trajectory_pub_->publish(traj);
      index_++;
      index_ %= goals_.size();
    } else if (check_starting_point_ && !joint_state_msg_received_) {
      RCLCPP_WARN(this->get_logger(), "Start configuration could not be checked!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Start configuration is not within configured limits!");
    }
  }

  std::string controller_name_;
  int wait_sec_between_publish_;
  std::vector<std::string> goal_names_;
  std::vector<std::string> joints_;
  bool check_starting_point_;
  bool starting_point_ok_;
  bool joint_state_msg_received_;
  std::vector<JointTrajectoryPoint> goals_;
  int index_;

  std::map<std::string, std::vector<double>> starting_point_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr joint_trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestTrajectory>());
  rclcpp::shutdown();
  return 0;
}
