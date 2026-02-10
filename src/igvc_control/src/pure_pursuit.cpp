
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <nav2_core/controller.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace igvc_control {

class PurePursuit final : public nav2_core::Controller {
public:
  void
  configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
    m_node        = parent;
    auto node     = m_node.lock();
    m_costmap_ros = costmap_ros;
    m_tf          = tf;
    m_plugin_name = name;
    m_logger      = node->get_logger();
    m_clock       = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(
        node, m_plugin_name + ".lookahead_distance", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node, m_plugin_name + ".min_velocity", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node, m_plugin_name + ".max_velocity", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, m_plugin_name + ".wheel_base",
                                                 rclcpp::ParameterValue(1.0));

    node->get_parameter(m_plugin_name + ".lookahead_distance", p_lookahead_distance);
    node->get_parameter(m_plugin_name + ".min_velocity", p_min_velocity);
    node->get_parameter(m_plugin_name + ".max_velocity", p_max_velocity);
    node->get_parameter(m_plugin_name + ".wheel_base", p_wheel_base);
  }

  void cleanup() override {
    RCLCPP_INFO(m_logger, "Cleaning up controller: %s", m_plugin_name.c_str());
  }

  void activate() override {
    RCLCPP_INFO(m_logger, "Activating controller: %s", m_plugin_name.c_str());
  }

  void deactivate() override {
    RCLCPP_INFO(m_logger, "Deactivating up controller: %s", m_plugin_name.c_str());
  }

  void setPlan(const nav_msgs::msg::Path &path) override { m_plan = path; }

  // MARK: Plan
  // ------------------------------------------------------------------------
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker *goal_checker) override {
    (void)velocity;
    (void)goal_checker;

    auto transformed_plan = transformGlobalPlan(pose);

    // Find the first pose which is at a distance greater than the specified lookahed
    // distance
    auto goal_pose_it =
        std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(),
                     [&](const auto &ps) {
                       return hypot(ps.pose.position.x, ps.pose.position.y) >=
                              p_lookahead_distance;
                     });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }

    // Get target point coordinates
    const double x_tp = goal_pose_it->pose.position.x;
    const double y_tp = goal_pose_it->pose.position.y;

    // Calculate alpha: angle from vehicle heading to target point
    const double alpha = std::atan2(y_tp, x_tp);

    // Pure pursuit formula: δ = arctan(2 * L * sin(α) / l_d)
    const double steering_angle =
        std::atan(2.0 * p_wheel_base * std::sin(alpha) / p_lookahead_distance);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = m_clock->now();
    cmd.header.frame_id = m_costmap_ros->getBaseFrameID();
    cmd.twist.linear.x  = 2.3;
    cmd.twist.linear.y  = 0.0;
    cmd.twist.linear.z  = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = steering_angle;

    return cmd;
  }

  void setSpeedLimit(const double &speed_limit, const bool &percentage) override {
    (void)speed_limit;
    (void)percentage;
  }

  nav_msgs::msg::Path
  transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose) {
    // Original Implementation taken fron nav2_dwb_controller

    if (m_plan.poses.empty()) {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(m_tf, m_plan.header.frame_id, pose, robot_pose,
                       m_transform_tolerance)) {
      throw nav2_core::PlannerException(
          "Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = m_costmap_ros->getCostmap();
    double dist_threshold =
        std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
        costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin = nav2_util::geometry_utils::min_by(
        m_plan.poses.begin(), m_plan.poses.end(),
        [&robot_pose](const geometry_msgs::msg::PoseStamped &ps) {
          return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
        });

    // From the closest point, look for the first point that's further then
    // dist_threshold from the robot. These points are definitely outside of the
    // costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(m_plan.poses), [&](const auto &global_plan_pose) {
          return nav2_util::geometry_utils::euclidean_distance(
                     robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from
    // global frame to local
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = m_plan.header.frame_id;
      stamped_pose.header.stamp    = pose.header.stamp;
      stamped_pose.pose            = global_plan_pose.pose;
      transformPose(m_tf, m_costmap_ros->getBaseFrameID(), stamped_pose,
                    transformed_pose, m_transform_tolerance);
      return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of
    // reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(transformation_begin, transformation_end,
                   std::back_inserter(transformed_plan.poses),
                   transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = m_costmap_ros->getBaseFrameID();
    transformed_plan.header.stamp    = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    m_plan.poses.erase(begin(m_plan.poses), transformation_begin);
    // global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                     const std::string frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose,
                     const rclcpp::Duration &transform_tolerance) const {
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      tf->transform(in_pose, out_pose, frame);
      return true;
    } catch (tf2::ExtrapolationException &ex) {
      auto transform =
          tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
      if ((rclcpp::Time(in_pose.header.stamp) -
           rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
        RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                     "Transform data too old when converting from %s to %s",
                     in_pose.header.frame_id.c_str(), frame.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                     "Data time: %ds %uns, Transform time: %ds %uns",
                     in_pose.header.stamp.sec, in_pose.header.stamp.nanosec,
                     transform.header.stamp.sec, transform.header.stamp.nanosec);
        return false;
      } else {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in transformPose: %s",
                   ex.what());
      return false;
    }
    return false;
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr m_node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m_costmap_ros;
  std::shared_ptr<tf2_ros::Buffer> m_tf;
  std::string m_plugin_name;

  rclcpp::Logger m_logger{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr m_clock;

  double p_lookahead_distance;
  double p_min_velocity;
  double p_max_velocity;
  double p_wheel_base;

  nav_msgs::msg::Path m_plan;

  rclcpp::Duration m_transform_tolerance{0, 0};
};

} // namespace igvc_control

PLUGINLIB_EXPORT_CLASS(igvc_control::PurePursuit, nav2_core::Controller)