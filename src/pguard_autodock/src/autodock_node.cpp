

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>

namespace pearlguard_autodock
{

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

class AutodockNode : public rclcpp::Node
{
public:
  AutodockNode()
  : Node("autodock_node")
  {
    declareParameters();
    readParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "start_docking",
      std::bind(&AutodockNode::handleStartDocking, this, std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "cancel_docking",
      std::bind(&AutodockNode::handleCancelDocking, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::milliseconds(static_cast<int>(control_period_ * 1000.0));
    control_timer_ = create_wall_timer(period, std::bind(&AutodockNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "autodock_node initialise: tag_frame='%s', base_frame='%s'. En attente d'un appel a "
      "'%s/start_docking'.",
      tagFrameName().c_str(), base_frame_.c_str(), get_namespace());
  }

private:
  enum class State { IDLE, SEARCHING, APPROACH, FINAL_APPROACH, DOCKED, FAILED };

  // ---------------------------------------------------------------------
  // Parametres
  // ---------------------------------------------------------------------
  void declareParameters()
  {
    declare_parameter("target_tag_id", 0);
    declare_parameter("tag_family", std::string("tag36h11"));
    declare_parameter("tag_frame_override", std::string(""));  // vide = auto (family:id)
    declare_parameter("base_frame", std::string("base_link"));

    declare_parameter("final_stop_distance", 0.40);
    declare_parameter("align_radius", 0.60);
    declare_parameter("stop_distance_tolerance", 0.03);
    declare_parameter("final_heading_tolerance", 0.05);

    declare_parameter("approach_linear_speed", 0.25);
    declare_parameter("final_approach_linear_speed", 0.08);
    declare_parameter("max_angular_speed", 0.6);
    declare_parameter("kp_heading", 1.5);
    declare_parameter("kp_final_heading", 2.0);
    declare_parameter("kp_linear", 0.6);


    declare_parameter("kp_final_lateral", 1.0);
    declare_parameter("tag_normal_sign", 1.0);
    declare_parameter("normal_filter_alpha", 0.25);
    declare_parameter("normal_ambiguity_dot_threshold", 0.0);
    declare_parameter("camera_forward_x", 1.0);
    declare_parameter("camera_forward_y", 0.0);
    declare_parameter("tag_lost_timeout", 1.0);
    declare_parameter("search_rotate_speed", 0.3);
    declare_parameter("search_timeout", 20.0);
    declare_parameter("max_search_reattempts", 3);
    declare_parameter("enable_blind_final_creep", true);
    declare_parameter("blind_creep_speed", 0.05);
    declare_parameter("blind_creep_duration", 1.0);
    declare_parameter("blind_creep_trigger_distance", 0.15);
    declare_parameter("control_period", 0.05);
    declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    declare_parameter("status_topic", std::string("autodock/status"));
  }

  void readParameters()
  {
    target_tag_id_ = get_parameter("target_tag_id").as_int();
    tag_family_ = get_parameter("tag_family").as_string();
    tag_frame_override_ = get_parameter("tag_frame_override").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    final_stop_distance_ = get_parameter("final_stop_distance").as_double();
    align_radius_ = get_parameter("align_radius").as_double();
    stop_distance_tolerance_ = get_parameter("stop_distance_tolerance").as_double();
    final_heading_tolerance_ = get_parameter("final_heading_tolerance").as_double();

    approach_linear_speed_ = get_parameter("approach_linear_speed").as_double();
    final_approach_linear_speed_ = get_parameter("final_approach_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    kp_heading_ = get_parameter("kp_heading").as_double();
    kp_final_heading_ = get_parameter("kp_final_heading").as_double();
    kp_linear_ = get_parameter("kp_linear").as_double();

    tag_normal_sign_ = get_parameter("tag_normal_sign").as_double();

    normal_filter_alpha_ = get_parameter("normal_filter_alpha").as_double();
    normal_ambiguity_dot_threshold_ = get_parameter("normal_ambiguity_dot_threshold").as_double();

    camera_forward_x_ = get_parameter("camera_forward_x").as_double();
    camera_forward_y_ = get_parameter("camera_forward_y").as_double();
    {
      const double len = std::hypot(camera_forward_x_, camera_forward_y_);
      if (len > 1e-6) {
        camera_forward_x_ /= len;
        camera_forward_y_ /= len;
      } else {
        camera_forward_x_ = 1.0;
        camera_forward_y_ = 0.0;
      }
    }

    tag_lost_timeout_ = get_parameter("tag_lost_timeout").as_double();
    search_rotate_speed_ = get_parameter("search_rotate_speed").as_double();
    search_timeout_ = get_parameter("search_timeout").as_double();
    max_search_reattempts_ = get_parameter("max_search_reattempts").as_int();

    enable_blind_final_creep_ = get_parameter("enable_blind_final_creep").as_bool();
    blind_creep_speed_ = get_parameter("blind_creep_speed").as_double();
    blind_creep_duration_ = get_parameter("blind_creep_duration").as_double();
    blind_creep_trigger_distance_ = get_parameter("blind_creep_trigger_distance").as_double();

    control_period_ = get_parameter("control_period").as_double();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
  }

  std::string tagFrameName() const
  {
    if (!tag_frame_override_.empty()) {
      return tag_frame_override_;
    }
    return tag_family_ + ":" + std::to_string(target_tag_id_);
  }

  // ---------------------------------------------------------------------
  // Services
  // ---------------------------------------------------------------------
  void handleStartDocking(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)

    state_ = State::SEARCHING;
    search_start_time_ = now();
    search_reattempts_ = 0;
    tag_last_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    blind_creep_active_ = false;
    have_filtered_normal_ = false;
    publishStatus("SEARCHING");

    response->success = true;
    response->message = "Docking demarre.";
    RCLCPP_INFO(get_logger(), "Docking demarre (service start_docking).");
  }

  void handleCancelDocking(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stopRobot();
    state_ = State::IDLE;
    blind_creep_active_ = false;
    publishStatus("IDLE");

    response->success = true;
    response->message = "Docking annule.";
    RCLCPP_INFO(get_logger(), "Docking annule (service cancel_docking).");
  }

  // ---------------------------------------------------------------------
  // Utilitaires TF
  // ---------------------------------------------------------------------

  bool getTagInBaseFrame(double & tag_x, double & tag_y, double & normal_x, double & normal_y)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        base_frame_, tagFrameName(), tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      return false;
    }

    const rclcpp::Time stamp(tf_msg.header.stamp);
    const double age = (now() - stamp).seconds();
    if (age > tag_lost_timeout_) {
      return false; 
    }

    tag_x = tf_msg.transform.translation.x;
    tag_y = tf_msg.transform.translation.y;

    tf2::Quaternion q(
      tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
      tf_msg.transform.rotation.z, tf_msg.transform.rotation.w);
    const tf2::Matrix3x3 rot(q);

    const tf2::Vector3 local_z(0.0, 0.0, 1.0);
    const tf2::Vector3 normal_base = rot * local_z * tag_normal_sign_;

    const double n_len = std::hypot(normal_base.x(), normal_base.y());
    if (n_len < 1e-6) {

      return false;
    }
    const double raw_normal_x = normal_base.x() / n_len;
    const double raw_normal_y = normal_base.y() / n_len;


    double disambiguated_normal_x = raw_normal_x;
    double disambiguated_normal_y = raw_normal_y;
    const double dot_physical = raw_normal_x * camera_forward_x_ + raw_normal_y * camera_forward_y_;
    if (dot_physical > 0.0) {
      disambiguated_normal_x = -raw_normal_x;
      disambiguated_normal_y = -raw_normal_y;
    }


    if (!have_filtered_normal_) {
      filtered_normal_x_ = disambiguated_normal_x;
      filtered_normal_y_ = disambiguated_normal_y;
      have_filtered_normal_ = true;
    } else {
      const double dot = disambiguated_normal_x * filtered_normal_x_
        + disambiguated_normal_y * filtered_normal_y_;
      if (dot < normal_ambiguity_dot_threshold_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Normale du tag incoherente avec l'estimation precedente (dot=%.2f) : "
          "probable ambiguite de pose AprilTag, mesure ignoree.", dot);
      } else {
        filtered_normal_x_ = normal_filter_alpha_ * disambiguated_normal_x
          + (1.0 - normal_filter_alpha_) * filtered_normal_x_;
        filtered_normal_y_ = normal_filter_alpha_ * disambiguated_normal_y
          + (1.0 - normal_filter_alpha_) * filtered_normal_y_;
        const double f_len = std::hypot(filtered_normal_x_, filtered_normal_y_);
        if (f_len > 1e-6) {
          filtered_normal_x_ /= f_len;
          filtered_normal_y_ /= f_len;
        }
      }
    }
    normal_x = filtered_normal_x_;
    normal_y = filtered_normal_y_;

    tag_last_seen_time_ = stamp;
    return true;
  }

  // ---------------------------------------------------------------------
  // Boucle de controle
  // ---------------------------------------------------------------------
  void controlLoop()
  {
    switch (state_) {
      case State::IDLE:
        return;  

      case State::SEARCHING:
        runSearching();
        return;

      case State::APPROACH:
      case State::FINAL_APPROACH:
        runApproach();
        return;

      case State::DOCKED:
  
        stopRobot();
        return;

      case State::FAILED:
        stopRobot();
        return;
    }
  }

  void runSearching()
  {
    double tx, ty, nx, ny;
    if (getTagInBaseFrame(tx, ty, nx, ny)) {
      RCLCPP_INFO(get_logger(), "Tag detecte, debut de l'approche.");
      state_ = State::APPROACH;
      publishStatus("APPROACH");
      return;
    }

    if ((now() - search_start_time_).seconds() > search_timeout_) {
      RCLCPP_ERROR(
        get_logger(), "Tag '%s' introuvable apres %.1f s de recherche. Abandon.",
        tagFrameName().c_str(), search_timeout_);
      stopRobot();
      state_ = State::FAILED;
      publishStatus("FAILED");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = search_rotate_speed_;
    cmd_vel_pub_->publish(cmd);
  }

  void runApproach()
  {
    double tag_x, tag_y, normal_x, normal_y;
    const bool tag_visible = getTagInBaseFrame(tag_x, tag_y, normal_x, normal_y);

    if (!tag_visible) {
      handleTagLostDuringApproach();
      return;
    }
    blind_creep_active_ = false;

    const double target_x = tag_x + final_stop_distance_ * normal_x;
    const double target_y = tag_y + final_stop_distance_ * normal_y;
    const double distance_to_target = std::hypot(target_x, target_y);
    last_known_distance_to_target_ = distance_to_target;

    const double final_heading_error = normalizeAngle(std::atan2(-normal_y, -normal_x));

    const bool in_final_phase = distance_to_target <= align_radius_;
    state_ = in_final_phase ? State::FINAL_APPROACH : State::APPROACH;

    double heading_error;
    double linear_speed_cap;
    double kp_h;
    if (in_final_phase) {
 
      heading_error = final_heading_error;
      linear_speed_cap = final_approach_linear_speed_;
      kp_h = kp_final_heading_;
      publishStatus("FINAL_APPROACH");
    } else {
      heading_error = normalizeAngle(std::atan2(target_y, target_x));
      linear_speed_cap = approach_linear_speed_;
      kp_h = kp_heading_;
      publishStatus("APPROACH");
    }

    if (distance_to_target <= stop_distance_tolerance_ &&
      std::abs(final_heading_error) <= final_heading_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Docking termine: distance=%.3f m, cap=%.3f rad.",
        distance_to_target, final_heading_error);
      stopRobot();
      state_ = State::DOCKED;
      publishStatus("DOCKED");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(kp_h * heading_error, -max_angular_speed_, max_angular_speed_);

    const double heading_slowdown = std::max(0.15, 1.0 - std::abs(heading_error) / (M_PI / 2.0));
    cmd.linear.x = std::clamp(kp_linear_ * distance_to_target, 0.0, linear_speed_cap) * heading_slowdown;

 
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[DEBUG approach] phase=%s tag=(%.3f,%.3f) normal=(%.3f,%.3f) target=(%.3f,%.3f) "
      "dist=%.3f heading_err=%.3f rad (%.1f deg) final_heading_err=%.3f rad -> cmd(v=%.3f, w=%.3f)",
      in_final_phase ? "FINAL" : "APPROACH",
      tag_x, tag_y, normal_x, normal_y, target_x, target_y,
      distance_to_target, heading_error, heading_error * 180.0 / M_PI, final_heading_error,
      cmd.linear.x, cmd.angular.z);

    cmd_vel_pub_->publish(cmd);
  }

  void handleTagLostDuringApproach()
  {
    const double time_since_seen = (now() - tag_last_seen_time_).seconds();


    if (enable_blind_final_creep_ && !blind_creep_active_ &&
      last_known_distance_to_target_ >= 0.0 &&
      last_known_distance_to_target_ <= blind_creep_trigger_distance_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Tag perdu a %.3f m du point cible (probablement sorti du champ de la camera en fin "
        "d'approche) : creep aveugle de %.1f s.",
        last_known_distance_to_target_, blind_creep_duration_);
      blind_creep_active_ = true;
      blind_creep_start_time_ = now();
    }

    if (blind_creep_active_) {
      if ((now() - blind_creep_start_time_).seconds() < blind_creep_duration_) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = blind_creep_speed_;
        cmd_vel_pub_->publish(cmd);
        return;
      }
      RCLCPP_INFO(get_logger(), "Creep aveugle termine, docking considere reussi.");
      stopRobot();
      state_ = State::DOCKED;
      blind_creep_active_ = false;
      publishStatus("DOCKED");
      return;
    }

    if (time_since_seen <= tag_lost_timeout_ * 3.0) {

      return;
    }

    ++search_reattempts_;
    if (search_reattempts_ > max_search_reattempts_) {
      RCLCPP_ERROR(
        get_logger(), "Tag perdu de facon repetee (%d tentatives). Abandon du docking.",
        search_reattempts_);
      stopRobot();
      state_ = State::FAILED;
      publishStatus("FAILED");
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Tag perdu depuis %.1f s (tentative %d/%d) : retour en recherche.",
      time_since_seen, search_reattempts_, max_search_reattempts_);
    stopRobot();
    state_ = State::SEARCHING;
    search_start_time_ = now();
    have_filtered_normal_ = false;
    publishStatus("SEARCHING");
  }

  void stopRobot()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  // Membres
  // ---------------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  State state_ = State::IDLE;
  rclcpp::Time search_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tag_last_seen_time_{0, 0, RCL_ROS_TIME};
  int search_reattempts_ = 0;
  double last_known_distance_to_target_ = -1.0;

  bool blind_creep_active_ = false;
  rclcpp::Time blind_creep_start_time_{0, 0, RCL_ROS_TIME};

  // Parametres
  int target_tag_id_ = 0;
  std::string tag_family_ = "tag36h11";
  std::string tag_frame_override_;
  std::string base_frame_ = "base_link";

  double final_stop_distance_ = 0.40;
  double align_radius_ = 0.60;
  double stop_distance_tolerance_ = 0.03;
  double final_heading_tolerance_ = 0.05;

  double approach_linear_speed_ = 0.25;
  double final_approach_linear_speed_ = 0.08;
  double max_angular_speed_ = 0.6;
  double kp_heading_ = 1.5;
  double kp_final_heading_ = 2.0;
  double kp_linear_ = 0.6;

  double tag_normal_sign_ = 1.0;

  double normal_filter_alpha_ = 0.25;
  double normal_ambiguity_dot_threshold_ = 0.0;
  bool have_filtered_normal_ = false;
  double filtered_normal_x_ = 0.0;
  double filtered_normal_y_ = 0.0;

  double camera_forward_x_ = 1.0;
  double camera_forward_y_ = 0.0;

  double tag_lost_timeout_ = 1.0;
  double search_rotate_speed_ = 0.3;
  double search_timeout_ = 20.0;
  int max_search_reattempts_ = 3;

  bool enable_blind_final_creep_ = true;
  double blind_creep_speed_ = 0.05;
  double blind_creep_duration_ = 1.0;
  double blind_creep_trigger_distance_ = 0.15;

  double control_period_ = 0.05;
  std::string cmd_vel_topic_ = "cmd_vel";
  std::string status_topic_ = "autodock/status";
};

}  // namespace pearlguard_autodock

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pearlguard_autodock::AutodockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
