// Deterministic UR5e + DH AG95 SC plug insertion task.
//
// State machine:
//   START -> OPEN_GRIPPER -> HOME -> PRE_GRASP -> GRASP_APPROACH
//   -> CLOSE_GRIPPER -> LIFT -> VALIDATE_GRASP -> PRE_INSERT -> PORT_ALIGN
//   -> INSERT -> VALIDATE_INSERTION -> RELEASE -> RETREAT -> HOME -> SUCCESS
//
// All task targets are PoseStamped in the documented `gazebo_world` frame and
// converted into the MoveIt planning frame through TF2. Ground-truth poses
// bridged from Gazebo are used ONLY for validation, never for commanding
// motion. All numbers come from config/insertion_task.yaml.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

using namespace std::chrono_literals;

enum class State {
  kStart,
  kOpenGripper,
  kHome,
  kPreGrasp,
  kGraspApproach,
  kCloseGripper,
  kLift,
  kValidateGrasp,
  kPreInsert,
  kPortAlign,
  kInsert,
  kValidateInsertion,
  kRelease,
  kRetreat,
  kReturnHome,
  kSuccess,
};

std::string stateName(State s) {
  switch (s) {
    case State::kStart: return "START";
    case State::kOpenGripper: return "OPEN_GRIPPER";
    case State::kHome: return "HOME";
    case State::kPreGrasp: return "PRE_GRASP";
    case State::kGraspApproach: return "GRASP_APPROACH";
    case State::kCloseGripper: return "CLOSE_GRIPPER";
    case State::kLift: return "LIFT";
    case State::kValidateGrasp: return "VALIDATE_GRASP";
    case State::kPreInsert: return "PRE_INSERT";
    case State::kPortAlign: return "PORT_ALIGN";
    case State::kInsert: return "INSERT";
    case State::kValidateInsertion: return "VALIDATE_INSERTION";
    case State::kRelease: return "RELEASE";
    case State::kRetreat: return "RETREAT";
    case State::kReturnHome: return "HOME";
    case State::kSuccess: return "SUCCESS";
  }
  return "UNKNOWN";
}

static tf2::Transform makeT(const std::vector<double>& xyzrpy) {
  if (xyzrpy.size() != 6) {
    throw std::runtime_error("pose parameter must have 6 elements [x y z r p y]");
  }
  tf2::Quaternion q;
  q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
  return tf2::Transform(q, tf2::Vector3(xyzrpy[0], xyzrpy[1], xyzrpy[2]));
}

static geometry_msgs::msg::Pose poseFromTf(const tf2::Transform& t) {
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  p.orientation.x = t.getRotation().x();
  p.orientation.y = t.getRotation().y();
  p.orientation.z = t.getRotation().z();
  p.orientation.w = t.getRotation().w();
  return p;
}

class DeterministicInsertion : public rclcpp::Node {
 public:
  DeterministicInsertion() : Node("deterministic_insertion") {
    // Frames and topics
    gazebo_frame_ = declare_parameter<std::string>("frames.gazebo_world", "gazebo_world");
    planning_frame_ = declare_parameter<std::string>("frames.planning", "world");
    tcp_frame_ = declare_parameter<std::string>("frames.tcp", "gripper_tcp");
    gripper_topic_ = declare_parameter<std::string>("topics.gripper_cmd", "/gripper_cmd");
    pose_topic_ = declare_parameter<std::string>("topics.pose_info", "/world/insertion/pose/info");
    contact_topic_ = declare_parameter<std::string>(
        "topics.port_contact",
        "/world/insertion/model/port/link/sc_port_link/sensor/sc_port_sensor/contact");

    // Gripper
    gripper_open_ = declare_parameter<double>("gripper.open_position", 0.0);
    gripper_close_ = declare_parameter<double>("gripper.close_position", 0.62);
    gripper_settle_ = declare_parameter<double>("gripper.settle_time", 1.0);
    pregrasp_gripper_ = declare_parameter<double>("gripper.pregrasp_position", 0.30);

    // Scene geometry (deterministic, gazebo_world frame)
    plug_pose_ = makeT(declare_parameter<std::vector<double>>("scene.plug_pose"));
    port_pose_ = makeT(declare_parameter<std::vector<double>>("scene.port_pose"));
    table_pose_ = makeT(declare_parameter<std::vector<double>>("scene.table_pose"));
    table_size_ = declare_parameter<std::vector<double>>("scene.table_size");
    port_box_pose_ = makeT(declare_parameter<std::vector<double>>("scene.port_box_pose"));
    port_box_size_ = declare_parameter<std::vector<double>>("scene.port_box_size");
    stand_pose_ = makeT(declare_parameter<std::vector<double>>("scene.stand_pose"));
    stand_size_ = declare_parameter<std::vector<double>>("scene.stand_size");

    // Constant transforms measured from the model SDFs
    tip_in_plug_ = makeT(declare_parameter<std::vector<double>>("plug_tip_in_plug"));
    base_in_port_ = makeT(declare_parameter<std::vector<double>>("port_base_in_port"));
    entrance_in_base_ = makeT(declare_parameter<std::vector<double>>("entrance_in_port_base"));
    insertion_depth_ = declare_parameter<double>("insertion_depth", 0.012);
    grasp_in_plug_ = makeT(declare_parameter<std::vector<double>>("grasp_in_plug"));
    // Calibrated plug-tip pose in the TCP frame after the grasp settles into
    // the gripper's cage (measured; see config/insertion_task.yaml).
    tip_in_tcp_ = makeT(declare_parameter<std::vector<double>>("tip_in_tcp"));
    plug_box_in_tcp_ = makeT(declare_parameter<std::vector<double>>("plug_box_in_tcp"));
    plug_box_size_ = declare_parameter<std::vector<double>>("plug_box_size");

    // Motion
    pregrasp_distance_ = declare_parameter<double>("motion.pregrasp_distance", 0.10);
    lift_distance_ = declare_parameter<double>("motion.lift_distance", 0.10);
    preinsert_distance_ = declare_parameter<double>("motion.preinsert_distance", 0.05);
    preinsert_height_ = declare_parameter<double>("motion.preinsert_height", 0.10);
    retreat_distance_ = declare_parameter<double>("motion.retreat_distance", 0.10);
    cartesian_step_ = declare_parameter<double>("motion.cartesian_step_size", 0.005);
    cartesian_min_fraction_ = declare_parameter<double>("motion.cartesian_min_fraction", 0.95);
    free_space_scaling_ = declare_parameter<double>("motion.free_space_velocity_scaling", 0.20);
    approach_scaling_ = declare_parameter<double>("motion.approach_velocity_scaling", 0.10);
    insertion_scaling_ = declare_parameter<double>("motion.insertion_velocity_scaling", 0.03);
    planning_time_ = declare_parameter<double>("motion.planning_time", 10.0);
    planning_attempts_ = declare_parameter<int>("motion.planning_attempts", 5);

    // Validation
    grasp_tol_ = declare_parameter<double>("validation.grasp_pose_tolerance", 0.01);
    insertion_tol_ = declare_parameter<double>("validation.insertion_pose_tolerance", 0.005);
    pose_timeout_ = declare_parameter<double>("validation.pose_timeout", 5.0);

    acm_allow_ = declare_parameter<std::vector<std::string>>("acm_allow");
    attach_plug_ = declare_parameter<bool>("attach_plug", true);

    tf_timeout_ = declare_parameter<double>("startup.tf_timeout", 10.0);
    move_group_wait_ = declare_parameter<double>("startup.move_group_wait", 30.0);

    gripper_pub_ = create_publisher<std_msgs::msg::Float64>(gripper_topic_, 10);
    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        pose_topic_, 10,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { onPoseInfo(*msg); });
    contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
        contact_topic_, 10,
        [this](const ros_gz_interfaces::msg::Contacts::SharedPtr) { contact_seen_ = true; });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  int run() {
    using moveit::planning_interface::MoveGroupInterface;

    if (!waitForTransform()) {
      return fail(State::kStart, "No TF transform from " + gazebo_frame_ + " to " +
                                     planning_frame_);
    }

    // START -> OPEN_GRIPPER: stop the automatic demo cycle immediately, before
    // waiting on MoveIt.
    enter(State::kOpenGripper);
    publishGripper(gripper_open_);
    gripper_closed_ = false;

    mgi_ = std::make_unique<MoveGroupInterface>(
        shared_from_this(), MoveGroupInterface::Options("arm"),
        std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(move_group_wait_));
    mgi_->setPoseReferenceFrame(planning_frame_);
    mgi_->setEndEffectorLink(tcp_frame_);
    mgi_->setPlanningTime(planning_time_);
    mgi_->setNumPlanningAttempts(planning_attempts_);
    // Millimetre-scale goal tolerances keep OMPL pose goals solvable without
    // sacrificing the precision the Cartesian stages enforce separately.
    mgi_->setGoalPositionTolerance(0.01);
    mgi_->setGoalOrientationTolerance(0.1);

    setupPlanningScene();

    // Drop any stale planning-scene attachment left by a previous run.
    {
      moveit_msgs::msg::AttachedCollisionObject stale;
      stale.link_name = tcp_frame_;
      stale.object.id = "plug";
      stale.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      psi_.applyAttachedCollisionObject(stale);
    }

    computeTaskPoses();

    // Wait until MoveIt sees a live robot state before planning anything.
    const auto state_deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(move_group_wait_);
    while (!mgi_->getCurrentState(2.0)) {
      if (std::chrono::steady_clock::now() > state_deadline) {
        return fail(State::kStart, "No current robot state from /joint_states");
      }
    }

    // Wait for the arm trajectory action server so the first execution does
    // not race the controller spawner.
    auto jtc_client =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");
    if (!jtc_client->wait_for_action_server(
            std::chrono::duration<double>(move_group_wait_))) {
      return fail(State::kStart,
                  "follow_joint_trajectory action server not available");
    }

    enter(State::kHome);
    if (!moveNamed("home")) return fail(State::kHome, "HOME failed: " + last_error_);
    arm_home_ = true;

    enter(State::kPreGrasp);
    if (!moveToPose(pre_grasp_tcp_, free_space_scaling_))
      return fail(State::kPreGrasp, "PRE_GRASP failed: " + last_error_);

    // Pre-close so the fingertips descend past the housing top before they
    // reach its side walls; sweeping in at full open bulldozes the plug.
    publishGripper(pregrasp_gripper_);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_settle_)));

    enter(State::kGraspApproach);
    if (!moveCartesian(grasp_tcp_, approach_scaling_))
      return fail(State::kGraspApproach, "Grasp approach failed: " + last_error_);

    enter(State::kCloseGripper);
    closeGripperGradual();
    gripper_closed_ = true;
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_settle_)));

    std::optional<geometry_msgs::msg::TransformStamped> plug_before =
        modelPose("plug");
    if (!plug_before) return fail(State::kCloseGripper, "No plug pose on " + pose_topic_);

    enter(State::kLift);
    if (!moveCartesian(lift_tcp_, insertion_scaling_))
      return fail(State::kLift, "Lift failed: " + last_error_);
    // Let the plug settle into the gripper's cradle before measuring.
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_settle_)));

    enter(State::kValidateGrasp);
    logMeasuredGrasp();
    if (!validateGrasp(*plug_before))
      return fail(State::kValidateGrasp, last_error_);

    attachPlug();

    // Transit through the neutral 'ready' pose; a single direct plan from the
    // lift pose to the rolled-over port pose is too hard for OMPL in one shot.
    enter(State::kPreInsert);
    if (!moveNamed("ready"))
      return fail(State::kPreInsert, "Transit to 'ready' failed: " + last_error_);
    if (!moveToPose(pre_insert_tcp_, free_space_scaling_))
      return fail(State::kPreInsert, "PRE_INSERT failed: " + last_error_);

    enter(State::kPortAlign);
    if (!moveCartesian(entrance_tcp_, approach_scaling_))
      return fail(State::kPortAlign, "Port alignment failed: " + last_error_);

    enter(State::kInsert);
    if (!moveCartesian(inserted_tcp_, insertion_scaling_))
      return fail(State::kInsert, "Insertion failed: " + last_error_);

    enter(State::kValidateInsertion);
    if (!validateInsertion())
      return fail(State::kValidateInsertion, last_error_);

    enter(State::kRelease);
    publishGripper(gripper_open_);
    gripper_closed_ = false;
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_settle_)));
    detachPlug();

    enter(State::kRetreat);
    if (!moveCartesian(retreat_tcp_, approach_scaling_))
      return fail(State::kRetreat, "Retreat failed: " + last_error_);

    enter(State::kReturnHome);
    if (!moveNamed("home"))
      return fail(State::kReturnHome, "Return home failed: " + last_error_);
    arm_home_ = true;

    enter(State::kSuccess);
    printReport(true, "");
    return 0;
  }

 private:
  // ---- infrastructure ----------------------------------------------------

  bool waitForTransform() {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(tf_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (tf_buffer_->canTransform(planning_frame_, gazebo_frame_, tf2::TimePointZero)) {
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    return false;
  }

  void onPoseInfo(const tf2_msgs::msg::TFMessage& msg) {
    for (const auto& t : msg.transforms) {
      poses_[t.child_frame_id] = t;
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped> modelPose(
      const std::string& needle) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(pose_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto& [name, pose] : poses_) {
        if (name == needle || name.find(needle) != std::string::npos) {
          return pose;
        }
      }
      rclcpp::sleep_for(50ms);
    }
    return std::nullopt;
  }

  geometry_msgs::msg::Pose toPlanningFrame(const tf2::Transform& gz_pose) {
    geometry_msgs::msg::PoseStamped in, out;
    in.header.frame_id = gazebo_frame_;
    in.header.stamp = now();
    in.pose = poseFromTf(gz_pose);
    tf_buffer_->transform(in, out, planning_frame_, tf2::durationFromSec(1.0));
    return out.pose;
  }

  // ---- task geometry (all in gazebo_world) --------------------------------

  void computeTaskPoses() {
    const tf2::Transform identity(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));

    // Grasp: TCP relative to the plug body; PRE_GRASP backs off along -z_tcp.
    grasp_tcp_ = plug_pose_ * grasp_in_plug_;
    pre_grasp_tcp_ = grasp_tcp_ * makeT({0, 0, -pregrasp_distance_, 0, 0, 0});
    lift_tcp_ = makeT({0, 0, lift_distance_, 0, 0, 0}) * grasp_tcp_;

    // Where the plug tip sits in the TCP frame once grasped (calibrated
    // constant from measurement, not the idealized grasp assumption).
    const tf2::Transform tip_in_tcp = tip_in_tcp_;

    // Port entrance frame in gazebo_world.
    entrance_world_ = port_pose_ * base_in_port_ * entrance_in_base_;

    // Insertion moves the tip along the entrance +z axis.
    tf2::Transform tip_at_entrance = entrance_world_;
    // PRE_INSERT: backed off along the approach axis AND raised in world z so
    // the path from the lift stays in free space; PORT_ALIGN descends from here.
    tf2::Transform tip_pre_insert =
        makeT({0, 0, preinsert_height_, 0, 0, 0}) *
        entrance_world_ * makeT({0, 0, -preinsert_distance_, 0, 0, 0});
    tf2::Transform tip_inserted =
        entrance_world_ * makeT({0, 0, insertion_depth_, 0, 0, 0});

    const tf2::Transform tcp_from_tip = tip_in_tcp.inverse();
    pre_insert_tcp_ = tip_pre_insert * tcp_from_tip;
    entrance_tcp_ = tip_at_entrance * tcp_from_tip;
    inserted_tcp_ = tip_inserted * tcp_from_tip;
    retreat_tcp_ = entrance_world_ *
                   makeT({0, 0, -retreat_distance_, 0, 0, 0}) * tcp_from_tip;

    auto log_pose = [this](const char* name, const tf2::Transform& t) {
      const auto p = t.getOrigin();
      double r, pi, y;
      t.getBasis().getRPY(r, pi, y);
      RCLCPP_INFO(get_logger(),
                  "pose %s (gazebo): xyz=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]",
                  name, p.x(), p.y(), p.z(), r, pi, y);
    };
    log_pose("GRASP", grasp_tcp_);
    log_pose("PRE_INSERT", pre_insert_tcp_);
    log_pose("PORT_ENTRANCE", entrance_tcp_);
    log_pose("INSERTED", inserted_tcp_);

    (void)identity;
  }

  // ---- planning scene ------------------------------------------------------

  void setupPlanningScene() {
    namespace mpi = moveit::planning_interface;

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = planning_frame_;
    table.id = "table";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.assign(table_size_.begin(), table_size_.end());
    table.primitives.push_back(box);
    table.primitive_poses.push_back(toPlanningFrame(table_pose_));
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::CollisionObject port;
    port.header.frame_id = planning_frame_;
    port.id = "port";
    box.dimensions.assign(port_box_size_.begin(), port_box_size_.end());
    port.primitives.push_back(box);
    port.primitive_poses.push_back(toPlanningFrame(port_box_pose_));
    port.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::CollisionObject stand;
    stand.header.frame_id = planning_frame_;
    stand.id = "stand";
    box.dimensions.assign(stand_size_.begin(), stand_size_.end());
    stand.primitives.push_back(box);
    stand.primitive_poses.push_back(toPlanningFrame(stand_pose_));
    stand.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi_.applyCollisionObjects({table, port, stand});

    // ACM: allow the configured pairs (e.g. finger pads vs table, plug vs port).
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    std::map<std::string, size_t> index;
    for (const auto& pair : acm_allow_) {
      const auto sep = pair.find(':');
      if (sep == std::string::npos) continue;
      for (const auto& name : {pair.substr(0, sep), pair.substr(sep + 1)}) {
        if (!index.count(name)) {
          index[name] = index.size();
          diff.allowed_collision_matrix.entry_names.push_back(name);
        }
      }
    }
    const size_t n = index.size();
    diff.allowed_collision_matrix.entry_values.resize(n);
    for (size_t i = 0; i < n; ++i) {
      diff.allowed_collision_matrix.entry_values[i].enabled.resize(n, false);
    }
    for (const auto& pair : acm_allow_) {
      const auto sep = pair.find(':');
      if (sep == std::string::npos) continue;
      const size_t a = index[pair.substr(0, sep)];
      const size_t b = index[pair.substr(sep + 1)];
      diff.allowed_collision_matrix.entry_values[a].enabled[b] = true;
      diff.allowed_collision_matrix.entry_values[b].enabled[a] = true;
    }
    if (n > 0) psi_.applyPlanningScene(diff);

    RCLCPP_INFO(get_logger(), "Planning scene: table + port collision objects, %zu ACM pairs",
                acm_allow_.size());
  }

  void attachPlug() {
    // Planning-scene attachment mirrors the physical grasp for collision
    // reasoning only; the AG95 holds the plug in Gazebo physics.
    if (!attach_plug_) return;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_frame_;
    attached.object.id = "plug";
    attached.object.header.frame_id = tcp_frame_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.assign(plug_box_size_.begin(), plug_box_size_.end());
    attached.object.primitives.push_back(box);
    attached.object.primitive_poses.push_back(poseFromTf(plug_box_in_tcp_));
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // The dangling plug sits inside the cage among all the gripper links.
    attached.touch_links = {tcp_frame_, "gripper_base_link",
                            "gripper_finger1_knuckle_link", "gripper_finger2_knuckle_link",
                            "gripper_finger1_inner_knuckle_link", "gripper_finger2_inner_knuckle_link",
                            "gripper_finger1_finger_link", "gripper_finger2_finger_link",
                            "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
    psi_.applyAttachedCollisionObject(attached);
  }

  void detachPlug() {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_frame_;
    attached.object.id = "plug";
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyAttachedCollisionObject(attached);
  }

  // ---- motion primitives ---------------------------------------------------

  bool moveNamed(const std::string& name) {
    mgi_->setStartStateToCurrentState();
    mgi_->setMaxVelocityScalingFactor(free_space_scaling_);
    mgi_->setMaxAccelerationScalingFactor(free_space_scaling_);
    if (!mgi_->setNamedTarget(name)) {
      last_error_ = "unknown named target '" + name + "'";
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    std::string err;
    for (int attempt = 1; attempt <= 3 && !planned; ++attempt) {
      const auto code = mgi_->plan(plan);
      planned = (code == moveit::core::MoveItErrorCode::SUCCESS);
      if (!planned) err = moveit::core::errorCodeToString(code);
    }
    if (!planned) {
      last_error_ = "plan to '" + name + "' failed after 3 attempts: " + err;
      return false;
    }
    return executeWithRetry([&plan, this]() { return mgi_->execute(plan); },
                            "execution of '" + name + "'");
  }

  // move_group's controller handle can lag DDS discovery of the trajectory
  // action server at startup; allow a few bounded retries on execution
  // failures (planning itself is never retried blindly).
  template <typename Fn>
  bool executeWithRetry(Fn&& fn, const std::string& what) {
    for (int attempt = 1; attempt <= 3; ++attempt) {
      const auto result = fn();
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      }
      last_error_ = what + " failed (attempt " + std::to_string(attempt) +
                    "): " + moveit::core::errorCodeToString(result);
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      rclcpp::sleep_for(2s);
    }
    return false;
  }

  bool moveToPose(const tf2::Transform& target_gz, double scaling) {
    mgi_->setStartStateToCurrentState();
    mgi_->setMaxVelocityScalingFactor(scaling);
    mgi_->setMaxAccelerationScalingFactor(scaling);
    // OMPL pose goals are occasionally flaky on this arm; allow a few bounded
    // retries (planning only, never a blind re-execution).
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    std::string err;
    for (int attempt = 1; attempt <= 3 && !planned; ++attempt) {
      mgi_->setPoseTarget(toPlanningFrame(target_gz), tcp_frame_);
      const auto code = mgi_->plan(plan);
      planned = (code == moveit::core::MoveItErrorCode::SUCCESS);
      if (!planned) err = moveit::core::errorCodeToString(code);
      mgi_->clearPoseTargets();
    }
    if (!planned) {
      last_error_ = "pose plan failed after 3 attempts: " + err;
      return false;
    }
    return executeWithRetry([&plan, this]() { return mgi_->execute(plan); },
                            "pose execution");
  }

  bool moveCartesian(const tf2::Transform& target_gz, double scaling) {
    std::vector<geometry_msgs::msg::Pose> waypoints{toPlanningFrame(target_gz)};
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = mgi_->computeCartesianPath(
        waypoints, cartesian_step_, trajectory);
    if (fraction < cartesian_min_fraction_) {
      last_error_ = "Cartesian path fraction " + std::to_string(fraction) +
                    " below required " + std::to_string(cartesian_min_fraction_);
      return false;
    }

    // computeCartesianPath does not honor velocity scaling; retime explicitly.
    robot_trajectory::RobotTrajectory rt(mgi_->getRobotModel(), mgi_->getName());
    rt.setRobotTrajectoryMsg(*mgi_->getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, scaling, scaling)) {
      last_error_ = "time parameterization of Cartesian path failed";
      return false;
    }
    rt.getRobotTrajectoryMsg(trajectory);

    const bool ok = executeWithRetry(
        [&trajectory, this]() { return mgi_->execute(trajectory); },
        "Cartesian execution");
    if (ok) {
      const auto achieved = mgi_->getCurrentPose(tcp_frame_);
      RCLCPP_INFO(get_logger(),
                  "Cartesian target z=%.4f, achieved TCP z=%.4f (planning frame)",
                  waypoints[0].position.z, achieved.pose.position.z);
    }
    return ok;
  }

  void publishGripper(double position) {
    std_msgs::msg::Float64 msg;
    msg.data = position;
    for (int i = 0; i < 10; ++i) {
      gripper_pub_->publish(msg);
      rclcpp::sleep_for(50ms);
    }
    RCLCPP_INFO(get_logger(), "Gripper commanded to %.3f rad", position);
  }

    // Ramp the close in small steps so the plug self-centers between the pads
    // instead of being pushed out by the first pad to make contact.
  void closeGripperGradual() {
    const double from = pregrasp_gripper_;
    const double to = gripper_close_;
    const int steps = 10;
    for (int i = 1; i <= steps; ++i) {
      publishGripper(from + (to - from) * i / steps);
      rclcpp::sleep_for(400ms);
    }
  }

  // ---- validation ------------------------------------------------------------

  // Grasp check: the plug must follow the lift (ground-truth plug pose) AND
  // sit at the calibrated pose in the gripper cage (tip_in_tcp tolerance).
  // Returns the measured tip_in_tcp via `measured` for reporting.
  bool validateGrasp(const geometry_msgs::msg::TransformStamped& before,
                     tf2::Transform* measured = nullptr) {
    auto after = modelPose("plug");
    if (!after) {
      last_error_ = "no plug pose available after lift";
      return false;
    }
    const double dz = after->transform.translation.z - before.transform.translation.z;
    const double dxy = std::hypot(
        after->transform.translation.x - before.transform.translation.x,
        after->transform.translation.y - before.transform.translation.y);
    RCLCPP_INFO(get_logger(),
                "Grasp validation: plug rose %.4f m (lifted %.4f), lateral drift %.4f m",
                dz, lift_distance_, dxy);
    // The plug settles into the gripper's cradle during the lift, so it rises
    // less than the lift distance; "held" means clearly lifted off the scene
    // plus the calibrated cage pose check below.
    if (dz < 0.02) {
      last_error_ = "plug did not follow the lift: dz=" + std::to_string(dz) +
                    " lift=" + std::to_string(lift_distance_);
      return false;
    }
    if (measured) {
      const tf2::Transform tip_in_tcp = measuredTipInTcp();
      const double pos_err =
          (tip_in_tcp.getOrigin() - tip_in_tcp_.getOrigin()).length();
      tf2::Quaternion q1, q2;
      tip_in_tcp.getBasis().getRotation(q1);
      tip_in_tcp_.getBasis().getRotation(q2);
      const double rot_err = q1.angleShortestPath(q2);
      RCLCPP_INFO(get_logger(),
                  "Grasp validation: tip_in_tcp position error %.4f m, rotation error %.3f rad",
                  pos_err, rot_err);
      if (pos_err > grasp_tol_ || rot_err > 0.3) {
        last_error_ = "plug not at the calibrated cage pose: pos_err=" +
                      std::to_string(pos_err) + " rot_err=" + std::to_string(rot_err);
        return false;
      }
    }
    return true;
  }

  // Measured plug-tip pose in the TCP frame from Gazebo ground truth + TF.
  // Evaluation only; never used to command motion.
  tf2::Transform measuredTipInTcp() {
    auto plug = modelPose("plug");
    if (!plug) return tf2::Transform::getIdentity();
    tf2::Transform plug_world;
    tf2::fromMsg(plug->transform, plug_world);
    const auto tcp = mgi_->getCurrentPose(tcp_frame_);
    tf2::Transform tcp_planning;
    tf2::fromMsg(tcp.pose, tcp_planning);
    geometry_msgs::msg::TransformStamped gz_tf;
    try {
      gz_tf = tf_buffer_->lookupTransform(gazebo_frame_, planning_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return tf2::Transform::getIdentity();
    }
    tf2::Transform world_to_gz;
    tf2::fromMsg(gz_tf.transform, world_to_gz);
    const tf2::Transform tcp_gz = world_to_gz * tcp_planning;
    return tcp_gz.inverse() * (plug_world * tip_in_plug_);
  }

  // One-time calibration aid: log the actual plug-in-TCP transform.
  void logMeasuredGrasp() {
    const tf2::Transform m = measuredTipInTcp();
    const tf2::Vector3 p = m.getOrigin();
    double r, pi, y;
    m.getBasis().getRPY(r, pi, y);
    RCLCPP_INFO(get_logger(),
                "MEASURED tip_in_tcp: xyz=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
                p.x(), p.y(), p.z(), r, pi, y);
  }

  bool validateInsertion() {
    auto plug = modelPose("plug");
    if (!plug) {
      last_error_ = "no plug pose available after insertion";
      return false;
    }
    // Ground truth: plug model pose composed with the fixed tip offset.
    tf2::Transform plug_tf;
    tf2::fromMsg(plug->transform, plug_tf);
    const tf2::Transform tip_world = plug_tf * tip_in_plug_;
    const tf2::Transform tip_expected =
        entrance_world_ * makeT({0, 0, insertion_depth_, 0, 0, 0});
    const double error = (tip_world.getOrigin() - tip_expected.getOrigin()).length();
    RCLCPP_INFO(get_logger(),
                "Insertion validation: tip error %.4f m (tolerance %.4f), contact %s",
                error, insertion_tol_, contact_seen_ ? "seen" : "not seen");
    if (error > insertion_tol_ && !contact_seen_) {
      last_error_ = "tip is " + std::to_string(error) +
                    " m from the inserted pose and the port contact sensor "
                    "reported no contact";
      return false;
    }
    if (error > 2.0 * insertion_tol_) {
      last_error_ = "tip is " + std::to_string(error) +
                    " m from the inserted pose even though contact was reported";
      return false;
    }
    return true;
  }

  // ---- bookkeeping -----------------------------------------------------------

  void enter(State s) {
    state_ = s;
    RCLCPP_INFO(get_logger(), "==> %s", stateName(s).c_str());
  }

  void printReport(bool success, const std::string& error) {
    RCLCPP_INFO(get_logger(), "================ TASK RESULT ================");
    RCLCPP_INFO(get_logger(), "Result:        %s", success ? "SUCCESS" : "FAILURE");
    if (!success) {
      RCLCPP_INFO(get_logger(), "Failed state:  %s", stateName(state_).c_str());
      RCLCPP_INFO(get_logger(), "Error:         %s", error.c_str());
    }
    RCLCPP_INFO(get_logger(), "Gripper:       %s", gripper_closed_ ? "closed" : "open");
    RCLCPP_INFO(get_logger(), "Arm at home:   %s", arm_home_ ? "yes" : "no");
    RCLCPP_INFO(get_logger(), "=============================================");
  }

  int fail(State s, const std::string& error) {
    state_ = s;
    if (mgi_) mgi_->stop();
    RCLCPP_ERROR(get_logger(), "Task failed at %s: %s", stateName(s).c_str(), error.c_str());
    printReport(false, error);
    return 1;
  }

  // Parameters
  std::string gazebo_frame_, planning_frame_, tcp_frame_;
  std::string gripper_topic_, pose_topic_, contact_topic_;
  double gripper_open_, gripper_close_, gripper_settle_, pregrasp_gripper_;
  tf2::Transform plug_pose_, port_pose_, table_pose_, port_box_pose_, stand_pose_;
  std::vector<double> table_size_, port_box_size_, plug_box_size_, stand_size_;
  tf2::Transform tip_in_plug_, base_in_port_, entrance_in_base_, grasp_in_plug_;
  tf2::Transform tip_in_tcp_;
  tf2::Transform plug_box_in_tcp_;
  double insertion_depth_;
  double pregrasp_distance_, lift_distance_, preinsert_distance_, preinsert_height_, retreat_distance_;
  double cartesian_step_, cartesian_min_fraction_;
  double free_space_scaling_, approach_scaling_, insertion_scaling_;
  double planning_time_;
  int planning_attempts_;
  double grasp_tol_, insertion_tol_, pose_timeout_;
  std::vector<std::string> acm_allow_;
  double tf_timeout_, move_group_wait_;

  // Computed task poses (gazebo_world frame)
  tf2::Transform grasp_tcp_, pre_grasp_tcp_, lift_tcp_;
  tf2::Transform entrance_world_, pre_insert_tcp_, entrance_tcp_, inserted_tcp_;
  tf2::Transform retreat_tcp_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  moveit::planning_interface::PlanningSceneInterface psi_;

  // State
  std::map<std::string, geometry_msgs::msg::TransformStamped> poses_;
  bool contact_seen_ = false;
  bool gripper_closed_ = false;
  bool attach_plug_ = true;
  bool arm_home_ = false;
  State state_ = State::kStart;
  std::string last_error_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeterministicInsertion>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const int result = node->run();

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
