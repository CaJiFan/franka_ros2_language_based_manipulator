#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <atomic> // For thread-safe flags

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "std_msgs/msg/string.hpp"

// --- GLOBAL FLAGS ---
// Atomic ensures the background thread can talk to the main thread safely
std::atomic<bool> release_signal_received(false);

// --- SUBSCRIBER CALLBACK ---
void intent_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Check if the message matches the JSON format expected
    // We look for the substring "'action':'release'"
    std::string data = msg->data;
    if (data.find("'action':'release'") != std::string::npos || 
        data.find("\"action\":\"release\"") != std::string::npos) { // Handle both ' and " quotes
        
        release_signal_received = true;
        // Print to console so you know it worked
        std::cout << "\n[HMI] RELEASE SIGNAL RECEIVED! Dropping object...\n" << std::endl;
    }
}

// --- HELPER: Create Pose ---
geometry_msgs::msg::Pose create_pose(double x, double y, double z, double r, double p, double yw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(r * M_PI/180.0, p * M_PI/180.0, yw * M_PI/180.0);
    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
}

// --- HELPER: Action Clients ---
// void open_gripper(rclcpp::Node::SharedPtr node) {
//     using MoveAction = franka_msgs::action::Move;
//     auto action_client = rclcpp_action::create_client<MoveAction>(node, "/franka_gripper/move");
//     if (!action_client->wait_for_action_server(std::chrono::seconds(2))) return;
//     auto goal = MoveAction::Goal();
//     goal.width = 0.08; 
//     goal.speed = 0.1;
//     action_client->async_send_goal(goal);
// }

void open_gripper(rclcpp::Node::SharedPtr node) {
    using MoveAction = franka_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveAction>;

    auto action_client = rclcpp_action::create_client<MoveAction>(node, "/franka_gripper/move");

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Gripper Action Server not available!");
        return;
    }

    auto goal = MoveAction::Goal();
    goal.width = 0.08; // Max width
    goal.speed = 0.1;

    // --- SETUP BLOCKING WAIT ---
    // We use a promise to signal when the action is completely done
    auto result_promise = std::make_shared<std::promise<void>>();
    auto result_future = result_promise->get_future();

    auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
    
    // Callback: This triggers when the robot says "I am finished moving"
    send_goal_options.result_callback = 
        [result_promise, node](const GoalHandleMove::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node->get_logger(), "Gripper Opened Successfully.");
            } else {
                RCLCPP_WARN(node->get_logger(), "Gripper Open Failed or Cancelled.");
            }
            // Unblock the main thread
            result_promise->set_value();
        };

    // Send the goal
    RCLCPP_INFO(node->get_logger(), "ACTION: Opening Gripper (Waiting for completion)...");
    action_client->async_send_goal(goal, send_goal_options);

    // WAIT HERE until the callback above fires
    // This keeps 'action_client' alive while the robot moves
    result_future.wait();
}
void grasp_object(rclcpp::Node::SharedPtr node) {
    using GraspAction = franka_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<GraspAction>;

    auto action_client = rclcpp_action::create_client<GraspAction>(node, "/franka_gripper/grasp");

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) return;

    auto goal = GraspAction::Goal();
    goal.width = 0.04; 
    goal.speed = 0.1; 
    goal.force = 40.0;
    goal.epsilon.inner = 0.05; 
    goal.epsilon.outer = 0.05;

    // --- BLOCKING SETUP ---
    auto result_promise = std::make_shared<std::promise<void>>();
    auto result_future = result_promise->get_future();

    auto options = rclcpp_action::Client<GraspAction>::SendGoalOptions();
    options.result_callback = [result_promise, node](const GoalHandleGrasp::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node->get_logger(), "Grasp Successful.");
        } else {
            RCLCPP_WARN(node->get_logger(), "Grasp Failed.");
        }
        result_promise->set_value();
    };

    RCLCPP_INFO(node->get_logger(), "ACTION: Grasping (Waiting)...");
    action_client->async_send_goal(goal, options);
    
    // Wait for finish
    result_future.wait();
}
// --- HELPER: Execute Cartesian Path ---
bool move_cartesian(moveit::planning_interface::MoveGroupInterface& move_group, 
                    const geometry_msgs::msg::Pose& target_pose, 
                    rclcpp::Node::SharedPtr node,
                    const std::string& move_name) 
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    RCLCPP_INFO(node->get_logger(), "PLANNING: %s...", move_name.c_str());
    
    // 1cm step resolution
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.90) {
        move_group.execute(trajectory);
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "FAILED: %s (Coverage: %.1f%%)", move_name.c_str(), fraction * 100.0);
        return false;
    }
}

int old_main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_to_pose_node");

  // --- 1. SET UP SUBSCRIBER ---
  // Listen to /hmi/intent_raw
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "/hmi/intent_raw", 10, intent_callback);

  // Spin in background to catch messages
  std::thread spinner([&node]() {
    rclcpp::spin(node);
  });

  // --- Parameters ---
  node->declare_parameter<double>("x", 0.5);
  node->declare_parameter<double>("y", 0.0);
  node->declare_parameter<double>("z", 0.4);
  node->declare_parameter<double>("r", 180.0);
  node->declare_parameter<double>("p", 0.0);
  node->declare_parameter<double>("yw", 45.0);
  node->declare_parameter<bool>("grasp", true); 

  // Get Parameters
  double tx = node->get_parameter("x").as_double();
  double ty = node->get_parameter("y").as_double();
  double tz = node->get_parameter("z").as_double();
  double tr = node->get_parameter("r").as_double();
  double tp = node->get_parameter("p").as_double();
  double tyw = node->get_parameter("yw").as_double();
  bool do_grasp = node->get_parameter("grasp").as_bool();

  auto target_pose = create_pose(tx, ty, tz, tr, tp, tyw);
  auto intermediate_pose = create_pose(0.467, -0.139, 0.382, -172.5, -3.5, -53.2);
  auto release_pose = create_pose(0.598, -0.485, 0.222, -169.2, -4.1, -143.5);

  // --- Initialize MoveGroup ---
  static const std::string PLANNING_GROUP = "fr3_arm"; 
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(0.1); 
  move_group.setMaxAccelerationScalingFactor(0.1);

  // ================= EXECUTION START =================
  RCLCPP_INFO(node->get_logger(), ">>> STARTING SEQUENCE <<<");

  if(do_grasp) {
      open_gripper(node);
      std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  // 1. Move to Pick Target
  if (move_cartesian(move_group, target_pose, node, "Move to Target")) {
      
      // 2. Grasp
      if(do_grasp) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          grasp_object(node);
          std::this_thread::sleep_for(std::chrono::seconds(2));
      }

      // 3. Move to Intermediate
      if (move_cartesian(move_group, intermediate_pose, node, "Move to Intermediate")) {
          
          // 4. Move to Release Pose
          if (move_cartesian(move_group, release_pose, node, "Move to Release")) {
              
              // 5. WAIT FOR SIGNAL
              if(do_grasp) {
                  RCLCPP_WARN(node->get_logger(), ">>> ARRIVED. WAITING FOR HMI RELEASE SIGNAL... <<<");
                  RCLCPP_INFO(node->get_logger(), "Please publish to /hmi/intent_raw: {'action':'release'}");
                  
                  // Wait Loop
                  while(!release_signal_received && rclcpp::ok()) {
                      std::this_thread::sleep_for(std::chrono::milliseconds(100));
                  }
                  
                  if (!rclcpp::ok()) return 0; // Exit if Ctrl+C pressed

                  // 6. Release (Open)
                  RCLCPP_INFO(node->get_logger(), "Signal Received. Releasing object.");
                  std::this_thread::sleep_for(std::chrono::seconds(1));
                  open_gripper(node);
                  std::this_thread::sleep_for(std::chrono::seconds(2));
                  
                  // 7. RETURN TO START (Ready Pose)
                  RCLCPP_INFO(node->get_logger(), "Returning to 'Ready' (Home) position...");
                  
                  // We use named target for "Ready" because it's safer for large moves
                  move_group.setNamedTarget("ready");
                  move_group.move(); // This plans and moves
                  
                  RCLCPP_INFO(node->get_logger(), ">>> MISSION COMPLETE <<<");
              }
          }
      }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_to_pose_node");

    // Spin in background
    std::thread spinner([&node]() {
        rclcpp::spin(node);
    });

    // --- CONFIGURATION ---
    node->declare_parameter<bool>("grasp", true); 
    bool do_grasp = node->get_parameter("grasp").as_bool();

    // --- 1. DEFINE YOUR 3 TARGETS (Position + Orientation) ---
    struct DemoTarget {
        double x, y, z;       // Position (meters)
        double r, p, yw;      // Orientation (degrees)
        std::string name;     // Label for logs
    };

    // EDIT YOUR COORDINATES HERE:
    std::vector<DemoTarget> targets = {
        // {0.541, -0.086, 0.181, -167.4, 4.7, -50.9,   "Object 1: Toy"},
        {0.568, -0.060, 0.087, 174.2, 1.5, -44.1,   "Object 1: spoon"},
        
        {0.285, 0.271, 0.081, -179.2, 1.3, 4.2,   "Object 2: Fork"},
        
        {0.682, 0.169, 0.223, -175.7, -8.6, 40.0,   "Object 3: Bottle"}
    };

    // Fixed Drop/Intermediate Poses (You can change these too if needed)
    // auto intermediate_pose = create_pose(0.467, -0.139, 0.382, -172.5, -3.5, -53.2);
    // auto release_pose = create_pose(0.598, -0.485, 0.222, -169.2, -4.1, -143.5);

    // --- SETUP MOVEIT ---
    static const std::string PLANNING_GROUP = "fr3_arm"; 
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.1); 
    move_group.setMaxAccelerationScalingFactor(0.1);

    // --- START LOOP ---
    RCLCPP_INFO(node->get_logger(), ">>> STARTING 3-OBJECT DEMO SEQUENCE <<<");

    if (do_grasp) {
        open_gripper(node);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Iterate through the list
    for (const auto& target : targets) {
        
        RCLCPP_INFO(node->get_logger(), "\n--- PROCESSING: %s ---", target.name.c_str());

        // 1. Create Pose using the SPECIFIC orientation for this target
        auto target_pose = create_pose(target.x, target.y, target.z, target.r, target.p, target.yw);

        // 2. Intermediate Pose (Inherits Orientation)
        geometry_msgs::msg::Pose intermediate_pose = target_pose; 
        intermediate_pose.position.x = 0.467;
        intermediate_pose.position.y = -0.139;
        intermediate_pose.position.z = 0.402;

        // 3. Release Pose (Inherits Orientation)
        geometry_msgs::msg::Pose release_pose = target_pose;
        release_pose.position.x = 0.598;
        release_pose.position.y = -0.485;
        release_pose.position.z = 0.222;

        if (move_cartesian(move_group, intermediate_pose, node, "Move to Intermediate")) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 2. Move to Target
            if (move_cartesian(move_group, target_pose, node, "Move to Pick")) {
                
                // 3. Grasp
                if (do_grasp) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    grasp_object(node);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                // 4. Move to Intermediate
                if (move_cartesian(move_group, intermediate_pose, node, "Move to Intermediate")) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    // 5. Move to Release
                    if (move_cartesian(move_group, release_pose, node, "Move to Release")) {
                        
                        // 6. Release
                        if (do_grasp) {
                            RCLCPP_INFO(node->get_logger(), "Dropping object...");
                            std::this_thread::sleep_for(std::chrono::seconds(3));
                            open_gripper(node);
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }
                    }
                }
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to reach %s. Skipping...", target.name.c_str());
            }

        }
        // Small pause before next object
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Return to Home at the very end
    RCLCPP_INFO(node->get_logger(), "Sequence Finished. Going Home.");
    move_group.setNamedTarget("ready");
    move_group.move();

    rclcpp::shutdown();
    spinner.join();
    return 0;
}

