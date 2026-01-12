#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <atomic>
#include <mutex>
#include <iostream>
#include <fstream> // Required for file reading
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "std_msgs/msg/string.hpp"

// geometry messages
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// --- DATA STRUCTURES ---
struct DetectedObject {
    double x, y, z;
    std::string label;
    bool valid = false;
};

struct MissionItem {
    std::string object_name;
    int trigger_time_sec;
};

// --- GLOBAL MEMORY ---
std::map<std::string, DetectedObject> object_memory;
std::mutex memory_mutex;
std::atomic<bool> release_signal_received(false);

// --- ON-DEMAND MODE GLOBALS ---
std::atomic<bool> take_signal_received(false);
std::string requested_object = "";
std::mutex object_request_mutex;
std::set<std::string> valid_objects = {"bottle", "fork", "spoon"};
std::set<std::string> taken_objects;  // Track which objects have been taken (max 3)

// --- GLOBAL TF BUFFER (So callback can use it) ---
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

// --- HELPER: Load Mission from JSON File ---
std::vector<MissionItem> load_mission_from_file(const std::string& filename) {
    std::vector<MissionItem> mission;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "[ERROR] Could not open mission file: " << filename << std::endl;
        return mission;
    }

    try {
        json data = json::parse(file);
        
        // Check project name just for logs
        std::string project = data.value("project", "Unknown");
        std::cout << "[SYSTEM] Loading Project: " << project << std::endl;

        // Parse "sequence" array
        if (data.contains("sequence") && data["sequence"].is_array()) {
            for (const auto& item : data["sequence"]) {
                MissionItem task;
                task.object_name = item["tool"];
                task.trigger_time_sec = item["startTime"];
                mission.push_back(task);
            }
        }
    } catch (const json::parse_error& e) {
        std::cerr << "[ERROR] JSON Syntax Error in mission file: " << e.what() << std::endl;
    }
    return mission;
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

// ... [Keep your open_gripper and grasp_object functions exactly as they were] ...
 // void open_gripper(rclcpp::Node::SharedPtr node) {
    // using MoveAction = franka_msgs::action::Move;
    // auto action_client = rclcpp_action::create_client<MoveAction>(node, "/franka_gripper/move");
    // if (!action_client->wait_for_action_server(std::chrono::seconds(2))) return;

    // auto goal = MoveAction::Goal();
    // goal.width = 0.08;
    // goal.speed = 0.1
    // action_client->async_send_goal(goal);

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
    send_goal_options.result_callback = [result_promise, node](const GoalHandleMove::WrappedResult & result) {

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

// --- HELPER: Execute Cartesian Path (Keep as is) ---
bool move_cartesian(moveit::planning_interface::MoveGroupInterface& move_group, 
                    const geometry_msgs::msg::Pose& target_pose, 
                    rclcpp::Node::SharedPtr node,
                    const std::string& move_name) 
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    RCLCPP_INFO(node->get_logger(), "PLANNING: %s...", move_name.c_str());
    
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.90) {
        move_group.execute(trajectory);
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "FAILED: %s (Coverage: %.1f%%)", move_name.c_str(), fraction * 100.0);
        return false;
    }
}

// --- PERCEPTION CALLBACK ---
void vision_callback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto data = json::parse(msg->data);
        std::lock_guard<std::mutex> lock(memory_mutex);

        if (data.is_array()) {
            for (const auto& item : data) {
                std::string label = item["label"];
                
                // 1. Get RAW Camera Coordinates
                double raw_x = item["x"];
                double raw_y = item["y"];
                double raw_z = item["z"];

                // 2. Prepare Transform Input
                geometry_msgs::msg::PointStamped point_camera;
                point_camera.header.frame_id = "camera_link"; // Defined in launch file 
                point_camera.point.x = raw_x;
                point_camera.point.y = raw_y;
                point_camera.point.z = raw_z;

                geometry_msgs::msg::PointStamped point_robot;

                // 3. Transform to Robot Frame ("fr3_link0")
                try {
                    // Check if transform is available
                    if (tf_buffer->canTransform("fr3_link0", "camera_link", tf2::TimePointZero)) {
                        
                        tf_buffer->transform(point_camera, point_robot, "fr3_link0");
                        
                        double final_x = point_robot.point.x;
                        double final_y = point_robot.point.y;
                        double final_z = point_robot.point.z;

                        // 4. Store ROBOT FRAME coordinates in memory
                        if (object_memory.find(label) == object_memory.end()) {
                            object_memory[label] = {final_x, final_y, final_z, label, true};
                            std::cout << "[VISION] New object stored (Transformed): " << label << std::endl;
                        } else {
                            DetectedObject& old = object_memory[label];
                            double dist = std::sqrt(std::pow(final_x - old.x, 2) + std::pow(final_y - old.y, 2) + std::pow(final_z - old.z, 2));
                            if (dist > 0.02) {
                                old.x = final_x; old.y = final_y; old.z = final_z;
                                std::cout << "[VISION] Memory updated for: " << label << std::endl;
                            }
                        }
                    } 
                } catch (tf2::TransformException &ex) {
                    // Startups often fail first few seconds, usually fine to ignore or log debug
                }
            }
        }
    } catch (const std::exception& e) { }
}

// --- HELPER: Extract object from message ---
std::string extract_object_from_message(const std::string& msg_data) {
    // Convert to lowercase for comparison
    std::string data = msg_data;
    std::transform(data.begin(), data.end(), data.begin(), ::tolower);

    // Check for valid objects
    for (const auto& obj : valid_objects) {
        if (data.find(obj) != std::string::npos) {
            return obj;
        }
    }
    return "";
}

// --- HMI CALLBACK ---
void intent_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string data = msg->data;
    std::transform(data.begin(), data.end(), data.begin(), ::tolower);

    // Check for release command
    if (data.find("release") != std::string::npos) {
        release_signal_received = true;
        std::cout << "\n[HMI] RELEASE SIGNAL RECEIVED!\n" << std::endl;
        return;
    }

    // Check for take command (on-demand mode)
    if (data.find("take") != std::string::npos) {
        std::string obj = extract_object_from_message(data);
        if (!obj.empty()) {
            std::lock_guard<std::mutex> lock(object_request_mutex);
            // Check if object was already taken
            if (taken_objects.find(obj) != taken_objects.end()) {
                std::cout << "\n[HMI] ERROR: Object '" << obj << "' was already taken!\n" << std::endl;
                return;
            }
            requested_object = obj;
            take_signal_received = true;
            std::cout << "\n[HMI] TAKE SIGNAL RECEIVED: " << obj << "\n" << std::endl;
        } else {
            std::cout << "\n[HMI] ERROR: Invalid object in take command. Valid: bottle, fork, spoon\n" << std::endl;
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_to_pose");

    // --- INITIALIZE TF LISTENER ---
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // --- PARAMETERS ---
    // Default path to your JSON file (Update this path!)
    node->declare_parameter<std::string>("mission_file", "/home/userlab/cjimenez/franka_ros2_language_based_manipulator/shared/sequence.json");
    std::string mission_file = node->get_parameter("mission_file").as_string();

    // Mode parameter: "sequence" or "on-demand"
    node->declare_parameter<std::string>("mode", "sequence");
    std::string mode = node->get_parameter("mode").as_string();

    RCLCPP_INFO(node->get_logger(), ">>> MODE: %s <<<", mode.c_str());

    // Validate mode
    if (mode != "sequence" && mode != "on-demand") {
        RCLCPP_ERROR(node->get_logger(), "Invalid mode '%s'. Use 'sequence' or 'on-demand'. Defaulting to 'sequence'.", mode.c_str());
        mode = "sequence";
    }

    // Hand parameter: "right" or "left"
    node->declare_parameter<std::string>("hand", "right");
    std::string hand = node->get_parameter("hand").as_string();
    RCLCPP_INFO(node->get_logger(), ">>> HAND: %s <<<", hand.c_str());

    // Validate hand side
    if (hand != "right" && hand != "left") {
        RCLCPP_ERROR(node->get_logger(), "Invalid hand side '%s'. Use 'right' or 'left'. Defaulting to 'right'.", hand.c_str());
        hand = "right";
    }

    // --- SUBSCRIBERS ---
    auto vision_sub = node->create_subscription<std_msgs::msg::String>(
        "/vision/detected_objects", 10, vision_callback);
    
    auto hmi_sub = node->create_subscription<std_msgs::msg::String>(
        "/hmi/intent_raw", 10, intent_callback);

    // --- MOVEIT SETUP ---
    static const std::string PLANNING_GROUP = "fr3_arm"; 
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setEndEffectorLink("fr3_hand_tcp");
    move_group.setMaxVelocityScalingFactor(0.15); 
    move_group.setMaxAccelerationScalingFactor(0.15);

    RCLCPP_INFO(node->get_logger(), "End-effector link: %s", move_group.getEndEffectorLink().c_str());

    // Fixed Poses (shared by both modes)
    auto intermediate_pose = create_pose(0.45, 0.00, 0.35, 179.5, -5.8, -2.0);
    auto release_pose = hand=="right" ? create_pose(0.598, -0.490, 0.22, 179.9, -3.9, -92.2) :
                                        create_pose(0.219, -0.455, 0.22, 179.9, -3.9, -92.2); // Left handed


    // ========================================================
    // SEQUENCE MODE
    // ========================================================
    if (mode == "sequence") {
        // --- LOAD MISSION ---
        std::vector<MissionItem> mission = load_mission_from_file(mission_file);

        if (mission.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Mission is empty or file not found! Exiting.");
            rclcpp::shutdown();
            return 0;
        }

        double travel_time_offset = 5.0;
        auto start_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(node->get_logger(), ">>> PROJECT STARTED (SEQUENCE MODE): %lu Tasks Loaded <<<", mission.size());

        // --- MAIN EXECUTION LOOP (SEQUENCE) ---
        for (const auto& task : mission) {

            RCLCPP_INFO(node->get_logger(), "\n--- NEXT TASK: Pick %s at T=%ds ---", task.object_name.c_str(), task.trigger_time_sec);

            // 1. TIME WAIT LOOP
            while (rclcpp::ok()) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                double time_until_trigger = task.trigger_time_sec - elapsed - travel_time_offset;

                if (time_until_trigger <= 0) break; // Time to go!

                if ((int)elapsed % 5 == 0) {
                     RCLCPP_INFO(node->get_logger(), "Waiting... T-minus %.1f sec", time_until_trigger);
                     std::this_thread::sleep_for(std::chrono::milliseconds(1005));
                } else {
                     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            // 2. MEMORY LOOKUP (WAIT if not found)
            DetectedObject target_obj;
            bool found = false;

            // Try looking up the object for 5 seconds if not immediately found
            for(int i=0; i<50; ++i) {
                {
                    std::lock_guard<std::mutex> lock(memory_mutex);
                    if (object_memory.find(task.object_name) != object_memory.end()) {
                        target_obj = object_memory[task.object_name];
                        found = true;
                        break;
                    }
                }
                if(!found) {
                     if(i==0) RCLCPP_WARN(node->get_logger(), "Scanning for '%s'...", task.object_name.c_str());
                     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            if (!found) {
                RCLCPP_ERROR(node->get_logger(), "SKIPPING: Object '%s' never found in vision memory!", task.object_name.c_str());
                continue;
            }

            // Execution Sequence
            // A. Move to Intermediate
            move_cartesian(move_group, intermediate_pose, node, "Intermediate");

            // Hardcoded orientation (Vertical Grasp)
            auto pick_pose = create_pose(
                target_obj.x, target_obj.y, target_obj.z,
                180, 0.0, 90
            );
            pick_pose.position.x -= 0.02; // Adjust for practical purposes
            pick_pose.position.y -= 0.075; // Adjust for practical purposes
            if (pick_pose.position.z < 0.056){
                pick_pose.position.z = -0.024; // Safety floor
            }else{
                pick_pose.position.z -= 0.08; // Adjust for table height
            }

            // 3. EXECUTE PICK (Uses coordinates from memory)
            RCLCPP_INFO(
                node->get_logger(),
                "Moving to %s: (%.3f, %.3f, %.3f, %.2f, %.2f, %.2f)",
                task.object_name.c_str(),
                pick_pose.position.x, pick_pose.position.y, pick_pose.position.z,
                pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z
            );

            // B. Hover & Pick
            auto pre_grasp = pick_pose;
            pre_grasp.position.z += 0.103; // Hover 10.3cm above

            std::this_thread::sleep_for(std::chrono::seconds(10));

            if (move_cartesian(move_group, pre_grasp, node, "Pre-Grasp")) {
                if (move_cartesian(move_group, pick_pose, node, "Pick")) {

                    grasp_object(node);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    move_cartesian(move_group, pre_grasp, node, "Retract");
                    move_cartesian(move_group, intermediate_pose, node, "Intermediate");
                    move_cartesian(move_group, release_pose, node, "Release Zone");

                    // C. Wait for HMI
                    RCLCPP_WARN(node->get_logger(), "WAITING FOR RELEASE...");
                    release_signal_received = false;
                    while(!release_signal_received && rclcpp::ok()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    open_gripper(node);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }
    }
    // ========================================================
    // ON-DEMAND MODE
    // ========================================================
    else if (mode == "on-demand") {
        RCLCPP_INFO(node->get_logger(), ">>> PROJECT STARTED (ON-DEMAND MODE) <<<");
        RCLCPP_INFO(node->get_logger(), "Valid objects: bottle, fork, spoon (max 3 takes)");
        RCLCPP_INFO(node->get_logger(), "Commands: 'take <object>' to pick, 'release' to drop");

        int successful_takes = 0;
        const int MAX_TAKES = 3;

        // Main on-demand loop
        while (rclcpp::ok() && successful_takes < MAX_TAKES) {
            // Wait for take command
            RCLCPP_INFO(node->get_logger(), "\n>>> WAITING FOR TAKE COMMAND... (Successful takes: %d/%d) <<<",
                        successful_takes, MAX_TAKES);

            take_signal_received = false;
            while (!take_signal_received && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!rclcpp::ok()) break;

            // Get the requested object
            std::string current_object;
            {
                std::lock_guard<std::mutex> lock(object_request_mutex);
                current_object = requested_object;
                requested_object = "";
            }

            RCLCPP_INFO(node->get_logger(), "\n--- TAKE REQUESTED: %s ---", current_object.c_str());

            // Look up object in memory
            DetectedObject target_obj;
            bool found = false;

            // Try looking up the object for 5 seconds if not immediately found
            for (int i = 0; i < 50; ++i) {
                {
                    std::lock_guard<std::mutex> lock(memory_mutex);
                    if (object_memory.find(current_object) != object_memory.end()) {
                        target_obj = object_memory[current_object];
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    if (i == 0) RCLCPP_WARN(node->get_logger(), "Scanning for '%s'...", current_object.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            if (!found) {
                RCLCPP_ERROR(node->get_logger(), "Object '%s' not found in vision memory. Waiting for new command...",
                             current_object.c_str());
                continue;  // Wait for another take command
            }

            // Execute pick sequence
            RCLCPP_INFO(node->get_logger(), "Object found at (%.3f, %.3f, %.3f)",
                        target_obj.x, target_obj.y, target_obj.z);

            // A. Move to Intermediate
            move_cartesian(move_group, intermediate_pose, node, "Intermediate");

            // Hardcoded orientation (Vertical Grasp)
            auto pick_pose = create_pose(
                target_obj.x, target_obj.y, target_obj.z,
                180, 0.0, 90
            );
            pick_pose.position.x -= 0.02;
            pick_pose.position.y -= 0.075;
            if (pick_pose.position.z < 0.056) {
                pick_pose.position.z = -0.024;
            } else {
                pick_pose.position.z -= 0.08;
            }

            RCLCPP_INFO(
                node->get_logger(),
                "Moving to %s: (%.3f, %.3f, %.3f)",
                current_object.c_str(),
                pick_pose.position.x, pick_pose.position.y, pick_pose.position.z
            );

            // B. Hover & Pick
            auto pre_grasp = pick_pose;
            pre_grasp.position.z += 0.103;

            std::this_thread::sleep_for(std::chrono::seconds(10));

            bool pick_successful = false;
            if (move_cartesian(move_group, pre_grasp, node, "Pre-Grasp")) {
                if (move_cartesian(move_group, pick_pose, node, "Pick")) {

                    grasp_object(node);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    move_cartesian(move_group, pre_grasp, node, "Retract");
                    move_cartesian(move_group, intermediate_pose, node, "Intermediate");
                    move_cartesian(move_group, release_pose, node, "Release Zone");

                    pick_successful = true;

                    // C. Wait for HMI release
                    RCLCPP_WARN(node->get_logger(), "WAITING FOR RELEASE...");
                    release_signal_received = false;
                    while (!release_signal_received && rclcpp::ok()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    open_gripper(node);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    // Mark object as taken
                    {
                        std::lock_guard<std::mutex> lock(object_request_mutex);
                        taken_objects.insert(current_object);
                    }
                    successful_takes++;
                    RCLCPP_INFO(node->get_logger(), "Successfully completed take for '%s'. Total: %d/%d",
                                current_object.c_str(), successful_takes, MAX_TAKES);
                }
            }

            if (!pick_successful) {
                RCLCPP_ERROR(node->get_logger(), "Pick failed for '%s'. Waiting for new command...",
                             current_object.c_str());
                // Don't increment successful_takes, wait for another command
            }
        }

        if (successful_takes >= MAX_TAKES) {
            RCLCPP_INFO(node->get_logger(), "All %d objects have been taken. On-demand session complete.", MAX_TAKES);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Project Complete. Going Home.");
    move_group.setNamedTarget("ready");
    move_group.move();

    rclcpp::shutdown();
    spinner.join();
    return 0;
}