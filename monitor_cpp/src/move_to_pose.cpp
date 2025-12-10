#include <memory>
#include <thread>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "franka_msgs/action/grasp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class GraspObjectNode : public rclcpp::Node {
public:
	using Grasp = franka_msgs::action::Grasp;
	
	GraspObjectNode() : Node("grasp_object_node") {

		// Subscription to receive target object pose
		sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/target_object_pose", 10,
			std::bind(&GraspObjectNode::object_callback, this, std::placeholders::_1)
		);
	}

	void run_pipeline(const geometry_msgs::msg::Pose &target_pose) {

		// Initialize MoveGroup
		static const std::string PLANNING_GROUP = "fr3_arm"; 
		moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
		
		// Safety: Slow down!
		move_group.setMaxVelocityScalingFactor(0.1); 
		move_group.setMaxAccelerationScalingFactor(0.1);
		
		/*// 1. Define Target Pose
		geometry_msgs::msg::Pose target_pose;
		target_pose.position.x = target_x;
		target_pose.position.y = target_y;
		target_pose.position.z = target_z;
		
		tf2::Quaternion q;
		q.setRPY(target_r * M_PI/180.0, target_p * M_PI/180.0, target_yw * M_PI/180.0);
		target_pose.orientation.x = q.x();
		target_pose.orientation.y = q.y();
		target_pose.orientation.z = q.z();
		target_pose.orientation.w = q.w();*/
		
		// 2. Cartesian Path Planning
		// Instead of finding *any* path, we force a straight line.
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(target_pose);
		
		moveit_msgs::msg::RobotTrajectory trajectory;
		// eef_step: Resolution (1cm steps)
		// jump_threshold: 0.0 means disable check (careful near singularities)
		const double jump_threshold = 0.0; 
		const double eef_step = 0.01;
		
		RCLCPP_INFO(this->get_logger(), "Computing Cartesian Path...");
		
		double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

		RCLCPP_INFO(this->get_logger(), "Path computation: %.2f%% achieved", fraction * 100.0);

		// 3. Execute if feasible
		// We allow execution if > 90% of the path is found
		if (fraction > 0.9) {
		RCLCPP_INFO(this->get_logger(), "Executing Cartesian Move...");
		move_group.execute(trajectory);
		} else {
		RCLCPP_ERROR(this->get_logger(), "Path incomplete! Robot cannot reach target in a straight line.");
		return;
		}

		// 4. Grasp object
		auto grasp_client = rclcpp_action::create_client<Grasp>(shared_from_this(), "/NS_1/franka_gripper/grasp");

		if (!grasp_client->wait_for_action_server(500ms)) {
		RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
		return;
		}

		auto goal_msg = Grasp::Goal();
		goal_msg.width = 0.00;
		goal_msg.speed = 0.03;
		goal_msg.force = 50;
		goal_msg.epsilon.inner = 0.01;
		goal_msg.epsilon.outer = 0.01;

		RCLCPP_INFO(this->get_logger(), "Sending grasp goal...");

		auto goal_handle_future = grasp_client->async_send_goal(goal_msg);
		auto goal_handle = goal_handle_future.get();
		if (!goal_handle) {
		RCLCPP_ERROR(this->get_logger(), "Failed to send grasp goal");
		return;
		}

		auto result_future = grasp_client->async_get_result(goal_handle);
		if (rclcpp::spin_until_future_complete(shared_from_this(), result_future, 2s)
			!= rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Grasp action timeout");
		return;
		}

		auto result = result_future.get();
		if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(this->get_logger(), "Grasp action failed");
		return;
		}

		RCLCPP_INFO(this->get_logger(), "Object grasped, lifting...");

		// 5. Lift object (10 cm)
		geometry_msgs::msg::Pose lift_pose = target_pose;
		lift_pose.position.z += 0.10;

		std::vector<geometry_msgs::msg::Pose> waypoints2;
		waypoints2.push_back(lift_pose);

		moveit_msgs::msg::RobotTrajectory trajectory2;
		double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

		if (fraction2 > 0.2) {
		move_group.execute(trajectory2);
		}

		RCLCPP_INFO(this->get_logger(), "Lift executed.");


		// 6. Move to release pose
		
		std::vector<geometry_msgs::msg::Pose> waypoints2;
		waypoints2.push_back(release_pose);

		moveit_msgs::msg::RobotTrajectory trajectory2;
		double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

		if (fraction2 > 0.2) {
		move_group.execute(trajectory2);
		}

		RCLCPP_INFO(this->get_logger(), "Moved to release pose.");
	}

private:
	void object_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
		run_pipeline(msg->pose);
	}

	// TODO: set release Pose
	geometry_msgs::msg::Pose release_pose;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GraspObjectNode>();

	// Spin in background
	std::thread spinner([&node]() {
		rclcpp::spin(node);
	});

	// Declare Parameters with default values (so it doesn't crash if you forget one)
	node->declare_parameter<double>("x", 0.5);
	node->declare_parameter<double>("y", 0.0);
	node->declare_parameter<double>("z", 0.4);
	node->declare_parameter<double>("r", 180.0);
	node->declare_parameter<double>("p", 0.0);
	node->declare_parameter<double>("yw", 45.0); // 'yw' because 'y' is taken by coordinate y

	// Get Parameters
	double target_x = node->get_parameter("x").as_double();
	double target_y = node->get_parameter("y").as_double();
	double target_z = node->get_parameter("z").as_double();
	double target_r = node->get_parameter("r").as_double();
	double target_p = node->get_parameter("p").as_double();
	double target_yw = node->get_parameter("yw").as_double();

	RCLCPP_INFO(node->get_logger(), "Target received: [%.2f, %.2f, %.2f]", target_x, target_y, target_z);

	// Initialize MoveGroup
	static const std::string PLANNING_GROUP = "fr3_arm"; 
	moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

	// Safety: Slow down!
	move_group.setMaxVelocityScalingFactor(0.1); 
	move_group.setMaxAccelerationScalingFactor(0.1);

	// 1. Define Target Pose
	geometry_msgs::msg::Pose target_pose;
	target_pose.position.x = target_x;
	target_pose.position.y = target_y;
	target_pose.position.z = target_z;

	tf2::Quaternion q;
	q.setRPY(target_r * M_PI/180.0, target_p * M_PI/180.0, target_yw * M_PI/180.0);
	target_pose.orientation.x = q.x();
	target_pose.orientation.y = q.y();
	target_pose.orientation.z = q.z();
	target_pose.orientation.w = q.w();

	node->run_pipeline(target_pose);

	rclcpp::shutdown();
	spinner.join();
	return 0;
}
