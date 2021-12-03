/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControlBox : public rclcpp::Node {
public:
	OffboardControlBox() : Node("offboard_control_box") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		trajectory_setpoint_publisher_ =
		 	this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		// Subscribing to lpos data
		lpos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic",10,std::bind(&OffboardControlBox::lpos_callback, this, _1));

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			
			// Check if land mode is set, if not set it in offboard mode
			if (land_ == 0) {
			
				if (offboard_setpoint_counter_ == 10) {
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

					// Arm the vehicle
					this->arm();
				}

            			// offboard_control_mode needs to be paired with trajectory_setpoint
				publish_offboard_control_mode();
				i_ = counter(i_);
				publish_trajectory_setpoint();

           		 	// stop the counter after reaching 11
				if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
				}
			}
			
			// If land mode is true, shift to land mode from offboard mode
			else {
				this->land();
			}

		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;
	void land() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	int i_ = 0;
	int land_ = 0;
	int reach_flag = 0;

	float x_sp[5] = {0,1.0,1.0,0.0,0.0};
	float y_sp[5] = {0,0.0,1.0,1.0,0.0};
	float z_sp = -1.2;

	time_t reach_time,current_time; // Noting time at each waypoint
	double duration; // Noting takeoff duration

	float x,y,z,error;

	void publish_offboard_control_mode();
	int counter(int i_);
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
	void lpos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControlBox::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControlBox::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Land the vehicle
 */
void OffboardControlBox::land() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 1.0);

	RCLCPP_INFO(this->get_logger(), "Land command send");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControlBox::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	// mode switch state machine
	if ((land_ ==  0) && i_ == 5) {
		land_ = 1;
	}
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Calculating the errors for increamenting the next waypoint counter
 */
int OffboardControlBox::counter(int i_) {
	if (i_ == 0) {
		error = z_sp - z;
	}
	else {
		error = pow(((x_sp[i_] - x)*(x_sp[i_] - x) + (y_sp[i_] - y)*(y_sp[i_] - y) + (z_sp - z)*(z_sp - z)),0.5);
	}

	if ((abs(error) < 0.04) && reach_flag == 0){
		reach_time = time(0);
		reach_flag = 1;
	}
	current_time = time(0);
	if (difftime(current_time,reach_time) > 5 && (reach_flag == 1)) {
		i_++;
		reach_flag = 0;
		reach_time = 0;
	}
	return i_;
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControlBox::publish_trajectory_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	if (i_ == 0) {
		msg.x = x;
		msg.y = y;
	}
	else {
		msg.x = x_sp[i_];
		msg.y = y_sp[i_];
	}
	msg.z = z_sp;
	msg.yaw = 0.0; // [-PI:PI]

	std::cout << "x is " << msg.x << std::endl;
	std::cout << "waypoint count" << i_ << std::endl;
	std::cout << "tracking error" << error << std::endl;

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControlBox::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControlBox::lpos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
	x = msg->x;
	y = msg->y;
	z = msg->z;
	// std::cout <<"Looks like current x, x is" << x << std::endl;
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControlBox>());

	rclcpp::shutdown();
	return 0;
}
