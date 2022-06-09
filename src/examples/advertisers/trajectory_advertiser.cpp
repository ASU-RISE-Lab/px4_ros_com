/****************************************************************************
 *
 * Copyright 2018 PX4 Development Team. All rights reserved.
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
 * @brief Debug Vect uORB topic adverstiser example
 * @file trajectory_advertiser.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;

class TrajectoryAdvertiser : public rclcpp::Node
{
public:
    TrajectoryAdvertiser() : Node("trajectory_advertiser") {
#ifdef ROS_DEFAULT_API
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
#else
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
#endif
        auto timer_callback =
        [this]()->void {
            auto trajectory_sp = px4_msgs::msg::TrajectorySetpoint();
            trajectory_sp.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            trajectory_sp.x = 1;
            trajectory_sp.y = -1;
            trajectory_sp.z = -0.8;
            RCLCPP_INFO(this->get_logger(), "\033[97m Publishing trajectory as: %llu x: %f y: %f z: %f \033[0m",
                                trajectory_sp.timestamp, trajectory_sp.x, trajectory_sp.y, trajectory_sp.z);
            this->publisher_->publish(trajectory_sp);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting trajectory advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryAdvertiser>());

    rclcpp::shutdown();
    return 0;
}
