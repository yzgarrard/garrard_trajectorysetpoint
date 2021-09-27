#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <chrono>
#include <cstdio>
#include <math.h>

using namespace std::chrono_literals;

class TrajectorySetpointGenerator : public rclcpp::Node {
public:
  TrajectorySetpointGenerator() : Node("garrard_trajectorysetpoint") {
    garrard_trajectorysetpoint_publisher_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("garrard_trajectorysetpoint_pub", 1);
  
    auto update_trajectory_sp_timer_callback = [this]() -> void {
      px4_msgs::msg::TrajectorySetpoint sp;
      double time_seconds = ((std::chrono::steady_clock::now() - initial_time).count())/1000000000.0;
      float traj_pos_z_sp = (float)(sin(time_seconds/5) - 2);
      // float traj_pos_z_sp = -2;
      sp.z = traj_pos_z_sp;
      RCLCPP_INFO(get_logger(), 
            "Updating position trajectory setpoint to x:\t%4.4f\ty:\t%4.4f\tz:\t%4.4f", 
            sp.x, sp.y, sp.z);
      garrard_trajectorysetpoint_publisher_->publish(sp);
    };
    sp_update_timer_ = this->create_wall_timer(100ms, update_trajectory_sp_timer_callback);
  }
private:
  rclcpp::TimerBase::SharedPtr sp_update_timer_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr garrard_trajectorysetpoint_publisher_;

  std::chrono::time_point<std::chrono::steady_clock> initial_time = std::chrono::steady_clock::now();
};

int main(int argc, char* argv[]) {
	std::cout << "Starting garrard_trajectorysetpoint node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectorySetpointGenerator>());

	rclcpp::shutdown();
	return 0;
}
