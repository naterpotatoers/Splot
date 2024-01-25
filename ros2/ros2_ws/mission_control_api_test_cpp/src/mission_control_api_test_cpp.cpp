#include<chrono>
#include<functional>
#include<memory>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MissionControlApiTestCpp : public rclcpp::Node
{
    public:
        MissionControlApiTestCpp(): Node("mission_control_api_test_cpp")
        {   
            // print message
            RCLCPP_INFO(this->get_logger(), "Hello World!");
        }
    private:
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlApiTestCpp>());
    rclcpp::shutdown();
    return 0;
}