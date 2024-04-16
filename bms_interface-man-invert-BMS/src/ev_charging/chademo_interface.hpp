#include "charging_mode_base.hpp"

#include <can_msgs/msg/frame.hpp>
#include <vector>

namespace HKPC{

class CHAdeMO : public ChargingModeBase {
public:
    explicit CHAdeMO(rclcpp::Node &node) : ChargingModeBase(node){}
private:
    OutputData run(ChargingState const &state, 
        charging_robot_system_msgs::msg::InverterState const &inv_msg) override;
    OutputData output_data_;
};


}//