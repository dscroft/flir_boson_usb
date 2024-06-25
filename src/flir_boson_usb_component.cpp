#include "flir_boson_usb_component.hpp"

#include <cv_bridge/cv_bridge.h>

#include <functional>
#include <string>

namespace flir_boson_component
{

Component::Component(const rclcpp::NodeOptions &options) : Node("usb_camera", options)
{}

} // namespace flir_boson

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_component::Component)
