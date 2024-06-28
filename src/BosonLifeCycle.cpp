#include "BosonLifeCycle.hpp"

#include <cv_bridge/cv_bridge.h>

#include <functional>
#include <string>

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    boson_life_cycle::BosonLifeCycle::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring");

    const std::string devPath = 
        this->declare_parameter<std::string>("dev", "/dev/video0");
    const float rate = 
        this->declare_parameter<float>("rate", 60.f);
    std::string sensorTypeStr = 
        this->declare_parameter<std::string>("sensor_type", "Boson320");
    std::string videoModeStr = 
        this->declare_parameter<std::string>("video_mode", "RAW16");

    /* convert strings parameters*/
    static const std::map<std::string, Video_Mode> videoModeMap = {
        {"raw16", Video_Mode::RAW16},
        {"yuv", Video_Mode::YUV}
    };

    static const std::map<std::string, Sensor_Types> sensorTypeMap = {
        {"boson320", Boson320},
        {"boson640", Boson640}
    };

    std::transform(sensorTypeStr.begin(), sensorTypeStr.end(), sensorTypeStr.begin(), ::tolower);
    std::transform(videoModeStr.begin(), videoModeStr.end(), videoModeStr.begin(), ::tolower);

    RCLCPP_INFO_STREAM(this->get_logger(), "Device: " << devPath);
    RCLCPP_INFO_STREAM(this->get_logger(), "Rate: " << rate);
    RCLCPP_INFO_STREAM(this->get_logger(), "Sensor Type: " << sensorTypeStr);
    RCLCPP_INFO_STREAM(this->get_logger(), "Video Mode: " << videoModeStr);


    auto sit = sensorTypeMap.find(sensorTypeStr);
    
    if( sit == sensorTypeMap.end() ) 
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid sensor type");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    auto vit = videoModeMap.find(videoModeStr);
    if( vit == videoModeMap.end() )
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid video mode");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    /* attempt to initialise camera */
    try
    {
        this->camera_.init(devPath, sit->second);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what() );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    /* configure but immediately stop callback timer */
    this->timer_ = this->create_wall_timer(
        std::chrono::seconds(1) / rate, 
        std::bind(&BosonLifeCycle::capture_and_publish, this) );
    this->timer_->cancel();

    /* configure publish topics */
    this->camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    /*this->image_pub_8 = this->create_publisher<sensor_msgs::msg::Image>("image8", 10);
    this->image_pub_8_norm = this->create_publisher<sensor_msgs::msg::Image>("image8_norm", 10);
    this->image_pub_heatmap = this->create_publisher<sensor_msgs::msg::Image>("image_heatmap", 10);
    this->image_pub_temp = this->create_publisher<sensor_msgs::msg::Image>("image_temp", 10);*/

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    boson_life_cycle::BosonLifeCycle::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Activating");

    this->timer_->reset(); //start callback timer

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  boson_life_cycle::BosonLifeCycle::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating");

    this->timer_->cancel(); //stop callback timer
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<boson_life_cycle::BosonLifeCycle> lc_node =
    std::make_shared<boson_life_cycle::BosonLifeCycle>("BosonLifeCycle");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}