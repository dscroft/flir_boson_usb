#ifndef BOSON_LIFE_CYCLE_HPP
#define BOSON_LIFE_CYCLE_HPP

#include <algorithm>
#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <BosonCamera.hpp>

namespace boson_life_cycle
{

class BosonLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit BosonLifeCycle(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}

private:
    BosonCamera camera_;

    rclcpp::TimerBase::SharedPtr timer_;
   
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    /*rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_8;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_8_norm;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_heatmap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_temp;*/
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &);

    void publish_raw16_( const std_msgs::msg::Header& header )
    {
        RCLCPP_INFO(this->get_logger(), "Publishing raw16 image");

        /*cv_bridge::CvImage message( header, "16UC1", this->camera_.get_linear() );
        sensor_msgs::msg::Image outMsg;
        message.toImageMsg( outMsg );

        this->image_pub->publish( outMsg );*/

        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage( header, "mono8", this->camera_.get_linear() ).toImageMsg();

        this->image_pub->publish( *msg );
    }   

    /**
     * @brief Publishes the thermal image in YUV format
     */
    void publish_yuv_( const std_msgs::msg::Header& header )
    {
        cv_bridge::CvImage message( header, "mono8", this->camera_.get_rgb() );
        sensor_msgs::msg::Image outMsg;
        message.toImageMsg( outMsg );

        this->image_pub->publish( outMsg );
    }

    void capture_and_publish()
    {
        RCLCPP_INFO(this->get_logger(), "Capturing frame");
        this->camera_.read();

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";

        RCLCPP_INFO_STREAM(this->get_logger(), this->camera_.get_video_mode());

        if( this->camera_.get_video_mode() == Video_Mode::RAW16 )
            publish_raw16_( header );
        /*else
            publish_yuv_( header );*/
        
        //cv::imshow("Thermal", camera.get_linear());
        //cv::imshow("Thermal RGB", camera.get_rgb());
    }
};

} // namespace flir_boson

#endif // BOSON_LIFE_CYCLE_HPP