#ifndef BOUNDING_HPP

#include <algorithm>
#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


#include "BosonCamera.hpp"

namespace flir_boson_component
{

class Component : public rclcpp::Node
{
public:
    explicit Component(const rclcpp::NodeOptions &options);

private:
    BosonCamera camera;

    rclcpp::TimerBase::SharedPtr timer_;

    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_8;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_8_norm;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_heatmap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_temp;
    
    virtual void onInit()
    {
        const std::string devPath = 
            this->declare_parameter<std::string>("dev", "/dev/video0");
        const float rate = 
            this->declare_parameter<float>("rate", 60.f);
        std::string sensorTypeStr = 
            this->declare_parameter<std::string>("sensor_type", "Boson320");
        const std::string videoModeStr = 
            this->declare_parameter<std::string>("video_mode", "RAW16");

        /* convert strings parameters*/
        static const std::map<std::string, Video_Mode> videoModeMap = {
            {"raw16", RAW16},
            {"yuv", YUV}
        };

        static const std::map<std::string, Sensor_Types> sensorTypeMap = {
            {"boson320", Boson320},
            {"boson640", Boson640}
        };

        std::for_each( sensorTypeStr.begin(), sensorTypeStr.end(), ::tolower );
        std::for_each( videoModeStr.begin(), videoModeStr.end(), ::tolower );

        auto sit = sensorTypeMap.find(sensorTypeStr);
        auto vit = videoModeMap.find(videoModeStr);
        if( sit != sensorTypeMap.end() && vit != videoModeMap.end() )
        {
            this->camera.init(devPath, sit->second, vit->second);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - Invalid sensor_type value provided. Exiting.");
        }
        /*capture_timer =
            nh.createTimer(ros::Duration(1.0 / frame_rate),
                           boost::bind(&BosonCamera::captureAndPublish, this, _1));
        }*/

       this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1) / rate, 
            std::bind(&Component::captureAndPublish, this) );
    
        this->camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        this->image_pub_8 = this->create_publisher<sensor_msgs::msg::Image>("image8", 10);
        this->image_pub_8_norm = this->create_publisher<sensor_msgs::msg::Image>("image8_norm", 10);
        this->image_pub_heatmap = this->create_publisher<sensor_msgs::msg::Image>("image_heatmap", 10);
        this->image_pub_temp = this->create_publisher<sensor_msgs::msg::Image>("image_temp", 10);
    }

    void publish_raw16_( const std_msgs::msg::Header& header )
    {
        cv_bridge::CvImage message( header, "16UC1", this->camera.get_linear() );
        sensor_msgs::msg::Image outMsg;
        message.toImageMsg( outMsg );

        this->image_pub->publish( outMsg );
    }   

    /**
     * @brief Publishes the thermal image in YUV format
     */
    void publish_yuv_( const std_msgs::msg::Header& header )
    {
        cv_bridge::CvImage message( header, "mono8", this->camera.get_rgb() );
        sensor_msgs::msg::Image outMsg;
        message.toImageMsg( outMsg );

        this->image_pub->publish( outMsg );
    }

    void captureAndPublish()
    {
        camera.read();

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";

        if( this->camera.get_video_mode() == Video_Mode::RAW16 )
            publish_raw16_( header );
        else
            publish_yuv_( header );
        
        //cv::imshow("Thermal", camera.get_linear());
        //cv::imshow("Thermal RGB", camera.get_rgb());
    }

    

    /*rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubDebug_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    EasyDNN dnn_;
    cv::Mat image_;
    std::vector<std::string> labels_;

    void image_callback(const sensor_msgs::msg::Image &msg);*/
};

} // namespace flir_boson

#endif // BOUNDING_HPP