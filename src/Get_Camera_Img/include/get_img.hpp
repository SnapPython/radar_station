#include "CameraApi.h"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "opencv4/opencv2/opencv_modules.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "MVCamera.hpp"
#include "Video_Saver.hpp"

using namespace std;
using namespace cv;
#define FARCAM 0
#define CLOSECAM 1


class GetImg : public::rclcpp::Node
{
public:
    GetImg();

private:
    std::string mat_type23encoding(int mat_type);

    void timer_callback();

    void camera_init();

    void get_exp(const std_msgs::msg::Int16 &exp_time);

    void get_is_large(const std_msgs::msg::Bool &is_large_resolution);

    void get_is_rcd(const std_msgs::msg::Bool &is_rcd);

    string num2str(double i);

    bool take_and_send_image();

    rclcpp::TimerBase::SharedPtr timer_;

    int false_idx=0;
    int deviceID=0;
    int count=0;
    bool flag=true;
    int init_suc;
    char cam_name[32]={'0','0','0','0','0','0','0'};
    char far_or_close;
    // shared image message
    Mat rawImg;
    sensor_msgs::msg::Image msg;
    int image_width_, image_height_, framerate_, exposure_=24000, brightness_, contrast_, saturation_, sharpness_, focus_,
    white_balance_, gain_,fps_mode=1;
    bool large_resolution_=false,is_record_=false,autofocus_, autoexposure_=0, auto_white_balance_=0;
    string rcd_path_;
    VideoSaver saver;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cfg_exp_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_large_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_rcd_sub;
};