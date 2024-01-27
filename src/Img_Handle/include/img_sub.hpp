#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include "opencv4/opencv2/opencv_modules.hpp"
#include "inference.h"
#include "my_msgss/msg/yolopoint.hpp"
#include "my_msgss/msg/yolopoints.hpp"
#include <vector>

using namespace std;
using namespace cv;

class Img_Sub : public::rclcpp::Node
{
public:
    Img_Sub();
    
    Mat sub_img;

    int i;

    void save_img();

    void img_callback(const sensor_msgs::msg::Image msg);

    void yolo_init();

    void yolo_robot_identify();

    void yolo_armor_identify(my_msgss::msg::Yolopoint &robot_box, cv::Rect &box);

    void draw_img();

    void test();

    my_msgss::msg::Yolopoints robot_boxes;

    vector<Detection> robot_output;

    Inference inf_armor;

    Inference inf_robot;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<my_msgss::msg::Yolopoint>::SharedPtr yolopoint_pub_;
    rclcpp::Publisher<my_msgss::msg::Yolopoints>::SharedPtr yolopoints_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr far_qimage_pub_;
};