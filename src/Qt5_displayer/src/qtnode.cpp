#include "qtnode.h"

qtNode::qtNode()
{
    this->start();
}

qtNode::~qtNode()
{
    
}

void qtNode::farImageCallback(const sensor_msgs::msg::Image msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        if(!cv_ptr->image.empty())
        {
            Mat far_image = cv_ptr->image;
            cv::resize(far_image,far_image,cv::Size(FAR_IMAGE_WIDTH,FAR_IMAGE_HEIGHT));
            far_qimage = QImage((const unsigned char*)(far_image.data),far_image.cols,far_image.rows,QImage::Format_RGB888);
        }
        //cout << "激活far函数" << endl;
        Q_EMIT updateFarImage();
    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(qnode->get_logger(),"cv_bridge exception: %s",e.what());
        return;
    }
}

void qtNode::depthImageCallback(const sensor_msgs::msg::Image msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
        if(!cv_ptr->image.empty())
        {
            Mat depth_image = cv_ptr->image;
            cv::resize(depth_image,depth_image,cv::Size(DEPTH_IMAGE_WIDTH,DEPTH_IMAGE_HEIGHT));
            Mat depth_image_8u;
            depth_image.convertTo(depth_image_8u,CV_8UC1,255.0/10.0);
            Mat depth_bgrimage;
            applyColorMap(depth_image_8u, depth_bgrimage, cv::COLORMAP_JET);
            depth_qimage = QImage((const unsigned char*)(depth_bgrimage.data),depth_bgrimage.cols,depth_bgrimage.rows,QImage::Format_BGR888);
            //depth_qimage = QImage((const unsigned char*)(depth_image.data),depth_image.cols,depth_image.rows,QImage::Format_Grayscale8);
        }
        //cout << "激活depth函数" << endl;
        Q_EMIT updateDepthImage();
    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(qnode->get_logger(),"cv_bridge exception: %s",e.what());
        return;
    }
}

void qtNode::pointsCallback(const my_msgss::msg::Points msg)
{
    world_qpoints = msg;
    Q_EMIT updatePoints();
}

void qtNode::run()
{   
    cout << "node开始运行" << endl;
    rclcpp::init(0, nullptr);
    qnode = std::make_shared<rclcpp::Node>("qt_node");
    /*try {
    node = std::make_shared<rclcpp::Node>("qt_node");
} catch (const std::bad_alloc& e) {
    std::cout << "Failed to allocate memory for node: " << e.what() << std::endl;
} catch (const rclcpp::exceptions::InvalidNodeNameError& e) {
    std::cout << "Invalid node name: " << e.what() << std::endl;
} catch (const rclcpp::exceptions::NameValidationError& e) {
    std::cout << "Node name validation error: " << e.what() << std::endl;
} catch (const std::exception& e) {
    std::cout << "Failed to create node: " << e.what() << std::endl;
}*/
    pnp_pub_ = qnode->create_publisher<std_msgs::msg::Float32MultiArray>("/qt/pnp", 10);
    far_sub_ = qnode->create_subscription<sensor_msgs::msg::Image>("/qt/far_qimage", 1, std::bind(&qtNode::farImageCallback, this, std::placeholders::_1));
    depth_sub_ = qnode->create_subscription<sensor_msgs::msg::Image>("/qt/depth_qimage", 1, std::bind(&qtNode::depthImageCallback, this, std::placeholders::_1));
    points_sub_ = qnode->create_subscription<my_msgss::msg::Points>("/qt/points", 10, std::bind(&qtNode::pointsCallback, this, std::placeholders::_1));
    rclcpp::spin(qnode);
    cout << "node异常关闭" << endl;
    rclcpp::shutdown();
}