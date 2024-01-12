#include "get_img.hpp"
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

MVCamera *mv_driver=NULL;
Size dist_size=Size(640,512);
rclcpp::Clock ros2_clock;
rclcpp::Time imgTime;
int cam_cnt;//the number of cameras connected

//SDK使用
extern int                  g_hCamera;          //设备句柄
extern unsigned char        * g_pRgbBuffer;     //处理后数据缓存区
extern tSdkFrameHead        g_tFrameHead;       //图像帧头信息
extern tSdkCameraCapbility  g_tCapability;      //设备描述信息

extern int                  g_read_fps;         //统计帧率
extern int                  g_SaveImage_type;   //保存图像格式

GetImg::GetImg() : Node("get_img")
{
    // Create publisher
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("img/send", 1);

    // Create timer
    timer_ = this->create_wall_timer(500ms, std::bind(&GetImg::timer_callback, this));

    // 保存路径
    rcd_path_ = "/home/mechax/zyb/radar_station/src/Get_Camera_Img/video/";

    this->camera_init();
}

std::string GetImg::mat_type23encoding(int mat_type)
{
    switch(mat_type)
    {
    case CV_8UC1:
       return "mono8";
    case CV_8UC3:
       return "bgr8";
    case CV_16SC1:
       return "mono16";
    case CV_8UC4:
       return "rgba8";
    default:
       return "";
    };
}

void GetImg::camera_init()
{
    cfg_exp_sub = this->create_subscription<std_msgs::msg::Int16>("exp_time", 1, std::bind(&GetImg::get_exp, this, std::placeholders::_1));
    is_rcd_sub = this->create_subscription<std_msgs::msg::Bool>("/mv_param/is_record", 1, std::bind(&GetImg::get_is_rcd, this, std::placeholders::_1));
    //is_large_sub = node_->create_subscription<std_msgs::msg::Bool>("/mv_param/is_large", 1, std::bind(&MVCamNode::get_is_large, this, _1));
    this->get_parameter("deviceID",this->deviceID);
    this->get_parameter("exp_time", this->exposure_);
    this->exposure_ = 30000;
    this->declare_parameter("image_width",640);
    this->get_parameter("image_width", this->image_width_);
    if(large_resolution_)
    {
        this->declare_parameter("image_height", 512);
        this->get_parameter("image_height", this->image_height_);
        this->declare_parameter("framerate",100);
        this->get_parameter("framerate", this->framerate_);
    }
    else
    {
        this->declare_parameter("image_height", 480);
        this->get_parameter("image_height", this->image_height_);
        this->declare_parameter("framerate", 30);
        this->get_parameter("framerate", this->framerate_);
    }
    this->declare_parameter("/framerate", 30);
    this->get_parameter("/framerate", this->framerate_);
    this->get_parameter("/is_record", this->is_record_);
    string project_path;
    this->declare_parameter("/battle_state/fps_mode", 0);
    this->get_parameter("/battle_state/fps_mode", this->fps_mode);
    mv_driver = new MVCamera;
    init_suc=mv_driver->Init(deviceID);
    mv_driver->SetExposureTime(autoexposure_, exposure_);
    mv_driver->SetLargeResolution(large_resolution_);
    mv_driver->Set_fps(fps_mode);
    mv_driver->Play();
}


void GetImg::timer_callback()
{
    this->take_and_send_image();
    //RCLCPP_INFO(this->get_logger(), "Begin to send image");
    // Get image from camera
    /*if(CameraGetImageBuffer(h_camera_, &sFrameInfo_, &pbyBuffer_, 1000) == CAMERA_STATUS_SUCCESS)
    {
        if (sFrameInfo_.uiMediaType == CAMERA_MEDIA_TYPE_MONO8)
        {
            img_ = cv::Mat(sFrameInfo_.iHeight, sFrameInfo_.iWidth, CV_8UC1, pbyBuffer_);
        }
        else if (sFrameInfo_.uiMediaType == CAMERA_MEDIA_TYPE_RGB8)
        {
            img_ = cv::Mat(sFrameInfo_.iHeight, sFrameInfo_.iWidth, CV_8UC3, pbyBuffer_);
        }
        else
        {
            std::cout << "Error: Unknown image type" << std::endl;
        }
        CameraReleaseImageBuffer(h_Camera,pbyBuffer);
    }

    // Publish image
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = this->now();
    img_msg.header.frame_id = "camera";
    img_msg.height = img_.rows;
    img_msg.width = img_.cols;
    img_msg.encoding = this->mat_type23encoding(img_.type());
    img_msg.is_bigendian = false;
    img_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(img_.step);
    img_msg.data = img_.data;
    img_pub_->publish(img_msg);
    RCLCPP_INFO(this->get_logger(), "Image sent");*/
}


void GetImg::get_exp(const std_msgs::msg::Int16 &exp_time)
{
    if(exposure_!=exp_time.data)
    {
        // exposure_=exp_time.data;
        exposure_ = 30000;
        mv_driver->SetExposureTime(autoexposure_, exposure_);

    }
}

void GetImg::get_is_large(const std_msgs::msg::Bool &is_large_resolution)
{
    if(is_large_resolution.data!=large_resolution_) //dafu
    {
        // large_resolution_=is_large_resolution->data;
        // mv_driver->SetLargeResolution(large_resolution_);
        mv_driver->SetExposureTime(0, 30000);
    }else{
        mv_driver->SetExposureTime(0, 15000);
    }
}


void GetImg::get_is_rcd(const std_msgs::msg::Bool &is_rcd)
    {
        if(is_record_!=is_rcd.data)
        {
            is_record_=is_rcd.data;
        }
    }




string GetImg::num2str(double i)

    {
        stringstream ss;
        ss << i;
        return ss.str();
    }
    ///
    /// \brief take_and_send_image
    /// use camera API in MVCamera.cpp
    /// \return
    ///
bool GetImg::take_and_send_image()
    {
        // grab the image

        mv_driver->GetFrame_B(this->rawImg,1);
        imgTime=ros2_clock.now();
        if(this->rawImg.empty())
        {
            RCLCPP_WARN(get_logger(), "NO IMG GOT FROM MV");
            return false;
        }
        if(is_record_)
        {
            saver.write(this->rawImg,rcd_path_);
        }
        if(large_resolution_)
            resize(this->rawImg,this->rawImg,dist_size);

        std_msgs::msg::Header imgHead;
        //DoveJH：用于在订阅者节点区分两个相机。
//        if(far_or_close == 'F')
//        {
//            imgHead.frame_id = "sensor_far";
//            imshow("far_img",rawImg);
//            int k = waitKey(30);
//            if(k == 'f' && !rawImg.empty())
//            {
//                ROS_INFO("Get %d far pictures!", count);
//                cout << rawImg.size().width << '\t' << rawImg.size().height <<endl;
//                stringstream ss;
//                ss<<"/home/chris/ws_livox/src/camera_lidar_calibration/data/photo/far"<<count<<".bmp";
//                cv::imwrite(ss.str(),rawImg);
//                count++;
//            }
//        }
//        else if(far_or_close == 'C')
//        {
//            imgHead.frame_id = "sensor_close";
//            imshow("close_img",rawImg);
//            int k = waitKey(30);
//            if(k == 'c' && !rawImg.empty())
//            {
//                ROS_INFO("Get %d far pictures!", count);
//                cout << rawImg.size().width << '\t' << rawImg.size().height <<endl;
//                stringstream ss;
//                ss<<"/home/chris/ws_livox/src/camera_lidar_calibration/data/photo/close"<<count<<".bmp";
//                cv::imwrite(ss.str(),rawImg);
//                count++;
//            }
//        }
        
        imgHead.stamp=imgTime;
        this->msg= *(cv_bridge::CvImage(imgHead, "bgr8", this->rawImg).toImageMsg());
        // publish the image
        img_pub_->publish(this->msg);
        // imshow("img",this->rawImg);
        // waitKey(1);
        cout << "publish image" << endl;

//        std::cout<<rawImg.size<<std::endl;
        return true;
    }



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetImg>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
