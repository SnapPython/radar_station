#include "img_sub.hpp"

Img_Sub::Img_Sub() : Node("img_sub")
{
   img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw",1,std::bind(&Img_Sub::img_callback,this,std::placeholders::_1));
   this->yolopoint_pub_ = this->create_publisher<my_msgss::msg::Yolopoint>("/img/yolopoint",1);
   this->yolopoints_pub_ = this->create_publisher<my_msgss::msg::Yolopoints>("/far_rectangles",1);
   this->far_qimage_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/qt/far_qimage",1);
//    this->i = 5;
//    this->declare_parameter("y", 2);
//    this->get_parameter("y", this->i);
//    cout << this->i << endl;
   RCLCPP_INFO(this->get_logger(), "begin to init");
   this->yolo_init();

   // this->test();
}

void Img_Sub::img_callback(sensor_msgs::msg::Image msg)
{
    RCLCPP_INFO(this->get_logger(), "began to receive img");
    this->robot_boxes.data.clear();
    this->robot_boxes.id = 0;
    this->robot_boxes.text = "none";
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    this->sub_img = cv_ptr->image;
    /*if(this->i == 1)
    {
        this->save_img();
        this->i++;
    }*/
    cout << "this->img_sub.rows = " << this->sub_img.rows << endl;
    cout << "this->img_sub.cols = " << this->sub_img.cols << endl;
    this->yolo_robot_identify();
    this->draw_img();
    RCLCPP_INFO(this->get_logger(), "receive_img");
}

void Img_Sub::yolo_init()
{
    bool runOnGPU = false;
    // 1. 设置你的onnx模型
    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    Inference inf_armors("/home/mechax/zyb/radar_station/src/Img_Handle/Yolov8_weight/armor/weights/best.onnx",
                        cv::Size(640, 640),
                        "/home/mechax/zyb/radar_station/src/Img_Handle/Yolov8_weight/armor/class/class.txt",
                        runOnGPU); // classes.txt 可以缺失
    this->inf_armor = inf_armors;
    Inference inf_robots("/home/mechax/zyb/radar_station/src/Img_Handle/Yolov8_weight/robot/weights/best.onnx",
                        cv::Size(640, 640),
                        "/home/mechax/zyb/radar_station/src/Img_Handle/Yolov8_weight/robot/class/class.txt",
                        runOnGPU);
    this->inf_robot = inf_robots;
}

void Img_Sub::yolo_robot_identify()
{
        auto start = std::chrono::steady_clock::now();
        // Inference starts here...
        this->robot_output = this->inf_robot.runInference(this->sub_img);

        int detections = robot_output.size();
        std::cout << "Number of detections:" << detections << std::endl;

        // feiyull
        // 这里需要resize下，否则结果不对
        cv::resize(this->sub_img, this->sub_img, cv::Size(this->sub_img.cols, this->sub_img.rows));

        for (int i = 0; i < detections; ++i)
        {
            Detection detection = robot_output[i];

            cv::Rect box = detection.box;
            if(box.x < 0)
            {
                box.x = 0;
            }
            if(box.y < 0)
            {
                box.y = 0;
            }
            if(box.x + box.width > this->sub_img.cols)
            {
                box.width = this->sub_img.cols - box.x;
            }
            if(box.y + box.height > this->sub_img.rows)
            {
                box.height = this->sub_img.rows - box.y;
            }
            my_msgss::msg::Yolopoint robot_box;
            robot_box.x = box.x;
            robot_box.y = box.y;
            robot_box.width = box.width;
            robot_box.height = box.height;
            this->yolo_armor_identify(robot_box, box);
            this->robot_boxes.data.push_back(robot_box);
            this->robot_boxes.id++;
            this->robot_boxes.text = "have";

            
            /*cv::Scalar color = detection.color;

            // Detection box
            cv::rectangle(this->sub_img, box, color, 2);

            /// Detection box text
            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(this->sub_img, textBox, color, cv::FILLED);
            cv::putText(this->sub_img, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);*/
        }
        /*cv::imshow("Inference", this->sub_img);
        cv::waitKey(0);
        cv::destroyAllWindows();*/
        if(this->robot_boxes.data.size() != 0)
        {
            RCLCPP_INFO(get_logger(), "began to send robot_boxes");
            this->yolopoints_pub_->publish(this->robot_boxes);
        }
        auto end = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Inference took %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

}

void Img_Sub::yolo_armor_identify(my_msgss::msg::Yolopoint &robot_box, cv::Rect &box)
{
    if(box.x < 0)
    {
        box.x = 0;
    }
    if(box.y < 0)
    {
        box.y = 0;
    }
    if(box.x + box.width > this->sub_img.cols)
    {
        box.width = this->sub_img.cols - box.x;
    }
    if(box.y + box.height > this->sub_img.rows)
    {
        box.height = this->sub_img.rows - box.y;
    }
    Mat armor_img = this->sub_img(box);
    resize(armor_img, armor_img, Size(300, 300));
    vector<Detection> armor_output = this->inf_armor.runInference(armor_img);
    resize(armor_img, armor_img, Size(300, 300));
    int detections = armor_output.size();
    for(int j = 0; j < detections; j++)
    {
    if(armor_output[j].class_id <= 8)
        {
            if(armor_output[j].class_id != 0)
            {
                robot_box.id = armor_output[j].class_id + 5;
                robot_box.color = "blue";
            }
            else
            {
                robot_box.id = 11;
                robot_box.color = "blue";
            }
        }
        else if(armor_output[j].class_id >= 9 && armor_output[j].class_id<= 17)
        {
            if(armor_output[j].class_id != 9)
            {
                robot_box.id = armor_output[j].class_id - 10;
                robot_box.color = "red";
            }
            else
            {
                robot_box.id = 5;
                robot_box.color = "red";
            }
        }
    }
}

void Img_Sub::draw_img()
{
    for(int i =0 ;i<this->robot_boxes.data.size();i++)
    {
        cv::Rect box;
        string classString = "不确定";
        box.x = this->robot_boxes.data[i].x;
        box.y = this->robot_boxes.data[i].y;
        box.width = this->robot_boxes.data[i].width;
        box.height = this->robot_boxes.data[i].height;
        cv::rectangle(this->sub_img, box, Scalar(255,0,0), 2);
        if(this->robot_boxes.data[i].id != 5 || this->robot_boxes.data[i].id != 11)
        {
            classString = "3 " + this->robot_boxes.data[i].color;
        }
        else if(this->robot_boxes.data[i].id == 5 || this->robot_boxes.data[i].id == 11)
        {
            classString = "shaobin " + this->robot_boxes.data[i].color;
        }
        else
        {
            classString = "不确定";
        }
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
        cv::rectangle(this->sub_img, textBox, Scalar(0,0,255), cv::FILLED);
        cv::putText(this->sub_img, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }
    far_qimage_pub_->publish(*(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", this->sub_img).toImageMsg()));
    //std::cout << "发送qimage图片" << std::endl;
    /*imshow("Img_Sub::draw_img", this->sub_img);
    waitKey(1);*/
}

void Img_Sub::test()
{
    this->sub_img = imread("/home/mechax/zyb/radar_station/src/Img_Handle/demo_picture/3.jpg");
    this->yolo_robot_identify();
    this->draw_img();
}

void Img_Sub::save_img()
{
    imwrite("/home/mechax/zyb/radar_station/src/Img_Handle/demo_picture/2.png", this->sub_img);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Img_Sub>());
    rclcpp::shutdown();
    return 0;
}