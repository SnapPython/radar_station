#include <zconf.h>
#include "MVCamera.hpp"
#include "rclcpp/rclcpp.hpp"

#include <signal.h>

extern int cam_cnt;

MVCamera::MVCamera() 
{
    iCameraCounts = 4;
    iStatus = 0;
    hCamera = 0;
    channel = 3;

    //tSdkCameraDevInfo       MVCamera::tCameraEnumList[4];
    tCapability;      //设备描述信息
    sFrameInfo;
    g_pRgbBuffer[0] = NULL;
    g_pRgbBuffer[1] = NULL;
    ready_buffer_id = 0;
    stopped = false;
    updated = false;
}

int MVCamera::Init(int id)
{
    cout << "CAMERA SDK INIT..." << endl;
    CameraSdkInit(1);
    cout << "DONE" << endl;
    cout << "ENUM CAMERA DEVICES..." << endl;
    int cam_id;
    //枚举设备，并建立设备列表CameraEnumerateDevice
    tSdkCameraDevInfo tCameraEnumList[4];
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
    // string ns=rclcpp::Node::get_namespace();
    //rclcpp::Node node("my_camera_node");
    //std::string ns = this->get_namespace();
    //string name_space;


    /*if(ns=="/sensor_far")name_space="FARCAM";
    if(ns=="/sensor_close")name_space="CLOSECAM";
    if(tCameraEnumList[0].acFriendlyName==name_space){
        id=0;

    }
    if(tCameraEnumList[1].acFriendlyName==name_space){
        id=1;
    }*/
    id = 0;
    cam_cnt=iCameraCounts;
//    if(iCameraCounts>0)cout<<tCameraEnumList[0].acFriendlyName<<endl;
    //没有连接设备

    // cout<<"ns:"<<ns<<endl;
    if(iCameraCounts==0){
        cout << "ERROR: NO CAMERA CONNECTED.\n" << endl;
        return -1;
    } else if (iCameraCounts <= id) {
        cout << "CONNECTED CAMERA NUMBERS:" << iCameraCounts << "TOO SMALL, ID:" << id << endl;
        return -1;
    } else {
        cout << "CONNECTED CAMERA NUMBERS:" << iCameraCounts << endl;
    }
    cout << "DONE" << endl;

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&(tCameraEnumList[id]),PARAM_MODE_BY_NAME,-1,&hCamera);
    //初始化失败
    if (iStatus!=CAMERA_STATUS_SUCCESS) {
        cout << "ERROR: CAMERA INIT FAILED" << endl;
        return -1;
    } else {
        cout << "CAMERA INIT SUCCESS." << endl;
    }

    //设置色温模式
//      if(id==0) {
//          iStatus = CameraSetPresetClrTemp(hCamera, 0);
//          if (iStatus == CAMERA_STATUS_SUCCESS) {
//              printf("CAMERA SETPRESETCLRTEMP SUCCESS!\n");
//          } else {
//              printf("CAMERA SETPRESETCLRTEMP FAILED! ERROR CODE: %d\n", iStatus);
//          }
//      }

    //  iStatus = CameraSetClrTempMode(hCamera, 1);
    //	if (iStatus == CAMERA_STATUS_SUCCESS) {
    //    printf("CAMERA SETCLRTEMPMODE SUCCESS!\n");
    //	} else {
    //    printf("CAMERA SETCLRTEMPMODE FAILED! ERROR CODE: %d\n", iStatus);
    //	}

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
//    CameraGetCapability(hCamera,&tCapability);
    CameraPlay(hCamera);

    //设置输出为彩色
    channel = 3;
    CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);

    //初始化缓冲区
    g_pRgbBuffer[0] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    g_pRgbBuffer[1] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    started=1;
    return true;
}

int MVCamera::Uninit()
{
    cout << "Save Parameter...\n" << endl;
    CameraSaveParameter(hCamera, 0);

    cout << "Uninit...\n" << endl;
    int status = CameraUnInit(hCamera);

    cout << "status:" << status << endl;

    if (status == CAMERA_STATUS_SUCCESS) {
        cout << "CAMERA UNINIT SUCCESS!" << endl;
    } else {
        cout << "CAMERA UNINIT FAILED! ERROR CODE:" << status << endl;
    }

    if (g_pRgbBuffer[0] != NULL) {
        free(g_pRgbBuffer[0]);
        g_pRgbBuffer[0] = NULL;
    }

    if (g_pRgbBuffer[1] != NULL) {
        free(g_pRgbBuffer[1]);
        g_pRgbBuffer[1] = NULL;
    }
    started=0;
    return 0;
}

int MVCamera::SetExposureTime(bool auto_exp, double exp_time)
{
    if (auto_exp) {
        CameraSdkStatus status = CameraSetAeState(hCamera, true);
        if (status == CAMERA_STATUS_SUCCESS) {
            cout << "ENABLE AUTO EXP SUCCESS" << endl;
        } else {
            cout << "ENABLE AUTO EXP FAILED." << endl;
            return status;
        }
    } else {
        CameraSdkStatus status = CameraSetAeState(hCamera, false);
        if (status == CAMERA_STATUS_SUCCESS) {
            cout << "DISABLE AUTO EXP SUCCESS." << endl;
        } else {
            cout << "DISABLE AUTO EXP FAILED." << endl;
            return status;
        }
        CameraSdkStatus status1 = CameraSetExposureTime(hCamera, exp_time);
        if (status1 == CAMERA_STATUS_SUCCESS) {
        cout << "SET EXP TIME SUCCESS." << endl;
        } else {
            cout << "SET EXP TIME FAILED." << endl;
            return status;
        }
    }
    CameraGetCapability(hCamera,&tCapability);


    return 0;
}

double MVCamera::GetExposureTime()
{
    int auto_exp;
    if (CameraGetAeState(hCamera, &auto_exp) == CAMERA_STATUS_SUCCESS) {
        if (auto_exp) {
            return 0;
        } else {
            double exp_time;
            if (CameraGetExposureTime(hCamera, &exp_time) == CAMERA_STATUS_SUCCESS) {
                return exp_time;
            } else {
                cout << "GET CAMERA EXP TIME ERROR." << endl;
                return -1;
            }
        }
    } else {
        cout << "GET CAMERA AE STATE ERROR." << endl;
        return -1;
    }

}

int MVCamera::SetLargeResolution(bool if_large_resolution)
{
    tSdkImageResolution resolution;
    if (if_large_resolution) {
        resolution.iIndex = 0;
        if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
            cout << "CAMERA SET LARGE RESOLUTION SUCCESS." << endl;
        } else {
            cout << "CAMERA SET LARGE RESOLUTION FAILED." << endl;
            return -1;
        }
    } else {
        resolution.iIndex = 1;
        CameraSetImageResolution(hCamera, &resolution);
        if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
            cout << "CAMERA SET SMALL RESOLUTION SUCCESS." << endl;
        } else {
            cout << "CAMERA SET SMALL RESOLUTION FAILED." << endl;
            return -1;
        }
    }
    CameraGetCapability(hCamera,&tCapability);

    return 0;
}

// white balance
int MVCamera::SetWBMode(bool auto_wb)
{
    int status = CameraSetWbMode(hCamera, auto_wb);
    if (CAMERA_STATUS_SUCCESS == status) {
        cout <<"CAMERA SETWBMODE " << auto_wb << "SUCCESS!" << endl;
    } else {
        cout <<"CAMERA SETWBMODE " << auto_wb << "status FAILED! ERROR CODE: " << status << endl;
    }
    return 0;
}

int MVCamera::GetWBMode(bool &auto_wb)
{
    int res = 0;
    if (CAMERA_STATUS_SUCCESS == CameraGetWbMode(hCamera, &res)) {
        printf("CAMERA GETWBMODE %d SUCCESS!\n", res);
    } else {
        printf("CAMERA GETWBMODE FAILED!\n");
    }
    auto_wb = res;
    return 0;;
}

int MVCamera::SetOnceWB()
{
    int status = CameraSetOnceWB(hCamera);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("CAMERA SETONCEWB SUCCESS!\n");
    } else {
        printf("CAMERA SETONCEWB FAILED, ERROR CODE: %d!\n", status);
    }
    return 0;
}

int MVCamera::SetGain(double gain)
{
    int set_gain = int(gain*100);
    int status = CameraSetGain(hCamera, set_gain, set_gain, set_gain);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("CAMERA SETGAIN SUCCESS!\n");
    } else {
        printf("CAMERA SETGAIN FAILED! ERROR CODE: %d\n", status);
    }
    CameraGetCapability(hCamera,&tCapability);
    return 0;
}

double MVCamera::GetGain()
{
    int r_gain, g_gain, b_gain;
    int status = CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("CAMERA GETGAIN SUCCESS!\n");
    } else {
        printf("CAMERA GETGAIN FAILED! ERROR CODE: %d\n", status);
    }

    return (r_gain + g_gain + b_gain)/300.;
}

int MVCamera::Play()
{
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    CameraPlay(hCamera);
    //    std::thread thread1(MVCamera::Read);
    //    thread1.detach();
    return 0;
}
// do not use
int MVCamera::Read()
{
    while (!stopped) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer[(ready_buffer_id+1)%2], &sFrameInfo);
            //            if (iplImage)
            //            {
            //                cvReleaseImageHeader(&iplImage);
            //            }
            //            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,1);

            //            cvSetData(iplImage,pbyBuffer,sFrameInfo.iWidth);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率

            ready_buffer_id = (ready_buffer_id + 1)%2;
            updated = true;
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }
    return 0;
}
int MVCamera::Set_fps(int fps_mode)
{
  int status=CameraSetFrameSpeed(hCamera,fps_mode);
  if(status==CAMERA_STATUS_SUCCESS)
  {
  printf("CAMERA SET FPS MODE SUCCESS! current mode is %d \n" ,fps_mode);
  }else
  {
    printf("CAMERA SET FPS MODE FAILED! ERROR CODE: %d\n", status);
    return -1;// the problem that camera failed to grab img often occurs when in high speed mode(USB3)

  }
  CameraGetCapability(hCamera,&tCapability);
return 0;
}
///
/// \brief MVCamera::GetFrame_B
/// \param frame
/// \param is_color
/// true if we want bgr img
/// \return
///
int MVCamera::GetFrame_B(Mat &frame,bool is_color)
{
    if (!stopped) {

        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
          // the problem that camera failed to grab img often occurs when in high speed mode(USB3.0)

            if (frame.cols != sFrameInfo.iWidth || frame.rows != sFrameInfo.iHeight) {
                printf("GetFrame: resize frame !\n");
                if(is_color)
                {
                    frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);

                }else
                {
                    frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1);
                }
            }

            if (iplImage)
            {
              // delete old one
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,1);

            cvSetData(iplImage,pbyBuffer,sFrameInfo.iWidth);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
            Mat Iimag=cvarrToMat(iplImage);//这里只是进行指针转换，将IplImage转换成Mat类型
            cv::cvtColor(Iimag,frame,COLOR_BayerRG2BGR);


            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }


    }
    return 0;
}
// do not use
int MVCamera::GetFrame(Mat &frame,bool is_color)
{
    //init iplImage
    // if we change resolution
    if (frame.cols != sFrameInfo.iWidth || frame.rows != sFrameInfo.iHeight) {
        printf("GetFrame: resize frame !\n");
        if(is_color)
        {
            frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);

        }else
        {
            frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1);
        }
    }

    while (!updated) {
        usleep(1000);
    }
    if(is_color)
    {
        memcpy(frame.data, g_pRgbBuffer[ready_buffer_id], frame.cols*frame.rows*3);
        //        Mat Iimag=cvarrToMat(iplImage);//这里只是进行指针转换，将IplImage转换成Mat类型
        //        cv::cvtColor(Iimag,frame,CV_BayerGR2BGR);

    }
    else
    {
        memcpy(frame.data, g_pRgbBuffer[ready_buffer_id], frame.cols*frame.rows);

    }
    updated = false;
    return 0;
}

int MVCamera::Stop()
{
    stopped = true;
    usleep(30000);
    return 0;
}
