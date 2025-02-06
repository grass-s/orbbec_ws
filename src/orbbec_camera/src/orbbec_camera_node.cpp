#include <iostream>

#include "ros/init.h"
#include "window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

static bool  sync1         = false;
static bool  started      = true;
static bool  hd2c         = false;
static bool  sd2c         = true;
static float alpha        = 0.5;
static int   windowWidth  = 0;
static int   windowHeight = 0;

// key press event processing
void keyEventProcess(Window &app, ob::Pipeline &pipe, std::shared_ptr<ob::Config> config) {
    ////Get the key value
    int key = app.waitKey(10);
    if(key == '+' || key == '=') {
        // Press the + key to increase alpha
        alpha += 0.1f;
        if(alpha >= 1.0f) {
            alpha = 1.0f;
        }
        app.setAlpha(alpha);
    }
    else if(key == '-' || key == '_') {
        // press - key to decrease alpha
        alpha -= 0.1f;
        if(alpha <= 0.0f) {
            alpha = 0.0f;
        }
        app.setAlpha(alpha);
    }
    else if(key == 'D' || key == 'd') {
        // Press the D key to switch the hardware D2C
        try {
            if(!hd2c) {
                started = false;
                pipe.stop();
                hd2c = true;
                sd2c = false;
                config->setAlignMode(ALIGN_D2C_HW_MODE);
                pipe.start(config);
                started = true;
            }
            else {
                started = false;
                pipe.stop();
                hd2c = false;
                sd2c = false;
                config->setAlignMode(ALIGN_DISABLE);
                pipe.start(config);
                started = true;
            }
        }
        catch(std::exception &e) {
            std::cerr << "Property not support" << std::endl;
        }
    }
    else if(key == 'S' || key == 's') {
        // Press the S key to switch the software D2C
        try {
            if(!sd2c) {
                started = false;
                pipe.stop();
                sd2c = true;
                hd2c = false;
                config->setAlignMode(ALIGN_D2C_SW_MODE);
                pipe.start(config);
                started = true;
            }
            else {
                started = false;
                pipe.stop();
                hd2c = false;
                sd2c = false;
                config->setAlignMode(ALIGN_DISABLE);
                pipe.start(config);
                started = true;
            }
        }
        catch(std::exception &e) {
            std::cerr << "Property not support" << std::endl;
        }
    }
    else if(key == 'F' || key == 'f') {
        // Press the F key to switch synchronization
        sync1 = !sync1;
        if(sync1) {
            try {
                // enable synchronization
                pipe.enableFrameSync();
            }
            catch(...) {
                std::cerr << "Sync not support" << std::endl;
            }
        }
        else {
            try {
                // turn off sync
                pipe.disableFrameSync();
            }
            catch(...) {
                std::cerr << "Sync not support" << std::endl;
            }
        }
    }
}

bool update_exposure = false;
bool autoexposure = true;

sensor_msgs::CameraInfo createCameraInfo(const std::string& frame_id, double fx, double fy, double cx, double cy, int width, int height, 
                                         const std::vector<double>& distortion, float R[9], float t[3]) {
  sensor_msgs::CameraInfo cam_info;
  cam_info.header.frame_id = frame_id;

  cam_info.width = width;
  cam_info.height = height;

  cam_info.K[0] = fx;
  cam_info.K[2] = cx;
  cam_info.K[4] = fy;
  cam_info.K[5] = cy;
  cam_info.K[8] = 1.0;

  // 投影矩阵 P
  // cam_info.P[0] = fx;
  // cam_info.P[2] = cx;
  // cam_info.P[5] = fy;
  // cam_info.P[6] = cy;
  // cam_info.P[10] = 1.0;
  cam_info.P[0] = R[0], cam_info.P[1] = R[1], cam_info.P[2] = R[2], cam_info.P[3] = t[0] / 1000.;  // TODO: 都存的depth->rgb外参
  cam_info.P[4] = R[3], cam_info.P[5] = R[4], cam_info.P[6] = R[5], cam_info.P[7] = t[1] / 1000.;
  cam_info.P[8] = R[6], cam_info.P[9] = R[7], cam_info.P[10] = R[8], cam_info.P[11] = t[2] / 1000.;

  // 畸变参数 D
  cam_info.distortion_model = "plumb_bob"; // 常见畸变模型
  cam_info.D = distortion;

  return cam_info;
}

void exposureCallback(const std_msgs::Bool::ConstPtr& msg) {
    autoexposure = msg->data;
    update_exposure = true;
    ROS_INFO("Received autoexposure: %s", msg->data ? "true" : "false");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "orbbec_camera");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher rgb_pub = it.advertise("/camera/color/image_raw", 1);
  image_transport::Publisher depth_pub = it.advertise("/camera/depth/image_raw", 1);
  ros::Publisher rgb_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1);
  ros::Publisher depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1);
  ros::Subscriber sub = nh.subscribe("/autoexposure", 10, exposureCallback);


  ob::Pipeline pipe;

  auto device = pipe.getDevice();
  auto depthModeList = device->getDepthWorkModeList();
  device->switchDepthWorkMode((*depthModeList)[1].name);  // 0：近距离； 1：远距离



  // Configure which streams to enable or disable for the Pipeline by creating a Config
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

  std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
  try {
    // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
    auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    if(colorProfiles) {
      colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
      std::cout << colorProfile->format() << " " << colorProfile->fps() << " " << colorProfile->width() << " " << colorProfile->height() << std::endl;
    }
    config->enableStream(colorProfile);
  } catch(...) {
    std::cerr << "Current device is not support color sensor!" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
  auto                                    depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
  std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
  if(depthProfiles) {
    depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
    std::cout << depthProfile->format() << " " << depthProfile->fps() << " " << depthProfile->width() << " " << depthProfile->height() << std::endl;
    // depthProfile.
  }
  config->enableStream(depthProfile);

  // Configure the alignment mode as hardware D2C alignment
  // config->setAlignMode(ALIGN_D2C_HW_MODE);

  // Start the pipeline with config
  try {
    pipe.start(config);
    pipe.enableFrameSync();
  } catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
  }

  auto info = pipe.getCameraParam();
  auto rgb_intr = info.rgbIntrinsic;
  auto depth_intr = info.depthIntrinsic;
  std::vector<double> rgb_D{info.rgbDistortion.k1, info.rgbDistortion.k2, info.rgbDistortion.p1, info.rgbDistortion.p2, info.rgbDistortion.k3};
  std::vector<double> depth_D{info.depthDistortion.k1, info.depthDistortion.k2, info.depthDistortion.p1, info.depthDistortion.p2, info.depthDistortion.k3};
  sensor_msgs::CameraInfo rgb_camera_info = createCameraInfo("rgb", rgb_intr.fx, rgb_intr.fy, rgb_intr.cx, rgb_intr.cy, rgb_intr.width, rgb_intr.height, rgb_D, info.transform.rot, info.transform.trans);
  sensor_msgs::CameraInfo depth_camera_info = createCameraInfo("depth", depth_intr.fx, depth_intr.fy, depth_intr.cx, depth_intr.cy, depth_intr.width, depth_intr.height, depth_D, info.transform.rot, info.transform.trans);

  // Create a window for rendering and set the resolution of the window
  // Window app("SyncAlignViewer", colorProfile->width(), colorProfile->height(), RENDER_OVERLAY);
  int i = 0;
  while(ros::ok()) {
      // keyEventProcess(app, pipe, config);
      if (update_exposure) {
        device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, autoexposure);
        update_exposure = false;
      }

      auto frameSet = pipe.waitForFrames(100);
      if(frameSet == nullptr) {
        continue;
      }

      auto colorFrame = frameSet->colorFrame();
      auto depthFrame = frameSet->depthFrame();
      // std::cout << i++ << std::endl;
      if(colorFrame != nullptr && depthFrame != nullptr) {

        auto info = pipe.getCameraParam();
        // std::cout << info.depthDistortion.k1 << " " << info.depthIntrinsic.cx << " " << info.rgbIntrinsic.cx << std::endl;
        // std::cout << depthFrame->width() << " " <<  colorFrame->width() << std::endl;
        // std::cout <<colorFrame->format() << " " << depthFrame->format() << std::endl;
        // std::cout << colorFrame->timeStamp() << " " << colorFrame->timeStamp() - depthFrame->timeStamp() << std::endl;
        // app.addToRender({ colorFrame, depthFrame });

        auto rgb_video_frame = colorFrame->as<ob::VideoFrame>();

        cv::Mat rgb_raw_mat(1, rgb_video_frame->dataSize(), CV_8UC1, rgb_video_frame->data());
        cv::Mat rgb_image = cv::imdecode(rgb_raw_mat, 1);

        ros::Time timestamp = ros::Time::now();
        std_msgs::Header header;
        header.stamp = timestamp;
        cv_bridge::CvImage rgb_msg(header, "bgr8", rgb_image);
        rgb_pub.publish(rgb_msg.toImageMsg());
        rgb_info_pub.publish(rgb_camera_info);

        cv::Mat depth_image;
        auto depth_video_frame = depthFrame->as<ob::VideoFrame>();
        cv::Mat depth_raw_mat = cv::Mat(depth_video_frame->height(), depth_video_frame->width(), CV_16UC1, depth_video_frame->data());
        float scale = depth_video_frame->as<ob::DepthFrame>()->getValueScale();

        // threshold to 5.12m
        cv::threshold(depth_raw_mat, depth_image, 5120.0f / scale, 0, cv::THRESH_TRUNC);  // 16UC1  单位毫米
        cv_bridge::CvImage depth_msg(header, "16UC1", depth_image);
        depth_pub.publish(depth_msg.toImageMsg());
        depth_info_pub.publish(depth_camera_info);
      }
      ros::spinOnce();
  }

  // Stop the Pipeline, no frame data will be generated
  pipe.stop();

  return 0;
}