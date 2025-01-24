#include <iostream>

#include "ros/init.h"
#include "window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <unistd.h>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct RGBDCameraInfo {
  RGBDCameraInfo(sensor_msgs::CameraInfo &rgb, sensor_msgs::CameraInfo &depth) {
    rgb_camera_info = rgb;
    depth_camera_info = depth;
  }
  sensor_msgs::CameraInfo rgb_camera_info;
  sensor_msgs::CameraInfo depth_camera_info;
};

const int maxDeviceCount = 9;

std::vector<std::shared_ptr<ob::Frame>> frames;
std::shared_ptr<ob::Frame>              colorFrames[maxDeviceCount];
std::shared_ptr<ob::Frame>              depthFrames[maxDeviceCount];
std::mutex                              frameMutex;

std::vector<image_transport::Publisher> rgb_pubs;
std::vector<image_transport::Publisher> depth_pubs;
std::vector<ros::Publisher> rgb_info_pubs;
std::vector<ros::Publisher> depth_info_pubs;


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

int main(int argc, char **argv) {
  ros::init(argc, argv, "orbbec_camera");
  ros::NodeHandle nh;

  ob::Context ctx;
  auto devList = ctx.queryDeviceList();
  int devCount = devList->deviceCount();
  image_transport::ImageTransport it(nh);

  for(int i = 0; i < devCount; i++) {
    std::string rgb_topic = "/camera/color/image_raw_" + std::to_string(i);
    std::string depth_topic = "/camera/depth/image_raw_" + std::to_string(i);
    std::string rgb_info = "/camera/color/camera_info_" + std::to_string(i);
    std::string depth_info = "/camera/depth/camera_info_" + std::to_string(i);

    rgb_pubs.emplace_back(it.advertise(rgb_topic, 5));
    depth_pubs.emplace_back(it.advertise(depth_topic, 5));
    rgb_info_pubs.emplace_back(nh.advertise<sensor_msgs::CameraInfo>(rgb_info, 1));
    depth_info_pubs.emplace_back(nh.advertise<sensor_msgs::CameraInfo>(depth_info, 1));
  }


  std::vector<std::shared_ptr<ob::Pipeline>> pipes;
  std::vector<RGBDCameraInfo> camera_infos;
  for(int i = 0; i < devCount; i++) {
    // Get the device and create the pipeline
    auto dev  = devList->getDevice(i);
    auto pipe = std::make_shared<ob::Pipeline>(dev);

    auto device = pipe->getDevice();
    auto depthModeList = device->getDepthWorkModeList();
    device->switchDepthWorkMode((*depthModeList)[1].name);  // 0：近距离； 1：远距离

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
      // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
      auto colorProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
      if(colorProfiles) {
        colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
        std::cout << colorProfile->format() << " " << colorProfile->fps() << " " << colorProfile->width() << " " << colorProfile->height() << std::endl;
      }
      config->enableStream(colorProfile);
    } catch(...) {
      std::cerr << "Current device is not support color sensor!" << std::endl;
      exit(EXIT_FAILURE);
    }

    auto                                    depthProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    if(depthProfiles) {
      depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
      std::cout << depthProfile->format() << " " << depthProfile->fps() << " " << depthProfile->width() << " " << depthProfile->height() << std::endl;
      // depthProfile.
    }
    config->enableStream(depthProfile);


    // Start the pipeline with config
    // try {
    //   pipe->start(config);
    //   pipe->enableFrameSync();
    // } catch(ob::Error &e) {
    //   std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    // }

    pipe->start(config, [i](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(frameMutex);
        if(frameSet->colorFrame()) {
            colorFrames[i] = frameSet->colorFrame();
        }
        if(frameSet->depthFrame()) {
            depthFrames[i] = frameSet->depthFrame();
        }
    });

    auto info = pipe->getCameraParam();
    auto rgb_intr = info.rgbIntrinsic;
    auto depth_intr = info.depthIntrinsic;
    std::vector<double> rgb_D{info.rgbDistortion.k1, info.rgbDistortion.k2, info.rgbDistortion.p1, info.rgbDistortion.p2, info.rgbDistortion.k3};
    std::vector<double> depth_D{info.depthDistortion.k1, info.depthDistortion.k2, info.depthDistortion.p1, info.depthDistortion.p2, info.depthDistortion.k3};
    sensor_msgs::CameraInfo rgb_camera_info = createCameraInfo("rgb", rgb_intr.fx, rgb_intr.fy, rgb_intr.cx, rgb_intr.cy, rgb_intr.width, rgb_intr.height, rgb_D, info.transform.rot, info.transform.trans);
    sensor_msgs::CameraInfo depth_camera_info = createCameraInfo("depth", depth_intr.fx, depth_intr.fy, depth_intr.cx, depth_intr.cy, depth_intr.width, depth_intr.height, depth_D, info.transform.rot, info.transform.trans);


    RGBDCameraInfo camera_info(rgb_camera_info, depth_camera_info);
    camera_infos.emplace_back(camera_info);
    pipes.emplace_back(pipe);
  }

  while(ros::ok()) {
    // keyEventProcess(app, pipe, config);
    frames.clear();
    {
      std::lock_guard<std::mutex> lock(frameMutex);
      int i = 0;
      for(auto pipe: pipes) {
        if(colorFrames[i] != nullptr) {
          frames.emplace_back(colorFrames[i]);
        }
        if(depthFrames[i] != nullptr) {
          frames.emplace_back(depthFrames[i]);
        }
        i++;
        auto colorFrame = frames[0];
        auto depthFrame = frames[1];

        if(colorFrame != nullptr && depthFrame != nullptr) {
          auto rgb_video_frame = colorFrame->as<ob::VideoFrame>();

          cv::Mat rgb_raw_mat(1, rgb_video_frame->dataSize(), CV_8UC1, rgb_video_frame->data());
          cv::Mat rgb_image = cv::imdecode(rgb_raw_mat, 1);

          ros::Time timestamp = ros::Time::now();
          std_msgs::Header header;
          header.stamp = timestamp;
          cv_bridge::CvImage rgb_msg(header, "bgr8", rgb_image);
          rgb_pubs[i].publish(rgb_msg.toImageMsg());
          rgb_info_pubs[i].publish(camera_infos[i].rgb_camera_info);

          cv::Mat depth_image;
          auto depth_video_frame = depthFrame->as<ob::VideoFrame>();
          cv::Mat depth_raw_mat = cv::Mat(depth_video_frame->height(), depth_video_frame->width(), CV_16UC1, depth_video_frame->data());
          float scale = depth_video_frame->as<ob::DepthFrame>()->getValueScale();

          // threshold to 5.12m
          cv::threshold(depth_raw_mat, depth_image, 5120.0f / scale, 0, cv::THRESH_TRUNC);  // 16UC1  单位毫米
          cv_bridge::CvImage depth_msg(header, "16UC1", depth_image);
          depth_pubs[i].publish(depth_msg.toImageMsg());
          depth_info_pubs[i].publish(camera_infos[i].depth_camera_info);
        } else {
          usleep(1000);
        }
        ros::spinOnce();
        i++;
      }
    }
  }

  int i = 0;
  for(auto &&pipe: pipes) {
    if(colorFrames[i]) colorFrames->reset();
    if(depthFrames[i]) depthFrames->reset();
    pipe->stop();
    i++;
  }

  return 0;
}