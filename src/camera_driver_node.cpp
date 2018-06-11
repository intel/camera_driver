/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include <algorithm>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "CameraDevice.h"
#include "CameraDeviceAtomIsp.h"

class CameraDriverNode {
public:
  CameraDriverNode(ros::NodeHandle &h, ros::NodeHandle &hCam);
  ~CameraDriverNode();
  bool spin();

private:
  ros::NodeHandle mNH;
  ros::NodeHandle mNhCam;
  image_transport::CameraPublisher mImgPub;
  camera_info_manager::CameraInfoManager mCamInfoMgr;
  sensor_msgs::Image mImgMsg;
  CameraDevice *mCamDev;
  int start();
  int stop();
  int pubData();
  int readData(sensor_msgs::Image &image);
};

CameraDriverNode::CameraDriverNode(ros::NodeHandle &nh, ros::NodeHandle &nhCam)
    : mNH(nh), mNhCam(nhCam), mCamInfoMgr(nhCam) {

  ROS_INFO_STREAM("ROS Node camera_driver");

  // advertise the main image topic
  image_transport::ImageTransport it(mNhCam);
  mImgPub = it.advertiseCamera("image_raw", 1);

  mImgMsg.header.frame_id = "camera";

  mCamDev = new CameraDeviceAtomIsp("/dev/video2");
}

CameraDriverNode::~CameraDriverNode() {
  ROS_INFO_STREAM("~CameraDriverNode");

  stop();

  delete mCamDev;
}

int CameraDriverNode::start() {
  ROS_INFO_STREAM("start");
  int ret = 0;

  ret = mCamDev->init();
  if (ret) {
    ROS_ERROR("Error in init camera");
    return ret;
  }

  ret = mCamDev->setSize(480, 360);
  if (ret) {
    ROS_ERROR("Error in setting resolution");
    return ret;
  }

  ret = mCamDev->setPixelFormat(CameraDevice::PIXEL_FORMAT_GREY);
  if (ret) {
    ROS_ERROR("Error in setting pixel format");
    return ret;
  }

  ret = mCamDev->start();
  if (ret) {
    ROS_ERROR("Error in start camera");
    mCamDev->uninit();
    return ret;
  }

  // Set camera info
  CameraInfo camInfo;
  ret = mCamDev->getInfo(camInfo);
  if (!ret) {
    if (!mCamInfoMgr.isCalibrated()) {
      mCamInfoMgr.setCameraName(camInfo.name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = mImgMsg.header.frame_id;
      camera_info.width = camInfo.width;
      camera_info.height = camInfo.height;
      mCamInfoMgr.setCameraInfo(camera_info);
    }
  }
  return ret;
}

int CameraDriverNode::stop() {
  ROS_INFO_STREAM("stop");
  int ret = 0;

  ret = mCamDev->stop();
  ret = mCamDev->uninit();
  return ret;
}

int CameraDriverNode::readData(sensor_msgs::Image &image) {

  int ret = 0;
  CameraFrame frame;
  ret = mCamDev->read(frame);
  if (!ret) {
    // Form a sensor_msgs::Image with the camera frame
    if (frame.pixFmt == CameraDevice::PIXEL_FORMAT_GREY)
      sensor_msgs::fillImage(image, "mono8", frame.height, frame.width,
                             frame.width, frame.buf);
    else
      ROS_ERROR("Unhandled Pixel Format");

    image.header.stamp = ros::Time(frame.sec, frame.nsec);
  } else
    ROS_ERROR("Error in reading camera frame");

  return ret;
}

int CameraDriverNode::pubData() {
  // ROS_INFO_STREAM("pubData");
  int ret = 0;

  // Get the camera frame
  ret = readData(mImgMsg);
  if (ret) {
    return ret;
  }

  // Get the camera info
  sensor_msgs::CameraInfoPtr ci(
      new sensor_msgs::CameraInfo(mCamInfoMgr.getCameraInfo()));
  ci->header.frame_id = mImgMsg.header.frame_id;
  ci->header.stamp = mImgMsg.header.stamp;

  // publish the image
  mImgPub.publish(mImgMsg, *ci);
  return ret;
}

bool CameraDriverNode::spin() {
  while (!ros::isShuttingDown())
  {
    if (start() == 0) {
      while (mNH.ok()) {
        if (pubData() < 0)
          ROS_WARN("No Camera Frame.");
        ros::spinOnce();
      }
    } else {
      // retry start
      usleep(1000000);
      ros::spinOnce();
    }
  }

  stop();

  return true;
}

int main(int argc, char **argv) {
  // Initialize the ROS system
  ros::init(argc, argv, "camera_driver");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;
  ros::NodeHandle nhCam("camera");
  CameraDriverNode cdn(nh, nhCam);
  cdn.spin();

  return 0;
}
