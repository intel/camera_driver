// ROS node for Aero camera

#define TEST 0

#include <algorithm>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "CameraDevice.h"
#include "CameraDeviceAtomIsp.h"

class AeroCameraNode {
public:
  AeroCameraNode(ros::NodeHandle h);
  ~AeroCameraNode();
  bool spin();

private:
  ros::NodeHandle mNH;
  image_transport::CameraPublisher mImgPub;
  sensor_msgs::Image mImgMsg;
  CameraDevice *mCamDev;
  int start();
  int stop();
  int pubData();
  int readData(sensor_msgs::Image &image);
};

AeroCameraNode::AeroCameraNode(ros::NodeHandle nh) : mNH(nh) {

  mNH = ros::NodeHandle("~");

  // advertise the main image topic
  image_transport::ImageTransport it(mNH);
  mImgPub = it.advertiseCamera("image_raw", 1);

  ROS_INFO_STREAM("ROS Node aero_camera");

  mCamDev = new CameraDeviceAtomIsp("/dev/video2");
}

AeroCameraNode::~AeroCameraNode() {
  ROS_INFO_STREAM("~AeroCameraNode");

  stop();

  delete mCamDev;
}

int AeroCameraNode::start() {
  ROS_INFO_STREAM("start");
  int ret = 0;

  ret = mCamDev->init();
  if (ret) {
    ROS_ERROR("Error in init camera");
    return ret;
  }

  ret = mCamDev->start();
  if (ret) {
    ROS_ERROR("Error in start camera");
    mCamDev->uninit();
    return ret;
  }

  return ret;
}

int AeroCameraNode::stop() {
  ROS_INFO_STREAM("stop");
  int ret = 0;

  ret = mCamDev->stop();
  ret = mCamDev->uninit();
  return ret;
}

int AeroCameraNode::readData(sensor_msgs::Image &image) {

  int ret = 0;
  CameraFrame frame;
  ret = mCamDev->read(frame);
  if (!ret) {
    // Form a sensor_msgs::Image with the camera frame
    sensor_msgs::fillImage(image, "yuv422", 480, 640, 640 * 2, frame.buf);
  } else
    ROS_ERROR("Error in reading camera frame");

  return ret;
}

int AeroCameraNode::pubData() {
  // ROS_INFO_STREAM("pubData");
  int ret = 0;

  // read
  ret = readData(mImgMsg);
  if (ret) {
    return ret;
  }

  // grab the camera info
  sensor_msgs::CameraInfo ci;

  // publish the image
  mImgPub.publish(mImgMsg, ci);
  return ret;
}

bool AeroCameraNode::spin() {
  while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid
                                 // restarting the node during a shutdown.
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
  ros::init(argc, argv, "aero_camera");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;
  AeroCameraNode acn(nh);
  acn.spin();

  return 0;
}
