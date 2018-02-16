// ROS node for Aero camera

#include "aero_camera_node.h"
#include "CameraDeviceAtomIsp.h"
#include <algorithm>

AeroCameraNode::AeroCameraNode() {
  // Establish this program as a ROS node.
  nh = ros::NodeHandle("~");

  ROS_INFO_STREAM("ROS Node aero_camera");

  // Create publisher for bottom camera
  mPubCamBottomImage =
      nh.advertise<sensor_msgs::Image>("/camera/bottom/image_raw", 1);

#if TEST
  // Subscribe to already available topic
  mSubRawImage = nh.subscribe("/cam0/image_raw", 1,
                              &AeroCameraNode::rawImageCallback, this);
#endif

  mCamDev = new CameraDeviceAtomIsp("/dev/video2");

  mCamDev->init();

  // mCamDev->start(std::bind(&AeroCameraNode::cameraFrameCallback,
  // this,std::placeholders::_1));
  mCamDev->start(std::bind(&AeroCameraNode::cameraFrameCallback2, this,
                           std::placeholders::_1, std::placeholders::_2));
}

AeroCameraNode::~AeroCameraNode() {
  ROS_INFO_STREAM("~AeroCameraNode");

  mCamDev->stop();

  mCamDev->uninit();
}

void AeroCameraNode::cameraFrameCallback2(void *frame, size_t frameLen) {
  ROS_INFO_STREAM("cameraFrameCallback2");
  sensor_msgs::Image image;

  // Form a sensor_msgs::Image with the camera frame
  sensor_msgs::fillImage(image, "yuv422", 480, 640, 640 * 2, frame);

  // Publish it in the topic
  mPubCamBottomImage.publish(image);
  sensor_msgs::clearImage(image);
}

void AeroCameraNode::cameraFrameCallback(std::vector<uint8_t> frame) {
  ROS_INFO_STREAM("cameraFrameCallback");
  sensor_msgs::Image image;
  std::string encoding;
  uint32_t height, width, stride;

  // Form a sensor_msgs::Image with the camera frame
  sensor_msgs::fillImage(image, "yuv422", 480, 640, 640 * 2, &frame[0]);

  // Publish it in the topic
  mPubCamBottomImage.publish(image);
}

#if TEST
void AeroCameraNode::rawImageCallback(
    const sensor_msgs::ImageConstPtr &imgMsg) {
  // publish it in new topic
  ROS_INFO_STREAM("rawImageCallback");
  mPubCamBottomImage.publish(imgMsg);
}
#endif

int main(int argc, char **argv) {
  // Initialize the ROS system
  ros::init(argc, argv, "aero_camera");
  AeroCameraNode aeroCameraNode;
  ros::Duration(2).sleep();
  ros::spin();

  return 0;
}
