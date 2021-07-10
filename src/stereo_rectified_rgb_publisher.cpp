
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <oakd_pcloud/stereo_pipeline.hpp>
#include <functional>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

// using namespace std::placeholders;
int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_rectified_rgb_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    StereoExampe stero_pipeline;
    stero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();

    ROS_INFO_STREAM("Data queque sizes:" << imageDataQueues.size());
    
    // this part would be removed once we have calibration-api
    std::string left_uri = camera_param_uri + "/" + "left.yaml";
    std::string right_uri = camera_param_uri + "/" + "right.yaml";
    std::string rectified_left_uri = camera_param_uri + "/" + "rectified_left.yaml";
    std::string rectified_right_uri = camera_param_uri + "/" + "rectified_right.yaml";
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(imageDataQueues[2],
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     stereo_uri,
                                                                                     "stereo");

    depthPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter leftRectifiedconverter(deviceName + "_left_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftRectifiedPublish(imageDataQueues[3],
                                                                                     pnh, 
                                                                                     std::string("rectified_left/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &leftRectifiedconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     left_uri,
                                                                                     "rectified_left");

    leftRectifiedPublish.addPubisherCallback();


    dai::rosBridge::ImageConverter rightRectifiedconverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightRectifiedPublish(imageDataQueues[4],
                                                                                     pnh, 
                                                                                     std::string("rectified_right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightRectifiedconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     right_uri,
                                                                                     "rectified_right");

    rightRectifiedPublish.addPubisherCallback();

    
    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[5],
                                                                                  pnh, 
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter, // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");
    rgbPublish.startPublisherThread();

    ros::spin();
    return 0;
}

