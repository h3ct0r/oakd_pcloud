#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>

#include "depthai/depthai.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <opencv2/imgproc/imgproc.hpp>


#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <oakd_pcloud/WLSfilterConfig.h>

namespace oakd_pcloud {

namespace enc = sensor_msgs::image_encodings;

class WlsFilterNodelet : public nodelet::Nodelet
{
  ros::NodeHandlePtr right_nh_;
  ros::NodeHandlePtr left_nh_;

  //dynamic_reconfigure::Server<oakd_pcloud::WLSfilterConfig> reconfigureServer;
  //boost::shared_ptr<reconfigureServer> server;
  dynamic_reconfigure::Server<oakd_pcloud::WLSfilterConfig> server;
  dynamic_reconfigure::Server<oakd_pcloud::WLSfilterConfig>::CallbackType f;

  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);

  float baseline = 75.0; // mm
  float disp_levels = 96.0;
  float fov = 71.86;
  float fov_rad = (fov / 2) * (M_PI / 180);

  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> left_it_, right_it_, depth_it_;
  image_transport::SubscriberFilter sub_depth_, sub_left_, sub_right_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_left_info_, sub_right_info_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                          sensor_msgs::Image, 
                                                          sensor_msgs::CameraInfo, 
                                                          sensor_msgs::Image, 
                                                          sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::CameraInfo, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  boost::shared_ptr<ExactSynchronizer> exact_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_depth_;

  //image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& left_msg,
               const sensor_msgs::CameraInfoConstPtr& left_info_msg,
               const sensor_msgs::ImageConstPtr& right_msg,
               const sensor_msgs::CameraInfoConstPtr& right_info_msg);

  void dynCfgCallback(oakd_pcloud::WLSfilterConfig &config, uint32_t level);
};

void WlsFilterNodelet::onInit()
{
  wls_filter->setLambda(16400.0);
  wls_filter->setSigmaColor(2.6);

  f = boost::bind(&WlsFilterNodelet::dynCfgCallback, this, _1, _2);
  server.setCallback(f);

  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  left_nh_.reset( new ros::NodeHandle(nh, "") );
  right_nh_.reset( new ros::NodeHandle(nh, "") );
  ros::NodeHandle depth_nh(nh, "");
  left_it_.reset( new image_transport::ImageTransport(*left_nh_) );
  right_it_.reset( new image_transport::ImageTransport(*right_nh_) );
  depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 10);
  bool use_exact_sync;
  private_nh.param("exact_sync", use_exact_sync, false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync)
  {
    exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_left_, sub_left_info_, sub_right_, sub_right_info_) );
    exact_sync_->registerCallback(boost::bind(&WlsFilterNodelet::imageCb, this, _1, _2, _3, _4, _5));
  }
  else
  {
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_left_, sub_left_info_, sub_right_, sub_right_info_) );
    sync_->registerCallback(boost::bind(&WlsFilterNodelet::imageCb, this, _1, _2, _3, _4, _5));
  }
  
  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&WlsFilterNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_depth_ = depth_nh.advertise<sensor_msgs::Image>("wls_depth", 1, connect_cb, connect_cb);
}

void WlsFilterNodelet::dynCfgCallback(oakd_pcloud::WLSfilterConfig &config, uint32_t level) {
  NODELET_INFO("Reconfigure Request: %f %f", 
            config.lambda, config.sigma);
  wls_filter->setLambda(config.lambda);
  wls_filter->setSigmaColor(config.sigma);
}

// Handles (un)subscribing when clients (un)subscribe
void WlsFilterNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_.getNumSubscribers() == 0)
  {
    sub_depth_.unsubscribe();
    sub_left_.unsubscribe();
    sub_left_info_.unsubscribe();
    sub_right_.unsubscribe();
    sub_right_info_.unsubscribe();
  }
  else if (!sub_depth_.getSubscriber())
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";

    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints("raw",ros::TransportHints(), private_nh, depth_image_transport_param);
    sub_depth_.subscribe(*depth_it_, "depth_image",       1, depth_hints);

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
    sub_left_  .subscribe(*left_it_, "left_image", 1, hints);
    sub_left_info_ .subscribe(*left_nh_,   "left_image_info",      1);

    sub_right_  .subscribe(*right_it_, "right_image", 1, hints);
    sub_right_info_ .subscribe(*right_nh_,   "right_image_info",      1);
  }
}

void WlsFilterNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::ImageConstPtr& left_msg,
                                const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                                const sensor_msgs::ImageConstPtr& right_msg,
                                const sensor_msgs::CameraInfoConstPtr& right_info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != right_msg->header.frame_id)
  {
    NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match right image frame id [%s]",
                           depth_msg->header.frame_id.c_str(), right_msg->header.frame_id.c_str());
    return;
  }

  cv_bridge::CvImagePtr depth_cv_ptr;
  cv_bridge::CvImagePtr left_cv_ptr;
  cv_bridge::CvImagePtr right_cv_ptr;
  try
  {
    depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    left_cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::TYPE_8UC1);
    right_cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::TYPE_8UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update camera model
  //model_.fromCameraInfo(info_msg);
  
  //NODELET_INFO("Passei por aqui....");

  cv::Mat leftFrame, rightFrame;
  cv::flip(left_cv_ptr->image, leftFrame, 1);
  cv::flip(right_cv_ptr->image, rightFrame, 1);
  //cv::imwrite("/tmp/rightFrame.jpg", rightFrame);
  //cv::imwrite("/tmp/depthFrame.jpg", depth_cv_ptr->image);
  //cv::imshow("flipped", rightFrame);

  cv::Mat res;
  wls_filter->filter(depth_cv_ptr->image, rightFrame, res, leftFrame);
  
  //cv::imwrite("/tmp/depthres.jpg", res);

  // float focal = depth_msg->width / (2. * tan(fov_rad));
  // float depthScaleFactor = baseline * focal;

  //NODELET_INFO_STREAM("depthScaleFactor:" << depthScaleFactor);
  // NODELET_INFO_STREAM("depth encoding:" << depth_msg->encoding);
  // NODELET_INFO_STREAM("right encoding:" << right_msg_in->encoding);

  //res = (depthScaleFactor / res);

  cv::Mat im_color;

  cv::Mat image_grayscale = res.clone();
  image_grayscale.convertTo(image_grayscale, CV_8U, 1/257);

  // cv::Mat res8unit = (res * (255/(96-1))) .astype(np.uint8)

  cv::applyColorMap(image_grayscale, im_color, cv::COLORMAP_JET);
  cv::imwrite("/tmp/depthres_color.jpg", im_color);

  sensor_msgs::Image filtered_depth;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(depth_msg->header, sensor_msgs::image_encodings::TYPE_16UC1, res);
  img_bridge.toImageMsg(filtered_depth); // from cv_bridge to sensor_msgs::Image
  pub_depth_.publish(filtered_depth); 
}

} // namespace oakd_pcloud

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oakd_pcloud::WlsFilterNodelet,nodelet::Nodelet);