#include <cmath>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ORB_SLAM2/include/System.h"
#include"ORB_SLAM2/include/Frame.h"
#include"ORB_SLAM2/include/Map.h"
#include"ORB_SLAM2/include/Tracking.h"
#include "ORB_SLAM2/include/Converter.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>


using std::placeholders::_1;
using std::placeholders::_2;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(ORB_SLAM2::System* pSLAM, const string &strVocFile, const string &strSettingsFile, const string &strDoRectify)
  : Node("subscriber_node"),
  m_SLAM(pSLAM),
  role_name_(declare_parameter<std::string>("role_name", "ego_vehicle")),
  sync(MySyncPolicy(10), left_camera_sub_, right_camera_sub_)
  {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    left_camera_sub_.subscribe(this, "/carla/ego_vehicle/rgb_left/image", rmw_qos_profile),
    right_camera_sub_.subscribe(this, "/carla/ego_vehicle/rgb_right/image", rmw_qos_profile),

    sync.registerCallback(std::bind(&SubscriberNode::inputCallback, this, _1, _2));
    

  }

private:


  void inputCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
                              const sensor_msgs::msg::Image::ConstSharedPtr& right_image) 
  {

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(left_image);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(right_image);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;
    Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,left_image->header.stamp.sec);
    if(Tcw.empty()){
      return;
    }
  
  }



  const std::string role_name_;
  

  message_filters::Subscriber<sensor_msgs::msg::Image> left_camera_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_camera_sub_;

  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync;

  ORB_SLAM2::System* m_SLAM;
};

int main(int argc, char **argv)
{
  if(argc < 4)
  {
      std::cerr << "\nUsage: ros2 run orb_node listener path_to_vocabulary path_to_settings do_rectify" << std::endl;        
      return 1;
  }
  
  rclcpp::init(argc, argv);
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);
  std::cout << "orb slam inizializzato" << std::endl;
  auto node = std::make_shared<SubscriberNode>(&SLAM, argv[1], argv[2], argv[3]);
  
  rclcpp::spin(node);
  
  SLAM.Shutdown();
  
  rclcpp::shutdown();
  SLAM.SaveTrajectoryTUM("ORB_SLAM_trajectory.txt");
  return 0;
}