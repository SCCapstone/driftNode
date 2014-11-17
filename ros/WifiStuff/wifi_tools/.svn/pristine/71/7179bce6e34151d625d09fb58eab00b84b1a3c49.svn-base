/*
 * Copyright (c) 2013, Seigo ITO
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Seigo ITO nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "std_srvs/Empty.h"

// WiFi
#include "wifi_tools/WifiData.h"
#include "wifi_tools/Out2File.h"



class WiFiMappingNode
{
public:
  WiFiMappingNode();
  ~WiFiMappingNode();

  void wifiScanReceived(const wifi_tools::WifiDataConstPtr& wifi_msg);
  bool outputFile(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
  message_filters::Subscriber<wifi_tools::WifiData>* wifiscan_sub_;
  tf::MessageFilter<wifi_tools::WifiData>* wifi_filter_;

  ros::NodeHandle nh_;
  tf::TransformListener* tf_;

  // frame_id for odom
  std::string odom_frame_id_;
  // frame_id for base
  std::string base_frame_id_;

  // topic of wifi_scan
  std::string wifiscan_topic_;

  // AP map
  std::map<std::string,pcl::PointCloud<pcl::PointXYZ> > ap_map_;

  // wifi map publisher
  ros::Publisher wifi_map_pub_;

  ros::ServiceServer out2file_srv_;
};



WiFiMappingNode::WiFiMappingNode()
{
  nh_.param("wifiscan_topic",wifiscan_topic_,std::string("/wifi_tools/wifi_data"));
  nh_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
  nh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));

  tf_ = new tf::TransformListener();

  // wifiscan subscriber
  wifiscan_sub_ = new message_filters::Subscriber<wifi_tools::WifiData>(nh_, wifiscan_topic_, 100);

  // Mesage integration
  wifi_filter_     =  new tf::MessageFilter<wifi_tools::WifiData>(*wifiscan_sub_, *tf_, odom_frame_id_, 100);
  wifi_filter_->registerCallback(boost::bind(&WiFiMappingNode::wifiScanReceived,this, _1));

  // wifi map publisher
  wifi_map_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/wifi_map", 10);

  // output file service
  out2file_srv_ = nh_.advertiseService("/wifi_mapping/o2file", &WiFiMappingNode::outputFile,this);

}



WiFiMappingNode::~WiFiMappingNode()
{
}



bool WiFiMappingNode::outputFile(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  // outpu2 pcd file
  std::string file_base = ros::package::getPath("wifi_tools") + "/map/";
  std::map<std::string,pcl::PointCloud<pcl::PointXYZ> >::iterator it = ap_map_.begin();
  while(it != ap_map_.end())
  {
    std::string filename = file_base + (std::string)(*it).first + ".pcd";
    pcl::io::savePCDFileASCII (filename, (*it).second);
    ++it;
  }
  ROS_INFO("WiFi maps have written to %s",file_base.c_str());
  return true;
}



void WiFiMappingNode::wifiScanReceived(const wifi_tools::WifiDataConstPtr& wifi_msg)
{
  // *** tf pose associated with this massage ***
  tf::Stamped<tf::Pose> odom_pose;
  tf::Stamped<tf::Pose> base_pose (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)), wifi_msg->header.stamp, base_frame_id_);
  try
  {
    this->tf_->transformPose(odom_frame_id_, base_pose, odom_pose);
  }
  catch(tf::TransformException e)
  {
          ROS_WARN("can't get transform. (%s)", e.what());
  }

  // wifi_msg
  std::map<std::string,pcl::PointCloud<pcl::PointXYZ> >::iterator it;
  std::string mac;
  double z_x,z_y,z_ss;
  for(int i=0; i<wifi_msg->data.size();i++)
  {
    mac = wifi_msg->data[i].mac_address;
    it = ap_map_.find(mac);
    z_x = (double)odom_pose.getOrigin().x();
    z_y = (double)odom_pose.getOrigin().y();
    z_ss = (double)wifi_msg->data[i].ss;
    if( it == ap_map_.end())
    {
      // first ap observation
      pcl::PointCloud<pcl::PointXYZ> ap;
      ap_map_.insert(std::map<std::string,pcl::PointCloud<pcl::PointXYZ> >::value_type(mac,ap));
      ROS_INFO("new ap %s",mac.c_str());
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.width    = 1;
      cloud.height   = 1;
      cloud.is_dense = false;
      cloud.points.resize (cloud.width * cloud.height);
      cloud.points[0].x = z_x;
      cloud.points[0].y = z_y;
      cloud.points[0].z = z_ss;
      (*it).second += cloud;
    }
  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "WiFi_mapping_node");
  ros::NodeHandle nh;
  WiFiMappingNode wifi_mapping;

  ROS_INFO("Waiting for WiFi signals...");
  ros::spin();
  return (0);
}
