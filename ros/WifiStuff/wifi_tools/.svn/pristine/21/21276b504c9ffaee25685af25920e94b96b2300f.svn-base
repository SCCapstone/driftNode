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
#include "sensor_msgs/PointCloud2.h"

// PCL
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

// boost
#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"
#include "boost/tokenizer.hpp"
#include "boost/algorithm/string.hpp"

class WifiView
{
public:
  WifiView();
  ~WifiView();
  void load();
  void run();

private:
  ros::NodeHandle nh_;

  // size of boxel grid filter
  float box_x_,box_y_,box_z_;

  // signal strength map
  std::map<std::string,sensor_msgs::PointCloud2> ap_map_;

  std::vector<ros::Publisher> wifi_map_pub_;

};



WifiView::WifiView()
{
  // size of boxel grid filter
  box_x_ = 1.0;
  box_y_ = 1.0;
  box_z_ = 256.0;

  // load Wifi Map
  load();

  // Mac address map publisher
  std::map<std::string,sensor_msgs::PointCloud2>::iterator it;

  it = ap_map_.begin();

  while(it != ap_map_.end())
  {
    std::ostringstream topic;
    topic << "wifimap" << (*it).first;
    std::string t = boost::algorithm::replace_all_copy(topic.str(),":","");
    ros::Publisher each_map_pub = nh_.advertise<sensor_msgs::PointCloud2>(t.c_str(), 1);
    wifi_map_pub_.push_back(each_map_pub);
    ++it;
  }
}



WifiView::~WifiView()
{

}



void WifiView::load()
{
  int ap_count = 0;
  namespace f = boost::filesystem;
  f::path map_dir = ros::package::getPath("wifi_tools") + "/map";
  f::directory_iterator end;
  for( f::directory_iterator it(map_dir); it!=end; ++it )
  {
    if( !(f::is_directory(*it)) )
    {
      //mac address
      std::string leaf = (*it).path().leaf().c_str();
      boost::char_separator< char > sep(".");
      boost::tokenizer< boost::char_separator< char > > tokens(leaf, sep);
      boost::tokenizer< boost::char_separator< char > >::iterator file_it;
      file_it = tokens.begin();
      std::string mac_address = (std::string)(*file_it);

      // load WiFi data
      std::ifstream ap_ifs((*it).path().c_str());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile<pcl::PointXYZ> ((*it).path().c_str(), *cloud);

      // boxel grid filter
      pcl::VoxelGrid<pcl::PointXYZ> box;
      box.setInputCloud (cloud);
      box.setLeafSize (box_x_,box_y_,box_z_);
      box.filter (*cloud_filtered);

      // cache data
      sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud_filtered, *cloud_msg);
      (*cloud_msg).header.frame_id = "/map";
      (*cloud_msg).header.stamp = ros::Time::now();
      ap_map_.insert(std::map<std::string,sensor_msgs::PointCloud2 >::value_type(mac_address,*cloud_msg));
      ap_count++;
    }
  }
  ROS_INFO("%d access points data have loaded.",ap_count);
}



void WifiView::run()
{
  std::map<std::string,sensor_msgs::PointCloud2>::iterator it;
  std::vector<ros::Publisher>::iterator pub_it;
  it = ap_map_.begin();
  pub_it = wifi_map_pub_.begin();
  while(it != ap_map_.end())
  {
    (*pub_it).publish((*it).second);
    ++pub_it;
    ++it;
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_viewer");
  ros::NodeHandle nh;
  WifiView wifi_map;

  ros::Rate rate_Hz(1);
  while (ros::ok())
  {
    wifi_map.run();
    rate_Hz.sleep();
  }
  return (0);
}
