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
// Gaussian Process
#include "gaussian_process/SingleGP.h"



class WifiGpmapNode
{
public:
  WifiGpmapNode();
  ~WifiGpmapNode();
  void loadmap();
  void evalGaussian();
  void publish();
private:
  ros::NodeHandle nh_;

  // size of boxel grid filter
  float box_x_,box_y_,box_z_;

  // signal strength map
  std::map<std::string,sensor_msgs::PointCloud2> ap_raw_map_;
  std::map<std::string,sensor_msgs::PointCloud2> gp_mean_map_;
  std::map<std::string,sensor_msgs::PointCloud2> gp_cov_map_;

  // wifi gpmap publisher
  std::vector<ros::Publisher> wifi_gpmean_pub_;
  std::vector<ros::Publisher> wifi_gpcov_pub_;

  // GP map parameter
  double gp_x_min_, gp_x_max_,gp_x_interval_, gp_y_min_, gp_y_max_,gp_y_interval_;
  std::string path_wifimap_;
};



WifiGpmapNode::WifiGpmapNode() :
    nh_("~")
{
  nh_.param("gp_x_min", gp_x_min_, 0.0);
  nh_.param("gp_x_max", gp_x_max_, 20.0);
  nh_.param("gp_x_interval", gp_x_interval_, 1.0);
  nh_.param("gp_y_min", gp_y_min_, 0.0);
  nh_.param("gp_y_max", gp_y_max_, 20.0);
  nh_.param("gp_y_interval", gp_y_interval_, 1.0);
  nh_.param("path_wifimap",path_wifimap_,ros::package::getPath("wifi_tools") + "/map");


  // size of voxel grid filter
  box_x_ = 1.0;
  box_y_ = 1.0;
  box_z_ = 256.0;

  // load Wifi Map
  loadmap();

  // Mac address map publisher
  std::map<std::string,sensor_msgs::PointCloud2>::iterator it;
  it = ap_raw_map_.begin();
  while(it != ap_raw_map_.end())
  {
    std::ostringstream mean_topic,cov_topic;
    mean_topic << "wifi_gpmean" << (*it).first;
    cov_topic << "wifi_gpcov" << (*it).first;
    std::string t1 = boost::algorithm::replace_all_copy(mean_topic.str(),":","");
    std::string t2 = boost::algorithm::replace_all_copy(cov_topic.str(),":","");
    ros::Publisher each_mean_pub = nh_.advertise<sensor_msgs::PointCloud2>(t1.c_str(), 1);
    ros::Publisher each_cov_pub = nh_.advertise<sensor_msgs::PointCloud2>(t2.c_str(), 1);
    wifi_gpmean_pub_.push_back(each_mean_pub);
    wifi_gpcov_pub_.push_back(each_cov_pub);
    ++it;
  }
}



WifiGpmapNode::~WifiGpmapNode()
{
}



void WifiGpmapNode::loadmap()
{
  int ap_count = 0;
  namespace f = boost::filesystem;
  f::path map_dir = path_wifimap_;
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
      ap_raw_map_.insert(std::map<std::string,sensor_msgs::PointCloud2 >::value_type(mac_address,*cloud_msg));
      ap_count++;
    }
  }
  ROS_INFO("%d access points data have loaded.",ap_count);
}



void WifiGpmapNode::evalGaussian()
{
  std::map<std::string,sensor_msgs::PointCloud2>::iterator it;
  it = ap_raw_map_.begin();
  while(it != ap_raw_map_.end())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr wifi_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg((*it).second,*wifi_pcl);

    std::vector<gaussian_process::SingleGP*> gp;
    size_t training_samples;

    // Initialize GP
    CovFuncND initialCovFunc;
    double initialSigmaNoise;

    // GP parameter
    vector<double> params = vector<double>(3);
    params[0] = 0.5;
    params[1] = 0.5;
    params[2] = 0.0;
    initialCovFunc.setHyperparameter(params);
    initialSigmaNoise = -5;
    for (size_t i = 0; i < 7; i++) {
      gp.push_back(   new gaussian_process::SingleGP(initialCovFunc,initialSigmaNoise));
    }

    // GP learn
    ROS_INFO("Learning gaussian process model... %s",(*it).first.c_str());
    int training_size = wifi_pcl->points.size();
    TVector < TDoubleVector > input(training_size);
    TVector<double> output[7];
    for (size_t j = 0; j < 7; j++) {
            output[j] = TDoubleVector(training_size);
    }

    for (size_t i = 0; i < training_size; i++) {
      input[i] = TDoubleVector(2);
      input[i][0] = wifi_pcl->points[i].x;
      input[i][1] = wifi_pcl->points[i].y;
      output[0][i] = wifi_pcl->points[i].z;
      output[1][i] = 0;
      output[2][i] = 0;
      output[3][i] = 0;
      output[4][i] = 0;
      output[5][i] = 0;
      output[6][i] = 0;
    }
    for(size_t j=0;j<7;j++) {
            gp[j]->SetData(input,output[j]);
    }

    // GP Evaluate
    ROS_INFO("Evaluating gaussian process model... %s",(*it).first.c_str());
    TDoubleVector inp(2);
    double mean_ss=0,var_ss=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mean_cloud (new pcl::PointCloud<pcl::PointXYZ>),cov_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(double i=gp_x_min_;i<gp_x_max_;i+=gp_x_interval_)
    {
      for(double j=gp_y_min_;j<gp_y_max_;j+=gp_y_interval_)
      {
        pcl::PointCloud<pcl::PointXYZ> p1,p2;
          p1.width = p2.width = 1;
          p1.height = p2.height = 1;
          p1.is_dense = p2.is_dense = false;
          p1.points.resize (p1.width * p1.height);
          p2.points.resize (p2.width * p2.height);
          p1.points[0].x = p2.points[0].x = i;
          p1.points[0].y = p2.points[0].y = j;
          inp(0) = i;
          inp(1) = j;
          gp[0]->Evaluate( inp, mean_ss, var_ss );
          p1.points[0].z = mean_ss;
          p2.points[0].z = var_ss;
          (*mean_cloud) += p1;
          (*cov_cloud) += p2;
      }
    }
    sensor_msgs::PointCloud2::Ptr gp_mean_msg(new sensor_msgs::PointCloud2()),gp_cov_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*mean_cloud, *gp_mean_msg);
    pcl::toROSMsg(*cov_cloud, *gp_cov_msg);
    gp_mean_msg->header.frame_id = gp_cov_msg->header.frame_id = "/map";
    gp_mean_map_.insert(std::map<std::string,sensor_msgs::PointCloud2 >::value_type((*it).first,*gp_mean_msg));
    gp_cov_map_.insert(std::map<std::string,sensor_msgs::PointCloud2 >::value_type((*it).first,*gp_cov_msg));
    ++it;
  }


}



void WifiGpmapNode::publish()
{
  std::map<std::string,sensor_msgs::PointCloud2>::iterator it1,it2;
  std::vector<ros::Publisher>::iterator pub_it1,pub_it2;
  it1 = gp_mean_map_.begin();
  it2 = gp_cov_map_.begin();
  pub_it1 = wifi_gpmean_pub_.begin();
  pub_it2 = wifi_gpcov_pub_.begin();
  while(it1 != gp_mean_map_.end())
  {
    (*pub_it1).publish((*it1).second);
    (*pub_it2).publish((*it2).second);
    ++pub_it1;++pub_it2;
    ++it1;++it2;
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_gpmap");
  ros::NodeHandle nh;
  WifiGpmapNode wifi_gpmap;
  wifi_gpmap.evalGaussian();
  ros::Rate rate_Hz(1);
  ROS_INFO("Start publish.");
  while (ros::ok())
  {
    wifi_gpmap.publish();
    rate_Hz.sleep();
  }
  return (0);
}
