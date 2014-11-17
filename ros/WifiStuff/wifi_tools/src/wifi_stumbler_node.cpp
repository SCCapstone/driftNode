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

#include "wifi_stumbler.h"

class WifiStumblerNode
{
public:
  WifiStumblerNode();
  ~WifiStumblerNode();
  void run();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher wifi_pub_;

  WifiStumbler stumbler_;
  std::string wlan_if_;         // WLAN interface
  std::string frame_id_;        // frame_id of WiFi adapter
};



WifiStumblerNode::WifiStumblerNode() :
    private_nh_("~")
{
  // parameter set
  private_nh_.param("wlan_if",wlan_if_,std::string("wlan0"));
  private_nh_.param("wlan_flame",frame_id_,std::string("base_link"));

  // initialization
  stumbler_.initialize(wlan_if_);
  stumbler_.setFrame(frame_id_);

  // wifi data publisher
  wifi_pub_ = nh_.advertise<wifi_tools::WifiData>("/wifi_tools/wifi_data", 1);
}



WifiStumblerNode::~WifiStumblerNode()
{
}



void WifiStumblerNode::run()
{
  ros::Rate rate_Hz(1);
  while(ros::ok())
  {
    wifi_tools::WifiData wifi_msg;
    stumbler_.stumble();
    wifi_msg = stumbler_.getData();
    ROS_INFO("Found %d AP.",wifi_msg.data.size());
    wifi_pub_.publish(wifi_msg);
    rate_Hz.sleep();
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_stumbler_node");
  WifiStumblerNode stumbler;
  stumbler.run();
  return(0);
};
