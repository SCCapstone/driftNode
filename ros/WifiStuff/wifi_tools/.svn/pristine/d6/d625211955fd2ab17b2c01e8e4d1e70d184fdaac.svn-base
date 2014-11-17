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



WifiStumbler::WifiStumbler()
{
}



WifiStumbler::~WifiStumbler()
{
  iw_sockets_close(wlan_sock_);
}



wifi_tools::WifiData WifiStumbler::getData()
{
  return wifi_stumble_;
}



void WifiStumbler::setFrame(std::string frame_id)
{
  wifi_stumble_.header.frame_id = frame_id;
}



bool WifiStumbler::initialize(std::string wlan_if)
{
  wlan_if_ = wlan_if;
  wlan_sock_ = iw_sockets_open();
  if(wlan_sock_<0)
  {
   ROS_ERROR("Failed to open wlan socket on %s", wlan_if_.c_str());
   return false;
  }
  return true;
}



bool WifiStumbler::stumble()
{
  int pid;
  pid = getpid();

  struct iwreq w_request;
  w_request.u.data.pointer = (caddr_t)&pid;
  w_request.u.data.length = 0;

  if (iw_set_ext(wlan_sock_, wlan_if_.c_str(), SIOCSIWSCAN, &w_request) < 0)
  {
    ROS_ERROR("Couldn't start stumbler.");
    return false;
  }

  timeval time;
  time.tv_sec = 0;
  time.tv_usec = 200000;

  uint8_t *p_buff = NULL;
  int buff_size = IW_SCAN_MAX_DATA;

  bool is_end = false;
  while(!is_end)
  {
    fd_set fds;
    int ret;
    FD_ZERO(&fds);
    ret = select(0, &fds, NULL, NULL, &time);
    if (ret == 0)
    {
      uint8_t *p_buff2;
      while (!is_end)
      {
        p_buff2 = (uint8_t *)realloc(p_buff, buff_size);
        p_buff = p_buff2;
        w_request.u.data.pointer = p_buff;
        w_request.u.data.flags = 0;
        w_request.u.data.length = buff_size;
        if (iw_get_ext(wlan_sock_, wlan_if_.c_str(), SIOCGIWSCAN, &w_request) < 0)
        {
          if (errno == E2BIG && range_.we_version_compiled > 16)
          {
            if (w_request.u.data.length > buff_size)
              buff_size = w_request.u.data.length;
            else
              buff_size *= 2;
            continue;
          }
          else if (errno == EAGAIN)
          {
            time.tv_sec = 0;
            time.tv_usec = 200000;
            break;
          }
        }
        else
          is_end = true;
      }
    }
  }

  // Put wifi data into ROS message
  wifi_tools::AccessPoint ap;
  iw_event event;
  stream_descr stream;

  wifi_stumble_.data.clear();
  wifi_stumble_.header.stamp = ros::Time::now();

  iw_init_event_stream(&stream, (char *)p_buff, w_request.u.data.length);
  is_end = false;
  int value = 0;
  while(!is_end)
  {
    value = iw_extract_event_stream(&stream, &event, range_.we_version_compiled);
    if(!(value>0))
    {
      is_end = true;
    }
    else
    {
      if(event.cmd == IWEVQUAL)
      {
        // quality part of statistics
        //ROS_INFO("command=IWEVQUAL");
        if (event.u.qual.level != 0 || (event.u.qual.updated & (IW_QUAL_DBM | IW_QUAL_RCPI)))
        {
          ap.noise = event.u.qual.noise;
          ap.ss    = event.u.qual.level;
        }
      }
      else if(event.cmd == SIOCGIWAP)
      {
        // get access point MAC addresses
        //ROS_INFO("command=SIOCGIWAP");
        char mac_buff[1024];
        iw_ether_ntop((const struct ether_addr *)&event.u.ap_addr.sa_data,mac_buff);
        ap.mac_address = std::string(mac_buff);
        ROS_INFO("mac_addr=%s",mac_buff);
      }
      else if(event.cmd == SIOCGIWESSID)
      {
        // get ESSID
        //ROS_INFO("command=SIOCGIWESSID");
        wifi_stumble_.data.push_back(ap);
      }
      else if(event.cmd == SIOCGIWENCODE)
      {
        // get encoding token & mode
        //ROS_INFO("command=SIOCGIWENCODE");
      }
      else if(event.cmd == SIOCGIWFREQ)
      {
        // get channel/frequency (Hz)
        //ROS_INFO("command=SIOCGIWFREQ");
      }
      else if(event.cmd == SIOCGIWRATE)
      {
        // get default bit rate (bps)
        //ROS_INFO("command=SIOCGIWRATE");
      }
      else if(event.cmd == SIOCGIWMODE)
      {
        // get operation mode
        //ROS_INFO("command=SIOCGIWMODE");

      }
      else if(event.cmd == IWEVCUSTOM)
      {
        // Driver specific ascii string
        //ROS_INFO("command=IWEVCUSTOM");
      }
      else if(event.cmd == IWEVGENIE)
      {
        // Generic IE (WPA, RSN, WMM, ..)
        //ROS_INFO("command=IWEVGENIE");
      }
      else
      {
        // another command
        //ROS_INFO("another command");
      }
    }
  }

  checkDuplication();

  return true;
}



void WifiStumbler::checkDuplication()
{
  //check duplication of MAC
  std::map<std::string,wifi_tools::AccessPoint> ap_map;
  std::map<std::string,wifi_tools::AccessPoint>::iterator it_map;
  std::vector<wifi_tools::AccessPoint>::iterator it;
  std::string mac;

  it = wifi_stumble_.data.begin();
  while( it != wifi_stumble_.data.end() )
  {
    mac = (std::string)(*it).mac_address;
    if( ap_map.find(mac) == ap_map.end())
    {
      // new access point
      ap_map.insert( std::map<std::string,wifi_tools::AccessPoint>::value_type(mac,*it) );
    }
    else
    {
      // duplicate access point
      //TODO FIFO, or something else?
    }
    ++it;
  }

  wifi_stumble_.data.clear();
  it_map = ap_map.begin();
  while( it_map != ap_map.end() )
  {
    wifi_stumble_.data.push_back(it_map->second);
    ++it_map;
  }
}


