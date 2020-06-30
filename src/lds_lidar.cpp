#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>

/**
 *@brief: For callback use only
**/
LdsLidar* g_lidars = nullptr;

/**
* @brief: lidars instance define
**/
LdsLidar::LdsLidar() 
{
}

LdsLidar::~LdsLidar() {
}

/**
* @brief: register lidars connection func and getData func
* @state: ip for PC is 192.168.1.X
          if X is 1, then lidar ip must be 192.168.1.11 - 192.168.1.80
          if X is 2, then lidar ip must be 192.168.1.81 - 192.168.1.150
          if X is 3, then lidar ip must be 192.168.1.151 - 192.168.1.220
* @warn:  if using stereocam, ip must be 192.168.2.X, diff from lidar, 
          and more, ip must bind to phythic address
          when ip for pc has been set, PC will serch submodule for lidar,
          user should not interrupt with this serch.
**/
int LdsLidar::InitLdsLidar(std::vector<std::string>& ip_strs, 
  std::vector<uint32_t>& interp_times, std::vector<std::string>& shared_paths) 
{
  if(ip_strs.size() != interp_times.size()) return -1;
  if(interp_times.size() != shared_paths.size()) return -1;
  if(ip_strs.size() > kMaxLidarCount) return -1;
  device_number = ip_strs.size();
  m_ip_strs.resize(device_number);
  m_interp_times.resize(device_number);
  m_shared_paths.resize(device_number);
  copy(ip_strs.begin(), ip_strs.end(), m_ip_strs.begin());
  copy(interp_times.begin(), interp_times.end(), m_interp_times.begin());
  copy(shared_paths.begin(), shared_paths.end(), m_shared_paths.begin());

  // Livox-SDK init fail!
  if (!Init()) {
    Uninit();
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i = 0; i < kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount;
    lidars_[i].connect_state = kConnectStateOff;
    lidars_[i].interp_time = 0;
    lidars_[i].length.clear();
    lidars_[i].datas.clear();
    lidars_[i].dcount = 0;
    lidars_[i].dlength = 0;
    lidars_[i].data_shared = nullptr;
  }

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  // Start livox sdk to receive lidar data
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail! existing.\n");
    return -1;
  }

  // Add here, only for callback use
  if (g_lidars == nullptr) {
    g_lidars = this;
  }

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) 
{
  for (uint32_t i = 0; i < kMaxLidarCount; i++) {
    lidars_[i].connect_state = kConnectStateOff;
    lidars_[i].interp_time = 0;
    lidars_[i].length.clear();
    lidars_[i].datas.clear();
    lidars_[i].dcount = 0;
    lidars_[i].dlength = 0;
    if(kMaxLidarCount != lidars_[i].handle){
      delete lidars_[i].data_shared;
      lidars_[i].data_shared = nullptr;
    }
  }

  Uninit();

  return 0;
}

/** 
 *@brief: Receiving point cloud data from Livox LiDAR
          Parsing the timestamp and the point cloud data
          100 points in 1 ms, so data_num must be 100
**/
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                              uint32_t data_num, void *client_data) 
{

  if (!data || !data_num || (handle >= kMaxLidarCount)) return;

  LdsLidar* lidar_this = static_cast<LdsLidar *>(client_data);

  LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
  lidar_this->lidars_[handle].length.push_back(data_num);
  for(int i = 0; i < data_num; i++)
  {
    lidar_this->lidars_[handle].datas.push_back(p_point_data[i].x);
    lidar_this->lidars_[handle].datas.push_back(p_point_data[i].y);
    lidar_this->lidars_[handle].datas.push_back(p_point_data[i].z);
    lidar_this->lidars_[handle].datas.push_back(p_point_data[i].reflectivity);
  }
  lidar_this->lidars_[handle].dcount ++;
  lidar_this->lidars_[handle].dlength += data_num;

  if (lidar_this->lidars_[handle].dcount % lidar_this->lidars_[handle].interp_time == 0) {

    lidar_this->lidars_[handle].curt_time = *((uint64_t *)(data->timestamp));
    float time_diff = (lidar_this->lidars_[handle].curt_time - lidar_this->lidars_[handle].last_time) / 1000000.;
    printf("id: %d, timediff: %f, length: %d\n", handle, time_diff, lidar_this->lidars_[handle].dlength);

    // here we share data or send data to server
    uint32_t write_length = 0;
    if(lidar_this->lidars_[handle].dlength > lidar_this->lidars_[handle].interp_time * 100)
      write_length = lidar_this->lidars_[handle].interp_time * 100;
    else
      write_length = lidar_this->lidars_[handle].dlength;

    lidar_this->lidars_[handle].data_shared->write_data(
      lidar_this->lidars_[handle].datas.data(), write_length);

    // clear all data if data is full
    lidar_this->lidars_[handle].length.clear();
    lidar_this->lidars_[handle].datas.clear();
    lidar_this->lidars_[handle].dcount = 0;
    lidar_this->lidars_[handle].dlength = 0;

    lidar_this->lidars_[handle].last_time = lidar_this->lidars_[handle].curt_time;
  }
}

/**
* @brief: p_lidar configs, ini once in common
* @callback: when this func is called, GetLidarDataCb will be called
**/
void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) 
{
  if (info == nullptr || info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s. existing.\n", info->broadcast_code);
    return;
  }
  // check whether ip is added into user_list
  bool found = false;
  for(int i = 0; i < g_lidars->device_number; i++)
    if(0 == strcmp(info->ip, g_lidars->m_ip_strs[i].c_str())) { found = true; }
  if(!found) return;

  // broadcast to localweb, found device ip, and check with user ip 
  bool result = false; uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) 
  {
    SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
    p_lidar->config.enable_fan = true;
    p_lidar->config.return_mode = kStrongestReturn;
    p_lidar->config.coordinate = kCoordinateCartesian;
    p_lidar->config.imu_rate = kImuFreq200Hz;
    for(int i = 0; i < g_lidars->device_number; i++)
    {
      if(0 == strcmp(info->ip, g_lidars->m_ip_strs[i].c_str()))
      {
        p_lidar->interp_time = g_lidars->m_interp_times[i];
        p_lidar->data_shared = new SharedMem(
          g_lidars->m_shared_paths[i], g_lidars->m_interp_times[i] * 100);
        break;
      }
    }    
  } 
  else 
  {
    printf("Add lidar to Connect Failed : %s \n", info->ip);
  }
}

/**
* @brief: Callback function of changing of device state
**/
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) 
{
  if (info == nullptr || info->handle >= kMaxLidarCount) return;

  LidarDevice* p_lidar = &(g_lidars->lidars_[info->handle]);
  if (type == kEventConnect) {
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  }
  else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
  }
  else if (type == kEventStateChange) {
    p_lidar->info = *info;
  }

  // Config lidar parameter 
  if (p_lidar->connect_state == kConnectStateOn) 
  {
    if (p_lidar->info.state == kLidarStateNormal) {
      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(info->handle, LdsLidar::SetCoordinateCb, g_lidars);
      } else {
        SetCartesianCoordinate(info->handle, LdsLidar::SetCoordinateCb, g_lidars);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(info->handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
                                     LdsLidar::SetPointCloudReturnModeCb, g_lidars);
        p_lidar->config.set_bits |= kConfigReturnMode;

        LidarSetImuPushFrequency(info->handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/**
* @brief: set PointCloud return mode
**/
void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, 
                                         uint8_t response, void *client_data) 
{
  if (handle >= kMaxLidarCount) return;

  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) 
  {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    printf("Set return mode success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } 
  else 
  {
    LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                 LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
    printf("Set return mode fail, try again!\n");
  }
}

/**
* @brief: set original coordinates based on laser location
**/
void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, 
                               uint8_t response, void *client_data) 
{
  if (handle >= kMaxLidarCount) return;

  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) 
  {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    printf("Set coordinate success!\n");

    if (!p_lidar->config.set_bits) {
       LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
       p_lidar->connect_state = kConnectStateSampling;
    }
  } 
  else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    }
    printf("Set coordinate fail, try again!\n");
  }
}

/**
* @brief: set imu frequence if imu exists
**/
void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, 
                                         uint8_t response, void *client_data) 
{

  if (handle >= kMaxLidarCount) return;

  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) 
  {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    printf("Set imu rate success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } 
  else
  {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                             LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** 
 *@brief: Callback function of starting sampling.
 *@callback: called in SetImuRatePushFrequencyCb, SetCoordinateCb, SetPointCloudReturnModeCb
**/
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, 
  uint8_t response, void *client_data) 
{
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) return;

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) p_lidar->connect_state = kConnectStateOn;
  }
  else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n", 
           status, handle, response);
  }
}

/** 
 *@brief: Callback function of stopping sampling.
**/
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, 
  uint8_t response, void *client_data) 
{
}
