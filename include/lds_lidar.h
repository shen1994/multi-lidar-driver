/** Livox LiDAR data source, data from dependent lidar */

#ifndef LDS_LIDAR_H_
#define LDS_LIDAR_H_

#include <memory>
#include <vector>
#include <string>

#include "livox_def.h"
#include "livox_sdk.h"
#include "shared_mem.h"

typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateConfig = 2,
  kConnectStateSampling = 3,
} LidarConnectState;

typedef enum {
  kConfigFan = 1,
  kConfigReturnMode = 2,
  kConfigCoordinate = 4,
  kConfigImuRate = 8
} LidarConfigCodeBit;

typedef enum {
  kCoordinateCartesian = 0,
  kCoordinateSpherical
} CoordinateType;

typedef struct {
  bool enable_fan;
  uint32_t return_mode;
  uint32_t coordinate;
  uint32_t imu_rate;
  uint32_t ip_enable;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserConfig;

typedef struct {
  uint8_t handle;
  uint32_t interp_time;
  uint32_t dcount;
  uint32_t dlength;
  std::vector<uint32_t> length;
  std::vector<uint32_t> datas;
  uint64_t curt_time;
  uint64_t last_time;
  SharedMem *data_shared;
  std::string device_id;
  LidarConnectState connect_state;
  DeviceInfo info;
  UserConfig config;
} LidarDevice;

/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar {
 public:

  static LdsLidar& GetInstance() {
    static LdsLidar lds_lidar;
    return lds_lidar;
  }

  int InitLdsLidar(std::vector<std::string>& ip_strs, 
    std::vector<uint32_t>& interp_times, std::vector<std::string>& shared_paths);
  int DeInitLdsLidar(void);

 private:
  LdsLidar();
  LdsLidar(const LdsLidar&) = delete;
  ~LdsLidar();
  LdsLidar& operator=(const LdsLidar&) = delete;

  static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,\
                             uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                        uint8_t response, void *clent_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle, \
                              uint8_t response, void *clent_data);
  static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                        uint8_t response, void *clent_data);

  uint32_t device_number;
  std::vector<std::string> m_ip_strs; 
  std::vector<uint32_t> m_interp_times;
  std::vector<std::string> m_shared_paths;
  LidarDevice lidars_[kMaxLidarCount];
};

#endif
