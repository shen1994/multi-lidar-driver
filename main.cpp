#include <iostream>
#include <string>
#include <vector>
#include "common.h"
#include "lds_lidar.h"

int main(int argc, char *argv[])
{
  std::vector<std::string> ip_code;
  std::vector<uint32_t> interp_time;
  std::vector<std::string> shared_path;
  ip_code.push_back("192.168.1.12");
  interp_time.push_back(100);
  shared_path.push_back("/cv/LidarData");

  LdsLidar& read_lidar = LdsLidar::GetInstance();

  int ret = read_lidar.InitLdsLidar(ip_code, interp_time, shared_path);
  if (!ret) 
    printf("Init lds lidar success!\n");
  else
    printf("Init lds lidar fail!\n");

  sleep(100);

  read_lidar.DeInitLdsLidar();

	return 0;
}