#pragma once
#include <iostream>
#include<cstdio>
#include <process.h>
#include <conio.h>
#include "string.h"
#include <map>

#include "MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define DEVICE_NUMBER 0 
#define LEFT_CAMERA_IP4 207
#define RIGHT_CAMERA_IP4 122

using namespace std;

class StereoCamera {
public:
	StereoCamera();// 构造函数
	void OpenDoubleCamera();
	void GrabImageDoubleCamera();
	int PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);// 打印设备信息
	~StereoCamera();// 析构函数
	cv::Mat left_img;
	cv::Mat right_img;
private:
	// 左相机句柄
	void* handle_left = NULL;
	// 右相机句柄
	void* handle_right = NULL;
	// 相机列表
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	map<int, int> camera_ip_map;
	int left_idx;
	int right_idx;
	unsigned char *left_pData;
	unsigned char *right_pData;
	unsigned int g_nPayloadSize = 0;
	MV_FRAME_OUT_INFO_EX stImageInfo_left;
	MV_FRAME_OUT_INFO_EX stImageInfo_right;
};
int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight);
// convert data stream in Mat format
bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char * pData, cv::Mat &srcImage);
