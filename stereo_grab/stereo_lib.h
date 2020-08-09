#pragma once
#ifndef __STEREO_LIB_H__
#define __STEREO_LIB_H__
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#define _ABS(x) ((x) > 0 ? (x) : -(x))
#include<iostream>
#include <stdio.h>
#define EPOCH_NUM 5
#include "hikivision_camera.h"
#include <opencv2/opencv.hpp>

#include <string>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;

typedef enum RESULT_TYPE {
	RETRUN_SUCCESS = 0,
	RETRUN_FAILED,
}RESULT;

RESULT single_shot_and_get_point();

#endif