// stereo_match.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once
#define _ABS(x) ((x) > 0 ? (x) : -(x))
// 已知坐标点的 个数 
#define KNOWN_POINT_NUMBER 6
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <math.h>
#include<numeric>
#include "hikivision_camera.h"

using namespace cv;
using namespace std;
// TODO: 在此处引用程序需要的其他标头。

// 找到目标点修正的矩阵
void find_rectify_matrix(Mat& img_l, Mat& img_r, Mat&out, Mat&inliers, Mat &Q);
// 找到 已知靶的坐标 
void find_known_point(Mat& img, vector<Point2f> &target_point);
// 找到cp3 的坐标点 
void find_cp3_point(Mat& img,vector<Point2f> &target_point);
// 通过 左右目的 数据 立体匹配
void match_cp3(vector<Point2f>& target_l_points, vector<Point2f>& target_r_points, Mat &Q, Mat &result);
// 立体匹配 已知靶坐标点 
void match_known_point(vector<Point2f> &left_target, vector<Point2f> &right_target, Mat &Q, vector<Vec3f> &out);
