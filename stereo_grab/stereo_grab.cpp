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
Mat M1(3, 3, CV_64FC1), D1(1, 14, CV_64FC1), M2(3, 3, CV_64FC1), D2(1, 14, CV_64FC1);
Mat R(3, 3, CV_64FC1), T(3, 1, CV_64FC1), R1(3, 3, CV_64FC1), P1(3, 4, CV_64FC1), R2(3, 3, CV_64FC1), P2(3, 4, CV_64FC1);
Mat Q(4, 4, CV_64FC1), affine_R, affine_T;
vector<double> rectify_param(30);
void match_cp3(vector<Point2f> &target_l_points, vector<Point2f> &target_r_points, Mat &Q, Mat &avg);
int find_cp3_point(Mat& img, vector<Point2f> &target_point);
void load_param();
void rectify_point(Mat &point);

int main() {
	StereoCamera Fine_Tune_Device;
	Fine_Tune_Device.OpenDoubleCamera(MV_GAIN_MODE_OFF);
	int b = 1;
	cout << "1 继续, 0 退出" << endl;
	while (cin >> b,b) {
		Mat average = Mat::zeros(3, 1, CV_64FC1);
		ofstream outfile("xiangxi.txt", ios::app);
		int count = 0;
		while(count< EPOCH_NUM){
			Fine_Tune_Device.GrabImageDoubleCamera();
			Size img_size = Fine_Tune_Device.left_img.size();
			// 读取内参和外参
			load_param();
			// 矫正
			Mat map11, map12, map21, map22;
			Mat img1r, img2r;
			initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

			remap(Fine_Tune_Device.left_img, img1r, map11, map12, INTER_LINEAR);
			remap(Fine_Tune_Device.right_img, img2r, map21, map22, INTER_LINEAR);

			vector<Point2f> target_r_points, target_l_points;
			// 右目拍摄 的 右图 
			int ret1 = find_cp3_point(img1r, target_r_points);
			// 左目拍摄 的 左图
			int ret2 = find_cp3_point(img2r, target_l_points);
			if (!(ret1&ret2)) {
				cout << "没有得到完整的cp3点,请移动到合适位置 "<<endl;
				continue;
			}
			count++;
			Mat result(4, 1, CV_64FC1);
			match_cp3(target_l_points, target_r_points, Q, result);
			//std::cout << result << endl;
			//保存测量前的坐标
			//outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0) << endl;
			//average += result;
			Mat get_point = result.rowRange(0, 3);
			//std::cout << get_point << endl;
			Mat predict_point = (affine_R * get_point + affine_T);
			//std::cout << predict_point << endl;

			 /*添加 修正之后的参数*/
			rectify_point(predict_point);

			cout << predict_point << endl;
			average += predict_point;
			//保存测量后的坐标
			outfile << predict_point.at<double>(0, 0) << " " << predict_point.at<double>(1, 0) << " " << predict_point.at<double>(2, 0) << endl;

		}
		ofstream avgout("pingjun.txt", ios::app);
		average /= 5;
		avgout << average.at<double>(0, 0) << " " << average.at<double>(1, 0) << " " << average.at<double>(2, 0) << endl;
		cout << "1 继续, 0 退出" << endl;
	}
	return 0;
}

void load_param() {
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", "intrinsics.yml");
		exit(1);
	}

	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open("extrinsics.yml", FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", "extrinsics.yml");
		exit(1);
	}
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["P1"] >> P1;
	fs["R2"] >> R2;
	fs["P2"] >> P2;
	fs["Q"] >> Q;

	fs.open("coodinate.yml", FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", "coodinate.ymll");
		exit(1);
	}
	fs["R"] >> affine_R;
	fs["T"] >> affine_T;
	rectify_param.clear();
	ifstream rec_params("rectify.txt");
	string line;
	while (getline(rec_params, line))
	{
		istringstream ss(line); // 建立ss与line之间的关联
		double t;
		while (ss >> t) { rectify_param.push_back(t);}
	}
}

// 找到cp3点的目标靶坐标
int  find_cp3_point(Mat& img, vector<Point2f> &target_point) {
	bool found;
	vector<Point2f> corners;
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 2e5;
	params.blobColor = 255;
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	found = findCirclesGrid(img, Size(2, 2), corners, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
	target_point = corners;
	if (!found) {
		//cout << "not find point" << endl;
		return 0;
	}
	else {
		return 1;
	}
}

void match_cp3(vector<Point2f> &target_l_points, vector<Point2f> &target_r_points, Mat &Q, Mat &avg) {
	assert(target_l_points.size() == target_r_points.size());
	avg = Mat::zeros(4, 1, CV_64FC1);
	for (int i = 0; i < target_l_points.size(); i++) {
		double distance_l = _ABS(target_r_points[i].x - target_l_points[i].x);
		Mat temp(4, 1, CV_64FC1);
		//cout << target_r_points[i].x << ","<< target_r_points[i].y << endl;
		temp.at<double>(0, 0) = target_r_points[i].x;
		temp.at<double>(1, 0) = target_r_points[i].y;
		temp.at<double>(2, 0) = distance_l;
		temp.at<double>(3, 0) = 1.;
		Mat result = (Q * temp);
		double W = result.at<double>(3, 0);
		result /= W;
		result.at<double>(3, 0) = distance_l;
		avg += result;
	}
	avg /= target_l_points.size();
}

void rectify_point(Mat &point) {
	if (rectify_param.size() % 3 != 0 ) {
		cout << "参数错误,修正失败" << endl;
		return;
	}
	double x = point.at<double>(0, 0), y = point.at<double>(1, 0), z = point.at<double>(2, 0);

	int param_size = rectify_param.size();
	// stride  是  10  次多项式   一共11个参数
	int stride = param_size / 3;
	double temp_x =0, temp_y =0, temp_z = 0;
	for (int j = stride; j > 0; j--) {
		double p_x = rectify_param[stride - j];
		double p_y = rectify_param[stride*2 - j];
		double p_z = rectify_param[stride*3 - j];
		int i = j;
		while (--i>0) {
			p_x *= z;
			p_y *= z;
			p_z *= z;
		}
		temp_x += p_x;
		temp_y += p_y;
		temp_z += p_z;
	}
	point.at<double>(0, 0) = temp_x + x;
	point.at<double>(1, 0) = temp_y + y;
	point.at<double>(2, 0) = temp_z;
}