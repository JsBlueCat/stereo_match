#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;

int main() {
	Mat get_point = (Mat_<double>(20, 3) << 23.6113, 50.9298, 2479.3699, 73.2837, 51.2914, 2479.5601, 122.9108, 51.6483, 2480.0381, 172.4878, 52.0556, 2480.6240, 222.0678, 52.4492, 2480.8979, 24.3092, -48.1941, 2485.2141, 73.9784, -47.7924, 2485.3101, 123.6220, -47.3512, 2485.6379, 173.2020, -46.9352, 2485.9019, 222.8058, -46.5280, 2486.2520, 17.1316, 59.5219, 2975.0620, 66.6411, 59.9738, 2975.4438, 116.2056, 60.3397, 2975.8560, 165.5872, 60.8220, 2976.4661, 215.0110, 61.2792, 2976.8481, 17.9412, -39.3385, 2980.9580, 67.4578, -38.8760, 2981.4121, 116.9714, -38.3805, 2981.6841, 166.4692, -37.9339, 2981.7222, 215.8590, -37.4508, 2982.3999);
	Mat real_point = (Mat_<double>(20, 3) << -100.0000, 0.0000, 995.0000, -50.0000, 0.0000, 995.0000, 0.0000, 0.0000, 995.0000, 50.0000, 0.0000, 995.0000, 100.0000, 0.0000, 995.0000, -100.0000, 100.0000, 995.0000, -50.0000, 100.0000, 995.0000, 0.0000, 100.0000, 995.0000, 50.0000, 100.0000, 995.0000, 100.0000, 100.0000, 995.0000, -100.0000, 0.0000, 1495.0000, -50.0000, 0.0000, 1495.0000, 0.0000, 0.0000, 1495.0000, 50.0000, 0.0000, 1495.0000, 100.0000, 0.0000, 1495.0000, -100.0000, 100.0000, 1495.0000, -50.0000, 100.0000, 1495.0000, 0.0000, 100.0000, 1495.0000, 50.0000, 100.0000, 1495.0000, 100.0000, 100.0000, 1495.0000);
	Mat affine, liner,affine_R,affine_T,estimate_M;
	estimateAffine3D(get_point, real_point, affine, liner,3,0.99);
	affine_R = affine(Rect(0, 0, 3, 3));
	affine_T = affine.colRange(3, 4);
	cout << affine_R << affine_T;

	FileStorage fs("coodinate.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << affine_R << "T" << affine_T;
		fs.release();
	}
	
	ofstream outfile("predict.txt", ios::app);
	outfile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
	outfile.precision(2);
	for (int i = 0; i < 20;  i++) {
		Mat test = get_point.row(i);
		Mat result = (affine_R * test.t() + affine_T);
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0)<< endl;
	}
	

	
	cin.get();
	return 0;
}