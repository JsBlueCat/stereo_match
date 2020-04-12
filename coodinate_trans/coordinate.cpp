#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;

int main() {
	Mat get_point = (Mat_<double>(14, 3) << 7.6444, 40.1062, 2722.9480, 58.0655, 39.6773, 2723.4180, 108.2646, 38.7833, 2723.6802, 158.3820, 37.7950, 2723.9041, 208.8638, 37.0357, 2724.4780, 107.1442, -11.6555, 2723.2961, 108.8318, 88.7318, 2723.9819, 18.6028, 32.7876, 2104.5962, 68.8361, 32.4976, 2105.2300, 118.8906, 31.7185, 2105.6479, 168.8810, 30.8519, 2106.0259, 219.2198, 30.2621, 2106.7922, 117.8832, -18.5994, 2105.0139, 119.3460, 81.4335, 2106.1699);
	Mat real_point = (Mat_<double>(14, 3) << -100, -2, 2745, -50, -2, 2745, 0, -2, 2745, 50, -2, 2745, 100, -2, 2745, 0, 48, 2745, 0, -52, 2745, -100, -2, 2143, -50, -2, 2143, 0, -2, 2143, 50, -2, 2143, 100, -2, 2143, 0, 48, 2143, 0, -52, 2143);
	Mat affine, liner,affine_R,affine_T,estimate_M;
	estimateAffine3D(get_point, real_point, affine, liner, 3 , 0.99);
	affine_R = affine(Rect(0, 0, 3, 3));
	affine_T = affine.colRange(3, 4);
	//cout << affine_R << affine_T;
	cout << liner << endl;
	FileStorage fs("coodinate.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << affine_R << "T" << affine_T;
		fs.release();
	}
	
	ofstream outfile("predict.txt", ios::app);
	outfile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
	outfile.precision(2);
	for (int i = 0; i < 14;  i++) {
		Mat test = get_point.row(i);
		Mat result = (affine_R * test.t() + affine_T);
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0)<< endl;
	}

	cin.get();
	return 0;
}