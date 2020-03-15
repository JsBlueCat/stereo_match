#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;

int main() {
	FileStorage fs("coodinate.yml", FileStorage::READ);
	Mat affine_R, affine_T;
	if (fs.isOpened())
	{
		fs["R"] >> affine_R;
		fs["T"] >> affine_T;
	}
	//cout << affine_R << affine_T;
	ifstream ifs("result.txt");
	ofstream outfile("predict.txt", ios::app);
	outfile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
	outfile.precision(2);
	double x, y, z;
	while (ifs >> x >> y >> z)
	{
		Mat point;
		point = (Mat_<double>(1, 3) << x, y, z);
		Mat result = affine_R * point.t() + affine_T;
		cout << result << endl;
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0) << endl;
	}
	cin.get();
	return 0;
}