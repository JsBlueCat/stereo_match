#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;


void get_affine_mat(Mat &get_point, Mat &real_point , Mat &affine_R , Mat &affine_T) {
	Mat affine, liner;
	estimateAffine3D(get_point, real_point, affine, liner, 3, 0.95);
	affine_R = affine(Rect(0, 0, 3, 3));
	affine_T = affine.colRange(3, 4);
	cout << liner << endl;
}

void get_coordinate_from_txt(Mat &get_point) {
	ifstream origin_point("origin.txt");
	double x, y, z;
	vector<Point3f> points;
	int flag = 0;
	get_point = Mat(90, 3, CV_64F);
	while (origin_point >> x >> y >> z) {
		get_point.at<double>(flag, 0) = x;
		get_point.at<double>(flag, 1) = y;
		get_point.at<double>(flag, 2) = z;
		flag++;
	};
	print(get_point);
}

void get_real_points_from_txt(Mat &real_points) {
	vector<Point3f> points;
	ifstream real_point("real_points.txt");
	double x, y, z;
	int flag = 0;
	real_points = Mat(90, 3, CV_64F);
	while (real_point >> x >> y >> z) {
		real_points.at<double>(flag, 0) = x;
		real_points.at<double>(flag, 1) = y;
		real_points.at<double>(flag, 2) = z;
		flag++;
	};
	print(real_points);
	//cout << real_points.size();
}

int main() {
	//Mat get_point = (Mat_<double>(30, 3) << );
	//Mat real_point = (Mat_<double>(30, 3) << );
	
	Mat get_point, real_point;
	get_coordinate_from_txt(get_point);
	get_real_points_from_txt(real_point);
	

	Mat affine, liner,affine_R,affine_T,estimate_M;
	get_affine_mat(get_point, real_point, affine_R, affine_T);
	//estimateAffine3D(get_point, real_point, affine, liner, 3 , 0.99);
	//affine_R = affine(Rect(0, 0, 3, 3));
	//affine_T = affine.colRange(3, 4);
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
	for (int i = 0; i < 90;  i++) {
		Mat test = get_point.row(i);
		Mat result = (affine_R * test.t() + affine_T);
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0)<< endl;
	}

	cin.get();
	return 0;
}