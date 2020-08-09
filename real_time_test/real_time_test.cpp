#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;
void rectify_point(Mat &point);
void show_img(Mat &img) {
	Mat cimg, cimg1;
	double sf = 640. / MAX(img.rows, img.cols);
	resize(img, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
	imshow("corners", cimg1);
	char c = (char)waitKey(500);
	if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
		exit(-1);
}

void show_find_point(Mat &img, vector<Point2f> &corners) {
	Mat cimg, cimg1;
	cvtColor(img, cimg, COLOR_GRAY2BGR);
	drawChessboardCorners(cimg, Size(8, 2), corners, true);
	//drawChessboardCorners(cimg, boardSize, corners, found);
	double sf = 640. / MAX(img.rows, img.cols);
	resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
	imshow("corners", cimg1);
	char c = (char)waitKey(500);
	if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
		exit(-1);
	cornerSubPix(img, corners, Size(8, 2), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
}
void find_point(Mat &img, vector<Point2f> &corners) {
	bool found;
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.filterByArea = true;
	params.minArea = 50;
	params.maxArea = 1e5;
	params.blobColor = 255;

	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	found = findCirclesGrid(img, Size(8, 2), corners, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
	show_find_point(img, corners);
	std::cout << corners << std::endl;
}

void split_img(Mat &img, Rect left,Rect middle, Rect right,Mat &left_known, Mat &target, Mat &right_known) {
	left_known = img(left);
	target = img(middle);
	right_known = img(right);

}

void fix_roi(vector<Point2f>& corners, Point2f &roi) {
	for (int i = 0; i < corners.size(); i++) {
		corners[i] += roi;
	}
}

//void use_known_point_to_rectify_known_result(vector<Vec3f> &known_point) {
//
//	Mat liner;
//	Mat result = estimateAffine2D(known_point, real_point, liner, cv::LMEDS, 1e-6, 1e5, 0.99, 1e4);
//
//
//	Mat  affine_R, affine_T;
//	affine_R = result(Rect(0, 0, 2, 2));
//	affine_T = result.colRange(2, 3);
//}

int main() {
	/*Mat img = imread("d://out.bmp", IMREAD_GRAYSCALE);
	show_img(img);
	vector<Point2f> corners;

	for (int i = 0; i < 2; i++) {
		corners.clear();
		find_point(img, corners);
	}*/
	Mat left_img = imread("d://left.bmp", IMREAD_GRAYSCALE);
	Mat right_img  = imread("d://right.bmp", IMREAD_GRAYSCALE);
	Rect left_left = Rect(Point(1817, 1457), Point(2273, 2665)), left_middle = Rect(Point(2273, 1457), Point(4517, 2681)), left_right = Rect(Point(4517, 1457), Point(4961, 2681));
	Rect right_left = Rect(Point(251,1397), Point(629,2615)), right_middle = Rect(Point(629,1397), Point(2933,2615)), right_right = Rect(Point(2933,1397), Point(3311,2615));
	Mat left_left_img, left_middle_img, left_right_img;
	Mat right_left_img, right_middle_img, right_right_img;
	split_img(left_img, left_left, left_middle, left_right, left_left_img, left_middle_img, left_right_img);
	split_img(right_img, right_left, right_middle, right_right, right_left_img, right_middle_img, right_right_img);
	vector<Point2f> left_corners, right_corners;
	find_point(left_left_img, left_corners);
	find_point(left_right_img, right_corners);
	fix_roi(left_corners, Point2f(1817, 1457));
	fix_roi(right_corners, Point2f(4517, 2681));
	cout << left_corners;




	//FileStorage fs("coodinate.yml", FileStorage::READ);
	//Mat affine_R, affine_T;
	//if (fs.isOpened())
	//{
	//	fs["R"] >> affine_R;
	//	fs["T"] >> affine_T;
	//}
	//cout << affine_R << affine_T;
	//ifstream ifs("result.txt");
	//ofstream outfile("predict.txt", ios::app);
	//double x, y, z;
	//while (ifs >> x >> y >> z)
	//{
	//	Mat result;
	//	result = (Mat_<double>(3,1) << x, y, z);
	//	//Mat result = affine_R * point.t() + affine_T;
	//	/*添加 修正之后的参数*/
	//	rectify_point(result);
	//	cout << result << endl;
	//	outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0) << endl;

	//}
	cin.get();
	return 0;
}

void rectify_point(Mat &point) {
	vector<double> rectify_param(30);
	rectify_param.clear();
	ifstream rec_params("rectify.txt");
	string line;
	while (getline(rec_params, line))
	{
		istringstream ss(line); // 建立ss与line之间的关联
		double t;
		while (ss >> t) { rectify_param.push_back(t); }
	}
	if (rectify_param.size() % 3 != 0) {
		cout << "参数错误,修正失败" << endl;
		return;
	}
	double x = point.at<double>(0, 0), y = point.at<double>(1, 0), z = point.at<double>(2, 0);

	int param_size = rectify_param.size();
	// stride  是  10  次多项式   一共11个参数
	int stride = param_size / 3;
	double temp_x = 0, temp_y = 0, temp_z = 0;
	for (int j = stride; j > 0; j--) {
		double p_x = rectify_param[stride - j];
		double p_y = rectify_param[stride * 2 - j];
		double p_z = rectify_param[stride * 3 - j];
		int i = j;
		while (--i > 0) {
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
	point.at<double>(2, 0) = temp_z+ z;
}