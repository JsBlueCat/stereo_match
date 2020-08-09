#include "stereo_lib.h"

Mat M1(3, 3, CV_64FC1), D1(1, 14, CV_64FC1), M2(3, 3, CV_64FC1), D2(1, 14, CV_64FC1);
Mat R(3, 3, CV_64FC1), T(3, 1, CV_64FC1), R1(3, 3, CV_64FC1), P1(3, 4, CV_64FC1), R2(3, 3, CV_64FC1), P2(3, 4, CV_64FC1);
Mat Q(4, 4, CV_64FC1), affine_R, affine_T;
vector<double> rectify_param(30);

void match_cp3(vector<Point2f> &target_l_points, vector<Point2f> &target_r_points, Mat &Q, Mat &avg);
void find_known_point(Mat& img, vector<Point2f> &target_point);
int find_cp3_point(Mat& img, vector<Point2f> &target_point);
void load_param();
void rectify_point(Mat &point);
void match_known_point(vector<Point2f> &left_target, vector<Point2f> &right_target, Mat &Q, vector<Vec3f> &out);
void match_two_known_point(vector<Point2f> &l_l_known_point, vector<Point2f>&l_r_known_point, vector<Point2f> &r_l_known_point, vector<Point2f> &r_r_known_point
	, Mat &Q, vector<Vec3f> &out);
double distance(Point2f &A, Point2f &B);

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

}

void split_img(Mat &img, Rect left, Rect middle, Rect right, Mat &left_known, Mat &target, Mat &right_known) {
	left_known = img(left);
	target = img(middle);
	right_known = img(right);

}

void fix_roi(vector<Point2f>& corners, Point2f &roi) {
	for (int i = 0; i < corners.size(); i++) {
		corners[i] += roi;
	}
	//std::cout << corners << std::endl;
}
typedef enum {
	left,
	right
} cam_mode;

int find_one_cam_target_point(Mat& img, vector<Point2f> &left_target_point, vector<Point2f> &right_target_point, vector<Point2f> &middle_known_point, cam_mode mod) {
	left_target_point.clear();
	right_target_point.clear();
	middle_known_point.clear();
	Rect left, middle, right;
	Mat left_img, right_img, middle_img;
	if (mod == cam_mode::left) {
		left = Rect(Point(1871, 1451), Point(2321, 2627)), middle = Rect(Point(2321, 0), Point(4367, img.rows)), right = Rect(Point(4367, 1451), Point(4793, 2627));
	}
	else {
		left = Rect(Point(383, 1373), Point(845, 2633)), middle = Rect(Point(845, 0), Point(2867, img.rows)), right = Rect(Point(2867, 1373), Point(3329, 2633));
	}
	split_img(img, left, middle, right, left_img, middle_img, right_img);
	find_point(left_img, left_target_point);
	find_point(right_img, right_target_point);
	find_cp3_point(middle_img, middle_known_point);

	if (mod == cam_mode::left) {
		fix_roi(left_target_point, Point2f(1871, 1451));
		fix_roi(middle_known_point, Point2f(2321, 0));
		fix_roi(right_target_point, Point2f(4367, 1451));
	}
	else {
		fix_roi(left_target_point, Point2f(383, 1373));
		fix_roi(middle_known_point, Point2f(845, 0));
		fix_roi(right_target_point, Point2f(2867, 1373));
	}
	return (left_target_point.size() > 0) & (right_target_point.size() > 0) & (middle_known_point.size() > 0);
}


RESULT single_shot_and_get_point() {
	StereoCamera Fine_Tune_Device;
	Fine_Tune_Device.OpenDoubleCamera(MV_GAIN_MODE_OFF);
	Mat average = Mat::zeros(3, 1, CV_64FC1);
	Mat average_origin = Mat::zeros(4, 1, CV_64FC1);
	ofstream outfile("xiangxi.txt", ios::app);
	ofstream origin("origin.txt", ios::app);
	ofstream known("known.txt", ios::app);
	int count = 0;
	while (count < EPOCH_NUM) {
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
		imwrite("left.bmp", img1r);
		imwrite("right.bmp", img2r);


		vector<Point2f> target_r_points, target_l_points;
		vector<Point2f> known_l_l_points, known_l_r_points, known_r_l_points, known_r_r_points;
		// 右目拍摄 的 右图 
		int ret1 = find_one_cam_target_point(img1r, known_l_l_points, known_l_r_points, target_l_points, cam_mode::left);
		// 左目拍摄 的 左图
		int ret2 = find_one_cam_target_point(img2r, known_r_l_points, known_r_r_points, target_r_points, cam_mode::right);
		if (!(ret1&ret2)) {
			cout << "没有得到完整的cp3点,或已知靶,请移动到合适位置 " << endl;
			continue;
		}
		count++;
		Mat result(4, 1, CV_64FC1);
		match_cp3(target_l_points, target_r_points, Q, result);
		vector<Vec3f> known_points_xyz;
		match_two_known_point(known_l_l_points, known_l_r_points, known_r_l_points, known_r_r_points, Q, known_points_xyz);
		Mat known_points_mat = Mat(known_points_xyz);
		//std::cout << known_points_mat << endl;
		//保存测量前的坐标
		//average += result;
		average_origin += result;
		Mat get_point = result.rowRange(0, 3);
		//std::cout << get_point << endl;
		Mat predict_point = (affine_R * get_point + affine_T);
		//std::cout << predict_point << endl;

		 /*添加 修正之后的参数*/
		//rectify_point(predict_point);

		cout << predict_point << endl;
		average += predict_point;
		//保存测量后的坐标
		outfile << predict_point.at<double>(0, 0) << " " << predict_point.at<double>(1, 0) << " " << predict_point.at<double>(2, 0) << endl;

	}
	ofstream avgout("pingjun.txt", ios::app);
	average /= 5;
	average_origin /= 5;
	avgout << average.at<double>(0, 0) << " " << average.at<double>(1, 0) << " " << average.at<double>(2, 0) << endl;

	origin << average_origin.at<double>(0, 0) << " " << average_origin.at<double>(1, 0) << " " << average_origin.at<double>(2, 0) << endl;
	return RESULT::RETRUN_SUCCESS;
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
		while (ss >> t) { rectify_param.push_back(t); }
	}
}

// 找到cp3点的目标靶坐标
int  find_cp3_point(Mat& img, vector<Point2f> &target_point) {
	bool found;
	vector<Point2f> corners;
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.filterByArea = true;
	params.minArea = 10;
	params.maxArea = 1e5;
	params.blobColor = 255;

	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	found = findCirclesGrid(img, Size(2, 2), corners, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
	target_point = corners;


	if (!found) {
		//cout << "not find point" << endl;
		return 0;
	}
	else {
		if (true)
		{
			Mat cimg, cimg1;
			cvtColor(img, cimg, COLOR_GRAY2BGR);
			drawChessboardCorners(cimg, Size(2, 2), corners, found);
			//drawChessboardCorners(cimg, boardSize, corners, found);
			double sf = 640. / MAX(img.rows, img.cols);
			resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
			imshow("target", cimg1);
			char c = (char)waitKey(500);
			if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
				exit(-1);
		}

		return 1;
	}
}

void match_cp3(vector<Point2f> &target_l_points, vector<Point2f> &target_r_points, Mat &Q, Mat &avg) {
	assert(target_l_points.size() == target_r_points.size());
	avg = Mat::zeros(4, 1, CV_64FC1);
	for (int i = 0; i < target_l_points.size(); i++) {
		double distance_l = _ABS(target_r_points[i].x - target_l_points[i].x);//_ABS(target_r_points[i].x - target_l_points[i].x);
		Mat temp(4, 1, CV_64FC1);
		//cout << target_r_points[i].x << ","<< target_r_points[i].y << endl;
		temp.at<double>(0, 0) = target_l_points[i].x;
		temp.at<double>(1, 0) = target_l_points[i].y;
		temp.at<double>(2, 0) = distance_l;
		temp.at<double>(3, 0) = 1.;
		Mat result = (Q * temp);
		double W = result.at<double>(3, 0);
		result /= W;
		result.at<double>(3, 0) = distance_l;
		avg += result;
	}
	avg /= target_l_points.size();
	cout << avg << endl;
}

void match_two_known_point(vector<Point2f> &l_l_known_point, vector<Point2f>&l_r_known_point, vector<Point2f> &r_l_known_point, vector<Point2f> &r_r_known_point
	, Mat &Q, vector<Vec3f> &out) {
	vector<Vec3f> l_out, r_out;
	match_known_point(l_l_known_point, r_l_known_point, Q, l_out);
	match_known_point(l_r_known_point, r_r_known_point, Q, r_out);
	for (int i = 0; i < r_out.size(); i++) {
		out.push_back(r_out[i]);
	}
	for (int i = 0; i < l_out.size(); i++) {
		out.push_back(l_out[i]);
	}

}

double distance(Point2f &A, Point2f &B) {
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}
// 立体匹配 已知靶坐标点 
void match_known_point(vector<Point2f> &left_target, vector<Point2f> &right_target, Mat &Q, vector<Vec3f> &out) {
	assert(left_target.size() == right_target.size());
	out.clear();
	for (int i = 0; i < left_target.size(); i++) {
		// 求直线距离 
		double distance_l = _ABS(right_target[i].x - left_target[i].x);  //distance(right_target[i], left_target[i]); 
		Mat temp(4, 1, CV_64FC1);
		temp.at<double>(0, 0) = left_target[i].x;
		temp.at<double>(1, 0) = left_target[i].y;
		temp.at<double>(2, 0) = distance_l;
		temp.at<double>(3, 0) = 1.;
		Mat result = (Q * temp);
		double W = result.at<double>(3, 0);
		result /= W;
		result.at<double>(3, 0) = distance_l;
		double x = result.at<double>(0, 0), y = result.at<double>(1, 0), z = result.at<double>(2, 0);
		Vec3f xyz(x, y, z);
		cout << xyz << endl;
		out.push_back(xyz);
	}
}


void rectify_point(Mat &point) {
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
	point.at<double>(2, 0) = temp_z;
}
