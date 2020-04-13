// stereo_match.cpp: 定义应用程序的入口点。
//

#include "stereo_match.h"

bool compute_function(Vec3f A, Vec3f B) {
	return ceil(A[0])/100 < ceil(B[0])/100;
}
bool compute_function_single(Vec3f A, Vec3f B) {
	return A[1] > B[1];
}
int main(int argc, char** argv)
{
	//"images\left(2).bmp" "images\right(2).bmp" -i="intrinsics.yml" -e="extrinsics.yml"
	// "images\2521_left.bmp" "images\2521_right.bmp" -i="intrinsics.yml" -e="extrinsics.yml"
	std::string img1_filename = "";
	std::string img2_filename = "";
	std::string intrinsic_filename = "";
	std::string extrinsic_filename = "";
	std::string disparity_filename = "";
	std::string point_cloud_filename = "";
	cv::CommandLineParser parser(argc, argv,
		"{@arg1||}{@arg2||}{i||}{e||}");
	img1_filename = samples::findFile(parser.get<std::string>(0));
	img2_filename = samples::findFile(parser.get<std::string>(1));
	if (parser.has("i"))
		intrinsic_filename = parser.get<std::string>("i");
	if (parser.has("e"))
		extrinsic_filename = parser.get<std::string>("e");

	if (img1_filename.empty() || img2_filename.empty())
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		return -1;
	}
	if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		return -1;
	}
	//StereoCamera stereo_camera;
	//Mat a, b;
	//stereo_camera.OpenDoubleCamera();
	//stereo_camera.GrabImageDoubleCamera();
	//double sf = 640. / MAX(stereo_camera.left_img.rows, stereo_camera.left_img.cols);
	//resize(stereo_camera.left_img, a, Size(), sf, sf, INTER_LINEAR_EXACT);
	//resize(stereo_camera.right_img, b, Size(), sf, sf, INTER_LINEAR_EXACT);
	//imshow("test_cam", a);
	//waitKey(1000);
	//imshow("test_cam", b);
	//waitKey(1000);

	Mat img1 = imread(img1_filename, 0);
	Mat img2 = imread(img2_filename, 0);

	Size img_size = img1.size();
	// 控制读取图片的个数 
	int n = 80 * 5 ;

	Rect roi1, roi2;
	Mat M1(3, 3, CV_64FC1), D1(1, 14, CV_64FC1), M2(3, 3, CV_64FC1), D2(1, 14, CV_64FC1);
	Mat R(3, 3, CV_64FC1), T(3, 1, CV_64FC1), R1(3, 3, CV_64FC1), P1(3, 4, CV_64FC1), R2(3, 3, CV_64FC1), P2(3, 4, CV_64FC1);
	Mat Q(4, 4, CV_64FC1);

	if (!intrinsic_filename.empty())
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename.c_str());
			return -1;
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		fs.open(extrinsic_filename, FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename.c_str());
			return -1;
		}
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["P1"] >> P1;
		fs["R2"] >> R2;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
	}
	bool flag = true;
	Mat affine_map, inliner;
	Mat affine_R, affine_T;
	vector<Vec3f> cp3_list;
	for (int i = 1; i <= n; i++) {
		string left = "images\\1600_left_(" + to_string(i) + ").bmp";
		string right = "images\\1600_right_(" + to_string(i) + ").bmp";
		img1 = imread(left, 0);
		img2 = imread(right, 0);
		if (img1.empty())
		{
			printf("Command-line parameter error: could not load the first input image file\n");
			return -1;
		}
		if (img2.empty())
		{
			printf("Command-line parameter error: could not load the second input image file\n");
			return -1;
		}
			
		// 矫正
		//stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
		Mat map11, map12, map21, map22;
		Mat img1r, img2r;

		//undistort(img1, img1r, M1, D1);
		//undistort(img2, img2r, M2, D2);

		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);
		// 左图
		img1 = img1r;
		// 右图
		img2 = img2r;

		if (flag) {
			/*****找到合适的坐标系变换矩阵*********/
			find_rectify_matrix(img1, img2, affine_map, inliner, Q);
	/*		cout << affine_map << endl;
			cout << inliner << endl;*/
			affine_R = Mat::eye(3, 3, CV_64F);
			affine_R = affine_map(Rect(0, 0, 3, 3));
			affine_T = affine_map.colRange(3,4);
			flag = false;
		}
		

		//vector<Vec3f> known_l_points_l, known_l_points_r, known_r_points_l, known_r_points_r;
		vector<Point2f> target_r_points, target_l_points;
		// 右目拍摄 的 右图 
		find_cp3_point(img1,target_r_points);
		// 左目拍摄 的 左图
		find_cp3_point(img2,target_l_points);

		Mat result(4, 1, CV_64FC1);
		match_cp3(target_l_points, target_r_points, Q, result);
		affine_T = {-141.,-43.,44.1,0.};
		cout << result + affine_T << endl;

		//Mat temp(3, 1, CV_64FC1);
		//Mat reproject_xyz;
		//temp.at<double>(0, 0) = result.at<double>(0, 0);
		//temp.at<double>(1, 0) = result.at<double>(1, 0);
		//temp.at<double>(2, 0) = result.at<double>(2, 0);
		//reproject_xyz = affine_R * temp + affine_T;
		//cout << "reproject :" << reproject_xyz << endl;

		ofstream outfile("out.txt", ios::app);
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0) << " " << result.at<double>(3, 0) << endl;

	}
	
	return 0;
}
// 找到坐标系变换矩阵
void find_rectify_matrix(Mat& img_l, Mat& img_r, Mat&out, Mat&inliers, Mat &Q) {
	// 已知的 已知靶 坐标点
	vector<Vec3f> known_point = {
		Vec3f(-350,110,1523),
		Vec3f(-350,45,1523),
		Vec3f(-350,-95,1523),
		Vec3f(-350,-160,1523),
		Vec3f(350,110,1523),
		Vec3f(350,45,1523),
		Vec3f(350,-95,1523),
		Vec3f(350,-160,1523),
	};
	vector<Point2f> left_known_point, right_known_point;
	find_known_point(img_l, left_known_point); // 找到左图的点
	find_known_point(img_r, right_known_point); // 找到右图的点
	vector<Vec3f> all_left_point;
	match_known_point(left_known_point, right_known_point, Q, all_left_point);
	for (int i = 0; i < all_left_point.size(); i++) {
		cout << all_left_point[i];
	}
	//for (int i = 0; i < all_left_point.size(); i++) {
	//	cout << all_left_point[i] << endl;
	//}
	estimateAffine3D(all_left_point, known_point,out,inliers,3 ,0.95);
	Vec3f temp(0,0,0);
	for (int i = 0; i < all_left_point.size(); i++) {
		temp += all_left_point[i] - known_point[i];
	}
	temp /= 8;
	cout << temp;
}

// 找到 已知靶的坐标 
void find_known_point(Mat& img, vector<Point2f> &target_point) {
	Rect left_known_target_roi = Rect(Point(0,0), Point(300,1000));
	Rect right_known_target_roi = Rect(Point(2000,2000), Point(2300,3300));
	Mat left_img = img(left_known_target_roi);
	Mat right_img = img(right_known_target_roi);

	bool left_found,right_found;
	vector<Point2f> corners_left, corners_right;
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 2e5;
	params.blobColor = 255;
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	left_found = findCirclesGrid(left_img, Size(6, 1), corners_left, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
	right_found = findCirclesGrid(right_img, Size(6, 1), corners_right, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
	
	assert(corners_left.size() == corners_right.size());
	target_point.clear();
	for (int i = 0; i < corners_left.size(); i++) {
		target_point.push_back(corners_left[i] + Point2f(0,0));
	}
	for (int i = 0; i < corners_left.size(); i++) {
		target_point.push_back(corners_right[i] + Point2f(2000, 2000));
	}
}

// 找到cp3点的目标靶坐标
void find_cp3_point(Mat& img, vector<Point2f> &target_point) {

	bool found;
	vector<Point2f> corners;
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 2e5;
	params.blobColor = 255;
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	found = findCirclesGrid(img, Size(2,2), corners, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);

	assert(found);
	if (false)
	{
		Mat cimg, cimg1;
		cvtColor(img, cimg, COLOR_GRAY2BGR);
		drawChessboardCorners(cimg, Size(2, 2), corners, found);
		//drawChessboardCorners(cimg, boardSize, corners, found);
		double sf = 640. / MAX(img.rows, img.cols);
		resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
		imshow("corners", cimg1);
		char c = (char)waitKey(500);
		if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
			exit(-1);
	}

	//求平均
	//target_point = accumulate(corners.begin(), corners.end(), Point2f(0,0)) / (int)corners.size();
	target_point = corners;
	/*
	// 圆孔目前4个点 
	assert(target_point_list.size() == 4);
	Point2f avg(0, 0);
	for (size_t i = 0; i < target_point_list.size(); i++) {
		Point2f center(target_point_list[i][0],target_point_list[i][1]);
		avg += center;
	}
	target_point = avg / 4;
	*/
	//namedWindow("circles", 0);
	//imshow("circles", img);
	//waitKey(0);
}


// 立体匹配 已知靶坐标点 
void match_known_point(vector<Point2f> &left_target, vector<Point2f> &right_target, Mat &Q ,vector<Vec3f> &out) {
	assert(left_target.size() == right_target.size());
	for (int i = 0; i < left_target.size(); i++) {
		// 求直线距离 
		double distance_l = _ABS(right_target[i].x - left_target[i].x);
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



void match_cp3(vector<Point2f> &target_l_points,vector<Point2f> &target_r_points,Mat &Q,Mat &avg) {
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



//for (size_t i = 0; i < known_l_points_l.size(); i++) {
//	//cout << "第 " << i + 1 << "对点" << endl;
//	Point2d A(cvRound(known_r_points_l[i][0]), cvRound(known_r_points_l[i][1])), B(cvRound(known_l_points_l[i][0]), cvRound(known_l_points_l[i][1]));
//	Point2d C(cvRound(known_r_points_r[i][0]), cvRound(known_r_points_r[i][1])), D(cvRound(known_l_points_r[i][0]), cvRound(known_l_points_r[i][1]));
//	double distance_l = _ABS(A.x - B.x);
//	//cout << distance_l << endl;
//	Mat temp(4, 1, CV_64FC1);
//	temp.at<double>(0, 0) = A.x;
//	temp.at<double>(1, 0) = A.y;
//	temp.at<double>(2, 0) = distance_l;
//	temp.at<double>(3, 0) = 1.;
//	Mat result(4, 1, CV_64FC1);
//	result = (Q * temp);
//	double W = result.at<double>(3, 0);
//	result /= W;
//	//cout << result << endl;
//	double f = M1.at<double>(0, 0);
//	double tx = _ABS(T.at<double>(0, 0));
//	//result.at<double>(2, 0) = f * tx / distance_l;
//	//cout << "左边的已知点" << i + 1 << "是:" << result << endl;


//	double distance_r = _ABS(C.x - D.x);
//	//cout << distance_r << endl;
//	temp.at<double>(0, 0) = C.x;
//	temp.at<double>(1, 0) = C.y;
//	temp.at<double>(2, 0) = distance_r;
//	temp.at<double>(3, 0) = 1.;
//	result = (Q * temp);
//	W = result.at<double>(3, 0);
//	result /= W;
//	//cout << result << endl;
//	//result.at<double>(2, 0) = f * tx / distance_r;
//	//cout << "右边的已知点" << i + 1 << "是:" << result << endl;

//}
// 计算 cp3