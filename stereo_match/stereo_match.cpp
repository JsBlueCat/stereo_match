// stereo_match.cpp: 定义应用程序的入口点。
//

#include "stereo_match.h"

void find_cp3_point(Mat img, vector<Vec3f> &known_l_point, vector<Vec3f> &known_r_point, vector<Vec3f> &target_point);
bool compute_function(Vec3f A, Vec3f B) {
	if ((cvRound(A[0]) / 10) < (cvRound(B[0]) / 10)) {
		return true;
	}
	else if ((cvRound(A[0]) / 10) == (cvRound(B[0]) / 10)) {
		return ((cvRound(A[1]) / 10) < (cvRound(B[1]) / 10));
	}
	else {
		return false;
	}
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
	Mat img1 = imread(img1_filename, 0);
	Mat img2 = imread(img2_filename, 0);

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

	Size img_size = img1.size();

	Rect roi1, roi2;

	if (!intrinsic_filename.empty())
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename.c_str());
			return -1;
		}

		Mat M1(3, 3, CV_64FC1), D1(1, 14, CV_64FC1), M2(3, 3, CV_64FC1), D2(1, 14, CV_64FC1);
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

		Mat R(3, 3, CV_64FC1), T(3, 1, CV_64FC1), R1(3, 3, CV_64FC1), P1(3, 4, CV_64FC1), R2(3, 3, CV_64FC1), P2(3, 4, CV_64FC1);
		Mat Q(4, 4, CV_64FC1);
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["P1"] >> P1;
		fs["R2"] >> R2;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		//stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		img1 = img1r;
		img2 = img2r;

		vector<Vec3f> known_l_points_l, known_l_points_r, target_l_points, known_r_points_l, known_r_points_r, target_r_points;
		// 左目拍摄 的 右图 
		find_cp3_point(img1, known_r_points_l, known_r_points_r, target_r_points);
		// 左目拍摄 的 左图
		find_cp3_point(img2, known_l_points_l, known_l_points_r, target_l_points);
		for (size_t i = 0; i < known_l_points_l.size(); i++) {
			cout << "第 " << i + 1 << "对点" << endl;
			Point2d A(cvRound(known_r_points_l[i][0]), cvRound(known_r_points_l[i][1])), B(cvRound(known_l_points_l[i][0]), cvRound(known_l_points_l[i][1]));
			Point2d C(cvRound(known_r_points_r[i][0]), cvRound(known_r_points_r[i][1])), D(cvRound(known_l_points_r[i][0]), cvRound(known_l_points_r[i][1]));
			double distance_l = _ABS(A.x - B.x);
			//cout << distance_l << endl;
			Mat temp(4, 1, CV_64FC1);
			temp.at<double>(0, 0) = A.x;
			temp.at<double>(1, 0) = A.y;
			temp.at<double>(2, 0) = distance_l;
			temp.at<double>(3, 0) = 1.;
			Mat result(4, 1, CV_64FC1);
			result = (Q * temp);
			double W = result.at<double>(3, 0);
			result /= W;
			//cout << result << endl;
			double f = M1.at<double>(0, 0);
			double tx = _ABS(T.at<double>(0, 0));
			result.at<double>(2, 0) = f * tx / distance_l;
			cout << "左边的已知点" << i + 1 << "是:" << result << endl;


			double distance_r = _ABS(C.x - D.x);
			//cout << distance_r << endl;
			temp.at<double>(0, 0) = C.x;
			temp.at<double>(1, 0) = C.y;
			temp.at<double>(2, 0) = distance_r;
			temp.at<double>(3, 0) = 1.;
			result = (Q * temp);
			W = result.at<double>(3, 0);
			result /= W;
			//cout << result << endl;
			result.at<double>(2, 0) = f * tx / distance_r;
			cout << "右边的已知点" << i + 1 << "是:" << result << endl;

		}
		// 计算 cp3
		for (size_t i = 0; i < target_r_points.size(); i++) {
			cout << "第 " << i + 1 << "对点" << endl;
			Point2d A(cvRound(target_r_points[i][0]), cvRound(target_r_points[i][1])), B(cvRound(target_l_points[i][0]), cvRound(target_l_points[i][1]));
			double distance_l = _ABS(A.x - B.x);
			//cout << distance_l << endl;
			Mat temp(4, 1, CV_64FC1);
			temp.at<double>(0, 0) = A.x;
			temp.at<double>(1, 0) = A.y;
			temp.at<double>(2, 0) = distance_l;
			temp.at<double>(3, 0) = 1.;
			Mat result(4, 1, CV_64FC1);
			result = (Q * temp);
			double W = result.at<double>(3, 0);
			result /= W;
			//cout << result << endl;
			double f = M1.at<double>(0, 0);
			double tx = _ABS(T.at<double>(0, 0));
			result.at<double>(2, 0) = f * tx / distance_l;
			cout << "cp3点" << i + 1 << "是:" << result << endl;
		}

	}
	return 0;
}


void find_cp3_point(Mat img, vector<Vec3f> &known_l_point, vector<Vec3f> &known_r_point, vector<Vec3f> &target_point) {
	//Mat img_gauss;
	//GaussianBlur(img, img_gauss, Size(9, 9), 2, 2);
	vector<Vec3f> circles;
	HoughCircles(img, circles, HOUGH_GRADIENT,
		2, 20, 200, 50);
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// draw the circle center
		circle(img, center, 3, Scalar(0, 255, 255), 3, 8, 0);
		// draw the circle outline
		circle(img, center, radius, Scalar(255, 0, 255), 10, 8, 0);
	}
	sort(circles.begin(), circles.end(), compute_function);
	known_l_point.assign(circles.begin(), circles.begin() + 4);
	known_r_point.assign(circles.end() - 4, circles.end());
	target_point.assign(circles.begin() + 4, circles.end() - 4);
	//cout << known_l_point.size() << "," << known_r_point.size() << "," << target_point.size() << endl;
	/*for (size_t i = 0; i < circles.size(); i++) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		cout << center << "radius" << radius << endl;
	}
	*/
	namedWindow("circles", 0);
	imshow("circles", img);
	waitKey(0);
}
