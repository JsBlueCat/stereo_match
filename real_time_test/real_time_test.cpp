#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include < iomanip >
using namespace std;
using namespace cv;
void rectify_point(Mat &point);
int main() {
	//FileStorage fs("coodinate.yml", FileStorage::READ);
	//Mat affine_R, affine_T;
	//if (fs.isOpened())
	//{
	//	fs["R"] >> affine_R;
	//	fs["T"] >> affine_T;
	//}
	//cout << affine_R << affine_T;
	ifstream ifs("result.txt");
	ofstream outfile("predict.txt", ios::app);
	double x, y, z;
	while (ifs >> x >> y >> z)
	{
		Mat result;
		result = (Mat_<double>(3,1) << x, y, z);
		//Mat result = affine_R * point.t() + affine_T;
		/*添加 修正之后的参数*/
		rectify_point(result);
		cout << result << endl;
		outfile << result.at<double>(0, 0) << " " << result.at<double>(1, 0) << " " << result.at<double>(2, 0) << endl;

	}
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