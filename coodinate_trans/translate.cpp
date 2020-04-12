#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include < iomanip >

using namespace std;
using namespace cv;

Mat R, T;
void read_stored_affine() {
	FileStorage fs("coodinate2d.yml", FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", "intrinsics.yml");
		exit(0);
	}

	fs["R"] >> R;
	fs["T"] >> T;
	fs.release();
}
int main() {
	read_stored_affine();
	ifstream data("test_data.txt"); //����ȡ�ļ���Ŀ¼
	ofstream outfile("predict_rareway.txt", ios::app);
	outfile.setf(ios::fixed, ios::floatfield);  // �趨Ϊ fixed ģʽ����С�����ʾ������
	outfile.precision(5);
	vector<int> res;
	string line;
	while (getline(data, line)) {
		stringstream ss; //������
		ss << line; //�����д�ֵ
		if (!ss.eof()) {
			string line_id;
			Mat row_data = Mat::zeros(cv::Size(1,2), CV_64FC1);
			ss >> line_id >> row_data.at<double>(0,0) >> row_data.at<double>(1,0);
			Mat predict_point = (R * row_data + T);
			outfile << line_id << " : " << predict_point.at<double>(0, 0) << " , " << predict_point.at<double>(1, 0) << endl;
		}
	}
	outfile.close();
	cin.get();
	return 0;
}