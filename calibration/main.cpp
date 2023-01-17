#include <opencv2\opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

std::vector<cv::Point2f> src_points;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	Mat image = *(Mat*)userdata;
	char temp[16];
	if (event == EVENT_LBUTTONDOWN) {
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		sprintf(temp, "(%d,%d)", x, y);

		cv::Mat image_clone = image.clone();

		putText(image_clone, temp, Point(x, y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
		cv::circle(image_clone, Point(x, y), 4, cv::Scalar(0, 0, 255), -1);

		if (src_points.size() <= 4) {
			src_points.push_back(Point2f(x, y));
		}

		imshow("test", image_clone);
	}
}

int main()
{
	cv::Mat img = imread("4.jpg");
	namedWindow("test", 0);//可缩放的窗口
	imshow("test", img);

	setMouseCallback("test", CallBackFunc, &img);//鼠标回调事件

	waitKey(0);

	std::cout << "test " << std::endl;

	std::vector<cv::Point2f> dst_points(4);
	dst_points[0] = Point2f(200, 220);
	dst_points[1] = Point2f(440, 220);
	dst_points[2] = Point2f(440, 460);
	dst_points[3] = Point2f(200, 460);

	for (int i = 0; i < src_points.size(); i++) {
		std::cout << src_points[i].x << ", " << src_points[i].y << std::endl;
	}

	cv::Mat m = cv::getPerspectiveTransform(src_points, dst_points);

	std::cout << "perspective transfrom : " << std::endl << m << std::endl;

	cv::Mat ret_img;
	cv::warpPerspective(img, ret_img, m, cv::Size(640, 480));

	cv::imshow("ret", ret_img);
	cv::waitKey(0);

	cv::imwrite("test.jpg", ret_img);

	return 0;
}
