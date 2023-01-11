#ifndef __VFH_H
#define __VFH_H

#include "data_container.h"
#include "data_type.h"
#include <unordered_map>
#include <opencv2/opencv.hpp>

#define DIST_MAX 30 
#define WIDTH 420
#define HEIGHT 300

namespace planner {

template<typename T>
static void angleNormalize(T &angle)
{
	if (angle >= 180) {
		angle -= 360;
	}

	if (angle <= -180) {
		angle += 360;
	}
}

template<typename T>
class VFH
{
public:
	using DataType = T;

	VFH()
	{

	}

	~VFH()
	{

	}

	void setScan(const char* buff)
	{
		memcpy(&scan, buff, sizeof(scan));
	}

	void getHistP()
	{
		for (int i = 0; i < 160; i++) {
			DataType belta = static_cast<DataType>(scan.angles[i]);
			angleNormalize(belta);

			DataType dist = static_cast<DataType>(scan.dists[i]) * 0.001;
			std::cout << "belta = " << belta << ", dist = " << dist << std::endl;

			if (belta == 90 || belta == -90) {
				continue;
			}

			if (dist <= 0.005 || dist >= 0.3) {
				dist = DIST_MAX;
			}

			DataType m = c * (a - b * dist);

			int k = std::ceil(belta / alpha);
			hist_p[k] += m;
		}

	}

	void getHistB()
	{
		/*if (!is_init) {
		is_init = true;
		hist_b_pre = hist_b;
		return;
		}*/

		for (auto& it : hist_p) {
			if (it.second >= tau_high) {
				hist_b[it.first] = 1;
			}
			else if (it.second < tau_low) {
				hist_b[it.first] = 0;
			}
			//else {
			//      hist_b[it.first] = hist_b_pre[it.first];
			//}
		}
		//hist_b_pre = hist_b;
	}

	void showHistP()
	{
		cv::Mat hist_p_img = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
		for (auto& it : hist_p) {
			std::cout << "hist P : " << it.first << ", " << it.second << std::endl;
			int x = it.first * 20 + WIDTH / 2;
			int y = HEIGHT - it.second * 10;
			int width = 20;
			int height = it.second * 10;
			cv::rectangle(hist_p_img, cv::Rect(x, y, width, height), cv::Scalar(0, 0, 255), -1);
		}


		cv::imshow("hist P", hist_p_img);
		cv::waitKey(1);

	}

	void showHistB()
	{
		cv::Mat hist_b_img = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
		for (auto& it : hist_b) {
			std::cout << "hist B : " << it.first << ", " << it.second << std::endl;
			int x = it.first * 20 + WIDTH / 2;
			int y = HEIGHT - it.second * 50;
			int width = 20;
			int height = it.second * 50;
			cv::rectangle(hist_b_img, cv::Rect(x, y, width, height), cv::Scalar(0, 0, 255), -1);
		}


		cv::imshow("hist B", hist_b_img);
		cv::waitKey(1);
	}

public:
	sensor::LaserScan scan;

private:

	std::unordered_map<int, DataType> hist_p;
	std::unordered_map<int, DataType> hist_b;

	static std::unordered_map<int, DataType> hist_b_pre;

	static const DataType alpha;
	static const DataType a;
	static const DataType b;
	static const DataType c;

	static bool is_init;

	static const DataType tau_high;
	static const DataType tau_low;
};

template<typename T>
const typename VFH<T>::DataType VFH<T>::tau_high = 8;

template<typename T>
const typename VFH<T>::DataType VFH<T>::tau_low = 8;

template<typename T>
bool VFH<T>::is_init = false;

template<typename T>
std::unordered_map<int, typename VFH<T>::DataType> VFH<T>::hist_b_pre;

template<typename T>
const typename VFH<T>::DataType VFH<T>::alpha = 5.0;

template<typename T>
const typename VFH<T>::DataType VFH<T>::a = 3;

template<typename T>
const typename VFH<T>::DataType VFH<T>::b = 10;

template<typename T>
const typename VFH<T>::DataType VFH<T>::c = 1;

}
#endif
