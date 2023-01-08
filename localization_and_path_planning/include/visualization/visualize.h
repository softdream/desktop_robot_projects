#ifndef __VISUALIZE_H
#define __VISUALIZE_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace visualize
{

class Visualization
{
public:
	Visualization()
	{
		mapInit();
	}

	Visualization( const int width_, const int height_ ) : map_width( width_ ), map_height( height_ )
	{
		mapInit();
	}

	~Visualization()
	{

	}

	template<typename T>
	void setOdometryPose( const Eigen::Matrix<T, 3, 1>& pose )
	{
		cv::circle(map, cv::Point2f(map_width / 2 - static_cast<float>(pose[0]) * scale, map_height / 2 - static_cast<float>(pose[1]) * scale), 3, cv::Scalar(0, 0, 255), -1);
	}

	void onMapDisplay()
	{
		cv::imshow( "map", map );
		cv::waitKey(5);
	}
	
	void resetMap()
	{
		map = cv::Mat::zeros( map_width, map_height, CV_8UC3 );
	}

private:
	void mapInit()
	{
		map = cv::Mat::zeros( map_width, map_height, CV_8UC3 );
	}

private:
	cv::Mat map;
	int map_width = 800;
	int map_height = 800;
	const float scale = 400;
};
	
}

#endif
