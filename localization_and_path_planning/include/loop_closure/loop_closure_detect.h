#ifndef __LOOP_CLOSURE_DETECT_H
#define __LOOP_CLOSURE_DETECT_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

#include <opencv2/imgproc/types_c.h>

namespace loop
{

class LoopClosureDetect
{
public:
	LoopClosureDetect()
	{
		m = (cv::Mat_<float>(3, 3) << 0.9805284446821321, 0.4785711510117803, 18.07706749956502,
        				     -0.02376233899824275, 2.171317981294783, -199.1425802477846,
        				     -8.226651254010183e-05, 0.001605764902014037, 1);
	}

	~LoopClosureDetect()
	{

	}

	void imageProcess( cv::Mat& source )
	{
		if (source.empty()) {
			return;
		}

		cv::Mat image;
		cv::warpPerspective(source, image, m, cv::Size(640, 480));
		
		cv::Mat src = image.clone();
		
		cv::cvtColor(image, image, CV_BGR2GRAY);
		cv::threshold(image, image, 250, 255, CV_THRESH_BINARY);
		
		std::vector<std::vector<cv::Point> > contours;

		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(image.clone(), contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		std::vector<cv::Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			mu[i] = moments(contours[i], false);
		}
	
		std::vector<cv::Point2f> mc;
		for (int i = 0; i < contours.size(); i++) {
			if (fabs(cv::contourArea(contours[i])) > 300) { 
				mc.push_back(cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00));
			}
		}

		std::vector<cv::Point2f> world_pose;
		for (int i = 0; i < mc.size(); i++) {
			//std::cout << i << ": x = " << mc[i].x << ", y = " << mc[i].y << std::endl;
			cv::circle(image, mc[i], 5, cv::Scalar(0, 0, 255), -1);

			float pose_x = (460 - mc[i].y) / 6 + 80;
			float pose_y = (320 - mc[i].x) / 6;
			cv::putText(image, cv::format("(%.2f, %.2f)", pose_x, pose_y), cv::Point(mc[i].x, mc[i].y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 1);
			world_pose.push_back(cv::Point2f(pose_x, pose_y));
		}

		pose = getPose(world_pose, mc, src);
	
		cv::imshow("dst", src);
		cv::waitKey(5);
		cv::imshow("image", image);
		cv::waitKey(5);
	}

	const Eigen::Matrix<float, 3, 1>& getPose() const
	{
		return pose;
	}	

private:
	
	template<typename T>	
	const Eigen::Matrix<typename T::value_type, 3, 1> getPose(const std::vector<T>& poses, const std::vector<T>& img_poses, cv::Mat& img)
	{
		Eigen::Matrix<typename T::value_type, 3, 1> ret_pose(0, 0, 0);

		if (poses.size() == 3) {
			typename T::value_type d1 = std::sqrt((poses[0].x - poses[1].x) * (poses[0].x - poses[1].x) + (poses[0].y - poses[1].y) * (poses[0].y - poses[1].y));
			typename T::value_type d2 = std::sqrt((poses[0].x - poses[2].x) * (poses[0].x - poses[2].x) + (poses[0].y - poses[2].y) * (poses[0].y - poses[2].y));
			typename T::value_type d3 = std::sqrt((poses[2].x - poses[1].x) * (poses[2].x - poses[1].x) + (poses[2].y - poses[1].y) * (poses[2].y - poses[1].y));

			std::cout << "d1 = " << d1 << ", d2 = "<< d2<<", d3 = "<< d3 << std::endl;

			if (d1 < d2 && d2 < d3) { // d1 d2 d3
				//std::cout << "d1 d2 d3" << std::endl;
				cv::line(img, cv::Point(img_poses[0].x, img_poses[0].y), cv::Point(img_poses[1].x, img_poses[1].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[0].x + poses[2].x) / 2;
				ret_pose[1] = (poses[0].y + poses[2].y) / 2;
				ret_pose[2] = cacuAngle(poses[0], poses[1]);
			}
			else if (d1 < d3 && d3 < d2) { // d1 d3 d2
				//std::cout << "d1 d3 d2" << std::endl;
				cv::line(img, cv::Point(img_poses[0].x, img_poses[0].y), cv::Point(img_poses[1].x, img_poses[1].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[1].x + poses[2].x) / 2;
				ret_pose[1] = (poses[1].y + poses[2].y) / 2;
				ret_pose[2] = cacuAngle(poses[1], poses[0]);
			}
			else if (d2 < d1 && d1 < d3) { // d2 d1 d3
				//std::cout << "d2 d1 d3" << std::endl;
				cv::line(img, cv::Point(img_poses[0].x, img_poses[0].y), cv::Point(img_poses[2].x, img_poses[2].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[0].x + poses[1].x) / 2;
				ret_pose[1] = (poses[0].y + poses[1].y) / 2;
				ret_pose[2] = cacuAngle(poses[0], poses[2]);
			}
			else if (d2 < d3 && d3 < d1) { // d2 d3 d1
				//std::cout << "d2 d3 d1" << std::endl;
				cv::line(img, cv::Point(img_poses[0].x, img_poses[0].y), cv::Point(img_poses[2].x, img_poses[2].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[2].x + poses[1].x) / 2;
				ret_pose[1] = (poses[2].y + poses[1].y) / 2;
				ret_pose[2] = cacuAngle(poses[2], poses[0]);
			}
			else if (d3 < d2 && d2 < d1) { // d3 d2 d1
				//std::cout << "d3 d2 d1" << std::endl;
				cv::line(img, cv::Point(img_poses[1].x , img_poses[1].y), cv::Point(img_poses[2].x, img_poses[2].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[2].x + poses[0].x) / 2;
				ret_pose[1] = (poses[2].y + poses[0].y) / 2;
				ret_pose[2] = cacuAngle(poses[2], poses[1]);
			}	
			else if (d3 < d1 && d1 < d2) { // d3 d1 d2
				//std::cout << "d3 d1 d2" << std::endl;
				cv::line(img, cv::Point(img_poses[1].x, img_poses[1].y), cv::Point(img_poses[2].x, img_poses[2].y), cv::Scalar(0, 255, 0), 3);
				ret_pose[0] = (poses[1].x + poses[0].x) / 2;
				ret_pose[1] = (poses[1].y + poses[0].y) / 2;
				ret_pose[2] = cacuAngle(poses[1], poses[2]);
			}
		}

		cv::putText(img, cv::format("(%.2f, %.2f, %.2f)", ret_pose[0], ret_pose[1], ret_pose[2]), cv::Point(300, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 1);
		return ret_pose;
	}

	template<typename T>
	auto cacuAngle(const T& p0, const T& p1)->typename T::value_type
	{	
        	//std::cout << "p0 = ( " << p0.x << ", " << p0.y << " ); p1 = ( " << p1.x << ", " << p1.y << " )" << std::endl;

        	typename T::value_type angle = 0;
        	angle = ::atan2((p1.y - p0.y), (p1.x - p0.x));

        	return angle;
	}


private:
	cv::Mat m; 
	Eigen::Matrix<float, 3, 1> pose;
};

}

#endif
