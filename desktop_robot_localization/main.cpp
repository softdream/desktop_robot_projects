#include "pose_estimate.h"
#include "visualize.h"
#include "keyboard.h"

#include "loop_closure_detect.h"

#include "path_planning.h"

#include <chrono>

// -------------------------------------- GLOBAL DATA ---------------------------------------- //
localization::PoseEstimate<float> pose_estimate;

// robot pose
Eigen::Matrix<float, 3, 1> odom_pose = Eigen::Matrix<float, 3, 1>::Zero();
// covarince matrix
Eigen::Matrix<float, 3, 3> convarice_p = Eigen::Matrix<float, 3, 3>::Identity();

// ------------------------------------------------------------------------------------------- //

void poseEstimateThread()
{
	pose_estimate.init();
	pose_estimate.spin();

	return;
}

void displayThread()
{
	visualize::Visualization visual;
	while( true ){
		odom_pose = pose_estimate.getRobotPose();
		std::cout<<"odom pose : "<<odom_pose.transpose()<<std::endl;
		convarice_p = pose_estimate.getConvarinceMatrixP();
		std::cout<<"Convarince Matrix P : "<<std::endl<<convarice_p<<std::endl;
		
		visual.setOdometryPose( odom_pose );
		visual.onMapDisplay();
		std::this_thread::sleep_for (std::chrono::milliseconds(100));
	}

	return;
}

void keyWPressed()
{
	pose_estimate.sendControlVec( 0.3, 0 );
}

void keyWReleased()
{
	pose_estimate.sendControlVec( 0, 0 );
}

void keyAPressd()
{
	std::cout<<"key A pressed !"<<std::endl;
	pose_estimate.sendCommand( transport::anti_mode );
}

void keyAReleased()
{
	std::cout<<"key A released !"<<std::endl;
	pose_estimate.sendCommand( transport::anti_mode );
}

void keyDPressed()
{

}

void keyDReleased()
{

}

void keyboardControl()
{
	keyboard::Keyboard keyboard;
	
	keyboard.init();
        keyboard.registCallBackFunc( keyWPressed, keyWReleased, 17, keyAPressd, keyAReleased, 30, keyDPressed, keyDReleased, 32);

        keyboard.spin();
}

int main()
{
	std::cout<<"---------------------- DESKTOP ROBORT LOCALIZATION --------------------"<<std::endl;

	std::thread pose_estimate_thread( poseEstimateThread );
	std::thread display_thread( displayThread );
	std::thread keyboard_control_thread( keyboardControl );

	pose_estimate_thread.join();
	display_thread.join();
	keyboard_control_thread.join();

	while(1){

	}

	return 0;
}
