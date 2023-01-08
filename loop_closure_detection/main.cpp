#include "loop_closure_detect.h"

void loopClosureDetectThread()
{
	loop::LoopClosureDetect loop_detect;
	
	cv::VideoCapture v_cap;

	v_cap.open(8);

	if (!v_cap.isOpened()) {
		std::cout << "can not open the camera !" << std::endl;
		return ;
	}
	std::cout << "Open the Camera !" << std::endl;

	cv::Mat frame;
	while (1) {
		int ret = v_cap.read(frame);
		if (ret < 0) {
			std::cerr << "can not read the frame !" << std::endl;
			break;
		}

		//cv::imshow( "video", frame );
		cv::flip(frame, frame, 0);

		if ((char)cv::waitKey(40) == 's') {
			cv::imwrite("test.jpg", frame);
		}

		loop_detect.imageProcess( frame );
	}

	v_cap.release();
}

int main()
{
	std::cout<<"----------------------------- LOOP CLUSURE DETECTION --------------------------------"<<std::endl;

	

	return 0;
}
