#include "data_transport.h"

#include <opencv2/opencv.hpp>


#define MAX_SIZE 1000
char buffer[MAX_SIZE + 8];

int main()
{
	std::cout<<"------------------ IMAGES SENDER -------------------"<<std::endl;

	transport::UdpClient udp;
	udp.initUdpClient();

	cv::VideoCapture v_cap;

	v_cap.open(0);

	if (!v_cap.isOpened()) {
		std::cout << "can not open the camera !" << std::endl;
		return 0;
	}
	std::cout << "Open the Camera !" << std::endl;

	// quality
	std::vector<int> quality;
	quality.push_back( cv::IMWRITE_JPEG_QUALITY );
	quality.push_back( 80 );
	
	cv::Mat frame;
	while (1) {
		int ret = v_cap.read(frame);
		if (ret < 0) {
			std::cerr << "can not read the frame !" << std::endl;
			break;
		}

		cv::imshow( "video", frame );

		cv::resize( frame, frame, cv::Size( 640, 480 ) );

		if ((char)cv::waitKey(40) == 's') {
			cv::imwrite("test.jpg", frame);
		}

		// send
		std::vector<uchar> data_encode;
		cv::imencode( ".jpg", frame, data_encode, quality );

		int size = data_encode.size();
		int times = size / MAX_SIZE;
		int extra = size % MAX_SIZE;
		std::cout<<"jpg size = "<<size<<std::endl;


		buffer[0] = 'L';
		buffer[1] = 'E';
		buffer[2] = 'N';
		buffer[3] = 'G';
		memcpy(&buffer[4], &size, 4);

		udp.write( buffer, 8, "127.0.0.1", 2334 );

		int i = 0;
		for( ; i < times; i ++ ){
			buffer[0] = 'd';
	                buffer[1] = 'a';
        	        buffer[2] = 't';
                	buffer[3] = 'a';

			memcpy(&buffer[4], &data_encode.data()[MAX_SIZE * i], MAX_SIZE);
			udp.write( buffer, MAX_SIZE + 4, "127.0.0.1", 2334 );
		}

		buffer[0] = 'f';
                buffer[1] = 'e';
                buffer[2] = 'n';
                buffer[3] = 'd';

		memcpy(&buffer[4], &data_encode.data()[MAX_SIZE * i], extra + 4);
                udp.write( buffer, extra + 4, "127.0.0.1", 2334 );
		std::cout<<"send count = "<<i<<std::endl;

		cv::waitKey(40);
	}
	v_cap.release();

	return 0;
}
