#ifndef __IMAGES_RECEIVE_H
#define __IMAGES_RECEIVE_H

#include "EpollEvent.h"
#include "data_transport.h"
#include <vector>

#include <opencv2/opencv.hpp>

#define MAX_RECV_SIZE 1400

namespace transport
{

using namespace std::placeholders;

class ImageReceiver : public UdpServer<MAX_RECV_SIZE + 8>
{
public:
        ImageReceiver()
        {

        }

        ~ImageReceiver()
        {

        }

	void init()
	{
		this->initUdpServer();

		// epoll event
		Event recv_event;
		recv_event.fd = this->sock_fd;
		recv_event.event |= EPOLLIN;
		recv_event.event |= EPOLLERR;
		recv_event.event |= EPOLLET;

		FUNC recv_cb = std::bind( &ImageReceiver::recvCallback, this, _1, _2 );
                recv_event.callback = recv_cb;

                event_base.addEvent( recv_event );
	}

	void* recvCallback( int fd, void* arg )
        {
		int len = this->read();


		if( len > 0 ){
			if( recv_buffer[0] == 'L' && recv_buffer[1] == 'E' && recv_buffer[2] == 'N' && recv_buffer[3] == 'G' ) {
				memcpy( &image_size, &recv_buffer[4], 4 );
				std::cout<<"image size = "<<image_size<<std::endl;	
				
				image_data.resize( image_size );
			}
			else if( recv_buffer[0] == 'd' && recv_buffer[1] == 'a' && recv_buffer[2] == 't' && recv_buffer[3] == 'a' ) {
				memcpy( &image_data.data()[MAX_RECV_SIZE * recv_count], &recv_buffer[4], MAX_RECV_SIZE );
				recv_count ++;
			}
			else if( recv_buffer[0] == 'f' && recv_buffer[1] == 'e' && recv_buffer[2] == 'n' && recv_buffer[3] == 'd' ) {
				std::cout<<"recv one frame end !"<<std::endl;
				if( len != 4 ){
					memcpy( &image_data.data()[MAX_RECV_SIZE * recv_count], &recv_buffer[4], len - 4 );
					// load image
					//image = cv::imdecode( cv::Mat( 1, image_size, CV_8UC1, image_data.data() ), CV_LOAD_IMAGE_UNCHANGED );
					image = cv::imdecode( cv::Mat( 1, image_size, CV_8UC1, image_data.data() ), cv::IMREAD_UNCHANGED );
					cv::flip( image, image, 0 );
					cv::imshow("video", image);
					cv::waitKey(5);

				}


				image_size = 0;
				recv_count = 0;
				image_data.clear();
                        }

		}
		
		return nullptr;
	}

	void spin()
        {
                while(true){
                        event_base.dispatcher();
                }
        }

private:
	EpollEvent event_base;

	int image_size = 0; 
	int recv_count = 0;
	std::vector<char> image_data;

	cv::Mat image;
};



}

#endif
