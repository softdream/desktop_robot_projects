#ifndef __DATA_TRANSPORT_H
#define __DATA_TRANSPORT_H

#include <iostream>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/time.h>

#include <Eigen/Dense>

#include "data_type.h"


namespace transport
{

typedef enum Command_
{
	anti_mode,
	start_lidar,
	stop_lidar
}Command;

typedef enum MessageType_
{
	none,
	odom_encoder,
	odom_imu,
	lidar_started,
	lidar_stoped,
	lidar_anti_mode,
	lidar_init,
	lidar_scan_data
}MessageType;

template<int SIZE>
class UdpServer
{
public:
	UdpServer()
	{

	}

	~UdpServer()
	{

	}

	bool initUdpServer( const int port = 2334 )
	{	
		sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
		if( sock_fd < 0 ){
			std::cerr<<"Can not create socket FD : "<<sock_fd<<std::endl;
			return false;
		}

		bzero( &sock_server_addr, sizeof( sock_server_addr ) );

		sock_server_addr.sin_family = AF_INET;
		sock_server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		sock_server_addr.sin_port = htons( port );
	
		if( ::bind( sock_fd, (struct sockaddr*)&sock_server_addr, sizeof(sock_server_addr) ) < 0 ){
			std::cerr<<"Failed to bind the socket server address !"<<std::endl;
			return -1;
		}

		std::cout<<"Init Udp Socket : "<<sock_fd<<std::endl;
		return true;
	}
	
	const int getSocketFd() const
	{
		return sock_fd;
	}

	const int read()
	{
		memset( recv_buffer, 0, sizeof( recv_buffer ) );

		int ret = ::recvfrom(sock_fd, recv_buffer, sizeof( recv_buffer ), 0 , (struct sockaddr*)&client_sock_addr, &client_sock_len);		
	
		if( ret <= 0 ){
			std::cerr<<"recv failed : "<<ret<<std::endl;
	
		}
		
		return ret;
	}

	const int write( const char* data, const int len )
	{
		int ret = ::sendto( sock_fd, data, len, 0, (struct sockaddr*)&client_sock_addr, client_sock_len );
		
		if( ret <= 0 ){
			std::cerr<<"send failed : "<<ret<<std::endl;
		}

		return ret;
	}
	
protected:
	// server info
	int sock_fd = -1;
	struct sockaddr_in sock_server_addr;

	// remote client info
	struct sockaddr_in client_sock_addr;
	socklen_t client_sock_len ;

	// receive buffer
	char recv_buffer[SIZE];
};

class OdomTransport : public UdpServer<200>
{	
public:
	OdomTransport()
	{

	}

	~OdomTransport()
	{

	}
	
	template<typename T>
	void sendControlVec( const T v, const T w )
	{
		sensor::Control u( static_cast<float>( v ), static_cast<float>( w ) );

		if( this->write( ( char * )&u, sizeof( u ) ) <= 0 ){
			std::cerr<<"failed to send control vector !"<<std::endl;
		}
		
	}
		

	void sendCommand( const Command com )
	{
		switch( com ){
			case anti_mode : { 
				const char *command = "anti";
				if( this->write( command, 4 ) <= 0 ){
					std::cerr<<"send anti mode command failed !"<<std::endl;
				}
				
				break;
			}
			case start_lidar : {
				const char *command = "start";
                                if( this->write( command, 5 ) <= 0 ){
                                        std::cerr<<"send start lidar command failed !"<<std::endl;
                                }

                                break;
			}
			case stop_lidar : {
				const char *command = "stopl";
                                if( this->write( command, 5 ) <= 0 ){
                                        std::cerr<<"send stop lidar command failed !"<<std::endl;
                                }

                                break;
			}
			default : break;
		}
	}

	template<typename T>
	const MessageType readFrameData( Eigen::Matrix<T, 6, 1>& encoder,
				 Eigen::Matrix<T, 4, 1>& imu )
	{
		std::string str = this->recv_buffer;

		encoder.setZero();
		imu.setZero();

		std::istringstream iss(str);

		std::string tag;
		iss >> tag;
	
		if (tag.compare("encoder") == 0) {
			std::string num;
			for (size_t i = 0; i < 6; i++) {
				iss >> num;
				encoder(i) = std::stod(num);
			}
			///std::cout << "encoder = " << encoder.transpose() << std::endl;
			return odom_encoder;
		}
		else if (tag.compare("imu") == 0) {
			std::string num;
			for (size_t i = 0; i < 4; i++) {
				iss >> num;
				imu(i) = std::stod(num);
				
			}
			//std::cout << "imu = " << imu.transpose() << std::endl;
			return odom_imu;
		}

		return none;
	}

};

class LidarTransport : public UdpServer<1000>
{
public:
	LidarTransport()	
	{
	
	}

	~LidarTransport()
	{

	}

	const MessageType readFrameData()
	{
		if (recv_buffer[0] == 's' && recv_buffer[1] == 't' && recv_buffer[2] == 'a' && recv_buffer[3] == 'r') {
			std::cout << recv_buffer << std::endl;
			return lidar_started;
		}
		else if (recv_buffer[0] == 's' && recv_buffer[1] == 't' && recv_buffer[2] == 'o' && recv_buffer[3] == 'p') {
			std::cout << recv_buffer << std::endl;
			return lidar_stoped;
		}
		else if (recv_buffer[0] == 'a' && recv_buffer[1] == 'n' && recv_buffer[2] == 't' && recv_buffer[3] == 'i') {
			std::cout << recv_buffer << std::endl;
			return lidar_anti_mode;
		}
		else if (recv_buffer[0] == 'i' && recv_buffer[1] == 'n' && recv_buffer[2] == 'i' && recv_buffer[3] == 't') {
			std::cout << recv_buffer << std::endl;
			return lidar_init;
		}
		else{
			memset( &scan_data, 0, sizeof( scan_data ) );
			memcpy( &scan_data, recv_buffer, sizeof( scan_data ) );
			
			return lidar_scan_data;
		}
	}

	const sensor::LaserScan& getScanData() const
	{
		return scan_data;
	}

private:
	sensor::LaserScan scan_data;
};

}

#endif
