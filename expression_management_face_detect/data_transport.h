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

#include <arpa/inet.h>

namespace transport
{

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

 		std::cout<<"received data len : "<<ret<<std::endl;
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


class UdpClient
{
public:
	UdpClient()
	{
	
	}

	~UdpClient()
	{
	
	}

	bool initUdpClient()
	{
		sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
                if( sock_fd < 0 ){
                        std::cerr<<"Can not create socket FD : "<<sock_fd<<std::endl;
                        return false;
                }

		std::cout<<"Init Upd Client : "<<sock_fd<<std::endl;

		return true;
	}

	const int write( const char* data, int len, const std::string& ip, const int port  )
	{
		struct sockaddr_in dst_addr;
		dst_addr.sin_family = AF_INET;
		dst_addr.sin_addr.s_addr = inet_addr( ip.c_str() );
		dst_addr.sin_port = htons( port );

		int ret = ::sendto( sock_fd, data, len, 0, (struct sockaddr*)&dst_addr, sizeof( dst_addr ) );
		if( ret <= 0 ){
			std::cerr<<"send data failed "<<ret<<std::endl;
		}

		return ret;
	
	}

private:
	int sock_fd = -1;
};


}

#endif
