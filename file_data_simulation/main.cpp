#include "file_read.h"

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

#include <thread>

int sock_fd = -1;

const int initUdpClient()
{
	int fd = socket( AF_INET, SOCK_DGRAM, 0 );
	if( fd <= 0 ){
		std::cerr<<"socket Udp Client init failed !"<<std::endl;
		exit(-1);
	}
}

void dataRecv()
{
	char recv_buff[50];

	while(1){
		struct sockaddr_in dest_addr;
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                dest_addr.sin_port = htons( 2334 );
	
		socklen_t sock_len = sizeof( dest_addr );
		int ret = recvfrom(sock_fd, recv_buff, sizeof( recv_buff ), 0 , (struct sockaddr*)&dest_addr, &sock_len);
		if( ret > 0 ){
			std::cout<<recv_buff<<std::endl;
		}
	}
}

int main()
{
	std::cout<<"--------------------- EKF FUSION--------------------"<<std::endl;

	std::thread t1( dataRecv );
	t1.detach();

	sock_fd = initUdpClient();

	ekf::DataRead data_read;
        data_read.openFile( "data/around1.txt" );

	int count = 0;
        while( !data_read.endOfFile() ){
                //std::cout<<"--------------------------frame count : "<<count<<"------------------------" <<std::endl;
	
		std::string data = data_read.readALine();		
	//	std::cout<<data<<std::endl;
	
		struct sockaddr_in dest_addr;
        	dest_addr.sin_family = AF_INET;
        	dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        	dest_addr.sin_port = htons( 2334 );

		int ret = sendto( sock_fd, data.c_str(), data.size(), 0, (struct sockaddr*)&dest_addr, sizeof( dest_addr ) );
	        if( ret <= 0 ){
        	        std::cerr<<"send data falied ..."<<std::endl;
        	}
	        else {
 //       	        std::cerr<<"send data succussfully ..."<<std::endl;
        	}

	
		usleep(20000);
	}

	return 0;
}
