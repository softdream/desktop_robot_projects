#ifndef __IMAGES_SEND_H
#define __IMAGES_SEND_H

#include "data_transport.h"

#include <fstream>
#include <iostream>


#define MAX_SIZE 1400

namespace transport
{

class ImageSender : UdpClient
{
public:
	ImageSender()
	{
		init();
	}

	ImageSender( const std::string ip_, const int port_ ) : ip( ip_ ), port( port_ ) 
        {
                init();
        }

	~ImageSender()
	{

	}

	void init()
	{
		this->initUdpClient();
	}


	bool piecesSendData( const char* data, int len )
	{
	    	int times = len / MAX_SIZE;
    		int remain_size = len % MAX_SIZE;

    		int i = 0;
    		for( ; i < times; i ++ ){
			int ret = this->write( &data[MAX_SIZE * i], MAX_SIZE, ip, port );
			if( ret <= 0 ) return false;
			 
   		}
		
		int ret = this->write( &data[MAX_SIZE * i], remain_size, ip, port );
		if( ret <= 0 ) return false;
    		return true;
	}

	bool openFile( const std::string& file_path )
	{
		file.open( file_path.c_str(), std::ios::in | std::ios::binary );
		if( !file.is_open() ){
			std::cerr<<"Failed to Open the File !"<<std::endl;
			return false;
		}

		return true;
	}
	
	void closeFile()
	{
		return file.close();
	}

	const int getFileLength()

	{
		file.seekg(0, std::ios::end);
		int length = file.tellg();
		file.seekg(0, std::ios::beg);
		
		return length;
	}

	bool readFile( char* data, const int len )
	{
		file.read( data, len );
		return true;
	}	
private:
	std::ifstream file;

	std::string ip = "192.168.1.19";
	int port = 2335;
};

}

#endif
