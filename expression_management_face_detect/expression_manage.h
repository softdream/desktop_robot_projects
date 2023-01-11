#ifndef __EXPRESSION_MANAGE_H
#define __EXPRESSION_MANAGE_H

#include "images_send.h"

#include <vector>

namespace expression
{

class ExpressionManagement
{
public:
	ExpressionManagement()	
	{
		blink_vec.resize( 7 );
	}

	~ExpressionManagement()
	{

	}

	void readExpressionImages(  )
	{
		// 1. read blink images
		for( int i = 0; i <= 7; i ++ ){
			
			sender.openFile( blink_path + std::to_string( i ) + ".jpg" );
			int len = sender.getFileLength();
		
			blink_vec[i].first = new char[ len ];
			blink_vec[i].second = len;
			sender.readFile( blink_vec[i].first, blink_vec[i].second );
			sender.closeFile();
		}
	}


private:
	transport::ImageSender sender;

	const std::string happy_path;
	const std::string mad_path = "./images/mad/";
	const std::string blink_path = "./images/blink/";

	std::vector<std::pair<char*, int>> blink_vec;
};

}

#endif
