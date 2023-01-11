#include "images_receive.h"

#include "expression_manage.h"

int main()
{
	std::cout<<"----------------------- RECV VIDEO TEST ------------------ "<<std::endl;

	transport::ImageReceiver recv;

	recv.init();

	recv.spin();

	return 0;
}
