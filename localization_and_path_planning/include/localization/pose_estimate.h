#ifndef __POSE_ESTIMATE_H
#define __POSE_ESTIMATE_H

#include "ekf_fusion.h"
#include "data_transport.h"
#include "EpollEvent.h"

#include <mutex>
#include <thread>

namespace localization
{

using namespace std::placeholders;

template<typename T>
class PoseEstimate
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
	using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;
	using Vector4 = typename Eigen::Matrix<DataType, 4, 1>;
	using Vector6 = typename Eigen::Matrix<DataType, 6, 1>;

	using Matrix3x3 = typename Eigen::Matrix<DataType, 3, 3>;
	
	PoseEstimate()
	{

	}

	~PoseEstimate()
	{

	}
	
	void init()
	{
		odom_recv.initUdpServer( 2334 );
		
		Event odom_event;
		odom_event.fd = odom_recv.getSocketFd();
		odom_event.event |= EPOLLIN;
		odom_event.event |= EPOLLERR;
		odom_event.event |= EPOLLET;
		odom_event.arg = NULL;
		FUNC recv_cb = std::bind( &PoseEstimate::odomRecvCallback, this, _1, _2 );	
		odom_event.callback = recv_cb;
		
		event_base.addEvent( odom_event );
	
	}	

	void spin()
	{
		while(true){
			event_base.dispatcher();
		}
	}

	void* odomRecvCallback( int fd, void* arg )
	{
		if( odom_recv.read() > 0 ){
			Vector6 encoder_data = Vector6::Zero();
        		Vector4 imu_data = Vector4::Zero();

			transport::MessageType type = odom_recv.readFrameData( encoder_data, imu_data );
			if( type == transport::odom_encoder ){
//				std::cout<<"encoder data = "<<encoder_data.transpose()<<std::endl;
				if( !is_init ){
					return NULL; 
				}

				if (encoder_data[4] != 0 || encoder_data[5] != 0) {
					is_static = false;
				}
				else {
					is_static = true;
				}

				Vector2 u( encoder_data(2), encoder_data(3) );
				odom_ekf.predict(u);
			}
			else if( type == transport::odom_imu ){
//				std::cout<<"imu data = "<<imu_data.transpose()<<std::endl;
			
				DataType gz = -imu_data(3) * ( M_PI / 180 );

				if( is_static ) gz = 0;
	
				DataType now_time = imu_data(0);
				if ( !is_init ) {
					pre_time = now_time;
					pre_gz = gz;
					is_init = true;
					return NULL;
				}

				DataType delta_t = ( now_time - pre_time ) / 1000;
				theta = delta_t * (pre_gz + gz) * 0.5;
				odom_ekf.update(theta);

				// update the value
				pre_gz = gz;
				pre_time = now_time;

				// get the key pose
				if( odom_ekf.isKeyFrame() ){
					pose_mux.lock();
					robot_pose = odom_ekf.getStateX();
					//robot_pose[1] = -robot_pose[1];
					//robot_pose[2] = -robot_pose[2];
//					std::cout << "pose = " << robot_pose.transpose() << std::endl;					
					pose_mux.unlock();	
				}					
			}
		}

		return nullptr;
	}

	const Vector3& getRobotPose() const
	{
		return robot_pose;
	}

	void sendControlVec( const DataType v, const DataType w )
	{
		return odom_recv.sendControlVec( v, w );
	}

	void sendCommand( const transport::Command com ) 
	{
		return odom_recv.sendCommand( com );
	}

	const std::mutex& getMutex()
	{
		pose_mux.lock();
		
		return pose_mux;

		pose_mux.unlock();
	}

	const Matrix3x3 getConvarinceMatrixP() 
	{
		return odom_ekf.getCovarinceMatrixP();
	}

private:
	EpollEvent event_base;

	transport::OdomTransport odom_recv;


	// ekf fusion
	ekf::EKF<DataType> odom_ekf;
	DataType pre_time = 0;
	bool is_init = false;
	DataType pre_gz = 0;
	DataType theta = 0;

	bool is_static = false;

	// pose
	Vector3 robot_pose = Vector3::Zero();
	std::mutex pose_mux;
	//Matrix3x3 convarice_matrix_p = Matrix3x3::Zero();
};

}

#endif
