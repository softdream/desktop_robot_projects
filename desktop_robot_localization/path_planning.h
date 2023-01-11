#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

#include "cluster.h"
#include "apf.h"
#include "apf_process.h"

#include "pid_tracking.h"
#include "trajectory.h"

#include <thread>

namespace planning
{

template<typename T>
class PathPlanner
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;

	PathPlanner()
	{

	}

	~PathPlanner()
	{

	}

	void setTargetPose( const Vector2& target )
	{
		target_pose = target;
		return apf_processor.setTargetPose(target);
	}

	const DataType runOncePlanning( const sensor::ScanContainer<DataType>& scan_container, const Vector3& robot_pose )
	{
		// 1. get obstacles clusters
		std::vector<std::vector<typename sensor::ScanContainer<DataType>::type>> clusters;
		int num = cluster.extractEuclideanClusters(scan_container, clusters);
		std::cout << "cluster number = " << num << std::endl;
	
		// 2. 
		robot_pose_2_dimension = Vector2( robot_pose[0], robot_pose[1] );

		// 3. get obstalces world coordinates
		//apf::Obstacles<float> obstacles;
		obs_mux.lock();
		for (auto& it : clusters) {
			Vector2 obs_pose_laser = cluster.getMaxDistAndMean(it);
			Vector2 obs_pose_world = apf_processor.pointLaser2World(obs_pose_laser, robot_pose);
			std::cout << "obstacle pose in world frame = " << obs_pose_world.transpose() << std::endl;
			obstacles.addObstacle( obs_pose_world );

		}
		obs_mux.unlock();

		// 4. get target yaw 
		DataType target_yaw = apf_processor.runApfOnce(robot_pose_2_dimension, obstacles);
	
		
		// 5. clear obstacles 
		obstacles.clearAll();
		
		return target_yaw;
	}

	bool isReached()
	{
		if( ( robot_pose_2_dimension - target_pose ).norm() < 0.04 ) {
			return true;
		}
		else {	
			return false;
		}
	}

	const apf::Obstacles<DataType>& getObstacles() 
	{
		obs_mux.lock();
		return obstacles;
		obs_mux.unlock();
	}

private:
	cluster::Cluster<DataType> cluster;
	apf::APFProcess<DataType> apf_processor;

	// 
	Vector2 robot_pose_2_dimension = Vector2::Zero();
	Vector2 target_pose = Vector2::Zero();

	//
	apf::Obstacles<DataType> obstacles;

	// 
	std::mutex obs_mux;
};

}

#endif
