#ifndef __APF_PROCESS_H
#define __APF_PROCESS_H

#include "apf.h"
#include "utils.h"

namespace apf
{

template<typename T>
class APFProcess
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;

	APFProcess()
	{

	}

	~APFProcess()
	{

	}

	void setTargetPose(const Vector2& target_pose_)
	{
		return apf.setTargetPose(target_pose_);
	}

	const DataType runApfOnce(const Vector2& robot_pose, const apf::Obstacles<DataType>& obstacles)
	{
		// 1. attraction
		Vector2 f_att_vec = apf.cacuFatt(robot_pose);

		// 2. repulsion
		Vector2 f_rep_vec = apf.cacuFrep(robot_pose, obstacles);
		std::cout << "repulsion vector = " << f_rep_vec.transpose() << std::endl;

		std::vector<Vector2> repulsion_vec1 = apf.getRepulsionVec1();
		apf.clearRepulsionVec1();

		// 3. total 
		Vector2 f_total = f_att_vec + f_rep_vec;

		// 4. get target angle
		float target_theta = ::atan2(f_total[1], f_total[0]);
		Utils::angleNormalize( target_theta );
		return target_theta;
	}

	const Vector2 pointLaser2World(const Vector2& pt_laser, const Vector3& robot_world)
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
		rotate << ::cos(robot_world[2]), -::sin(robot_world[2]),
			::sin(robot_world[2]), ::cos(robot_world[2]);

		Vector2 trans(robot_world[0], robot_world[1]);

		return rotate * pt_laser + trans;
	}

	const Vector2 pointWorld2Laser(const Vector2& pt_world, const Vector3& robot_world)
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
		rotate << ::cos(robot_world[2]), -::sin(robot_world[2]),
			::sin(robot_world[2]), ::cos(robot_world[2]);

		Vector2 trans(robot_world[0], robot_world[1]);

		return rotate.inverse() * pt_world - trans;
	}


private:
	APF<DataType> apf;
};
}

#endif