#ifndef __POSE_GRAPH_OPTIMIZE_H
#define __POSE_GRAPH_OPTIMIZE_H

#include "nano_pgo.hpp"

namespace pgo
{

template<typename T>
class GraphOptimizer
{
public:
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Matrix3x3 = typename Eigen::Matrix<T, 3, 3>;

	void addVertexesAndEdges( const std::vector<Vector3>& key_poses )
	{
		for (int i = 0; i < key_poses.size() - 1; i++) {
			optimizer.addVertex(key_poses[i], i);
		//	std::cout << "key pose[" << i << "] = " << key_poses[i].transpose() << std::endl;

			Vector3 V = optimizer.homogeneousCoordinateTransformation(key_poses[i], key_poses[i + 1]);

			Matrix3x3 info_matrix = Matrix3x3::Identity();

			optimizer.addEdge(V, i, i + 1, info_matrix);
		}
		optimizer.addVertex(key_poses[key_poses.size() - 1], key_poses.size() - 1);
	}
	
	void addLoopConstraint( const Vector3& pose )	
	{
		Vector3 V( pose[0] * 0.001, pose[1] * 0.001, pose[2] )
		
		Matrix3x3 info_matrix = Matrix3x3::Identity();

		optimizer.addEdge(V, 0, key_poses.size() - 1, info_matrix);
		std::cout << "loop closure edge : " << V.transpose() << ", vertex : " << 0 << " to : " << key_poses.size() - 1 << std::endl;
	}

	const Vector3 runPGO( std::vector<Vector3>& vertex_poses )
	{
		optimizer.execuGraphOptimization(1);
		
		vertex_poses = optimizer.getReultVertexPosesVector();
		
		Vector3 ret = vertex_poses.back();	
	
		optimizer.clear();
		
		return ret;
	}

private:
	pgo::GraphOptimizer<T> optimizer;
};

}

#endif
