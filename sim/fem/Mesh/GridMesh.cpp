#include "Mesh.h"
#include "GridMesh.h"
#include <fstream>
#include <sstream>
#include <iostream>
using namespace FEM;
GridMesh::
GridMesh(const int& w_num, const int& h_num,
		const double& w_length, const double& h_length,
		const Eigen::Vector3d& origin,
		const Eigen::Vector3d& w_orientation,
		const Eigen::Vector3d& h_orientation,
		const Eigen::Affine3d& T
		)
	:Mesh()
{
	double w_grid = w_length / (w_num-1);
	double h_grid = h_length / (h_num-1);

	// Particles
	for(int i=0; i<w_num; i++) {
		for(int j=0; j<h_num; j++) {
			Eigen::Vector3d vertex_position;
			vertex_position[0] = origin[0]+w_grid*w_orientation[0]*i+h_grid*h_orientation[0]*j;
			vertex_position[1] = origin[1]+w_grid*w_orientation[1]*i+h_grid*h_orientation[1]*j;
			vertex_position[2] = origin[2]+w_grid*w_orientation[2]*i+h_grid*h_orientation[2]*j;
			vertex_position= T*vertex_position;
            mParticles.push_back(vertex_position);
		}
	}

	for(int i=0; i<w_num-1; i++) {
		for(int j=0; j<h_num-1; j++) {
			mSprings.push_back(Eigen::Vector2d(h_num*(i+0)+(j+0),h_num*(i+1)+(j+0)));
			mSprings.push_back(Eigen::Vector2d(h_num*(i+0)+(j+1),h_num*(i+0)+(j+0)));
		}
	}

	for(int i=0; i<w_num-1; i++) {
		mSprings.push_back(Eigen::Vector2d(h_num*(i+0)+h_num-1,h_num*(i+1)+h_num-1));
	}
	for(int j=0; j<h_num-1; j++) {
		mSprings.push_back(Eigen::Vector2d(h_num*(w_num-1)+j+1,h_num*(w_num-1)+j));
	}

    for(int i=0; i<w_num-1; i++) {
		for(int j=0; j<h_num-1; j++) {
			mSprings.push_back(Eigen::Vector2d(h_num*(i+0)+j,(h_num)*(i+1)+(j+1)));
		}
	}

	for(int i=0; i<w_num-1; i++) {
		for(int j=0; j<h_num-1; j++) {
			mSprings.push_back(Eigen::Vector2d(h_num*(i+0)+(j+1),(h_num)*(i+1)+(j+0)));
		}
	}
}
