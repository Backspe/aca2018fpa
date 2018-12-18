#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "sim/fem/Constraint/ConstraintHeader.h"
#include "sim/fem/World.h"
#include "sim/Cloth.h"

int main2() {
	FEM::World*	mSoftWorld;
	Cloth* mCloth;
	mSoftWorld = new FEM::World(
		FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		FEM::OptimizationMethod::OPTIMIZATION_METHOD_NEWTON,
		FEM::LinearSolveType::SOLVER_TYPE_LDLT,
		1.0/100.0,										//time_step
		100, 											//max_iteration	
		0.99											//damping_coeff
		);

	mCloth = new Cloth();
	mCloth->Initialize(mSoftWorld);
	mSoftWorld->Initialize();
	const auto& particles = mSoftWorld->mX;
	for(int i = 0; i < 100; i++) {
		std::cout << particles.block<3,1>(3*i,0)[0] << ", " << particles.block<3,1>(3*i,0)[1] << ", " << particles.block<3,1>(3*i,0)[2] << std::endl;
	}
	mSoftWorld->TimeStepping();
	for(int i = 0; i < 100; i++) {
		std::cout << particles.block<3,1>(3*i,0)[0] << ", " << particles.block<3,1>(3*i,0)[1] << ", " << particles.block<3,1>(3*i,0)[2] << std::endl;
	}
}
