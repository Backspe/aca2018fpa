#ifndef __CLOTH_H__
#define __CLOTH_H__
#include "fem/World.h"
#include "fem/Mesh/MeshHeader.h"
#include "fem/Constraint/ConstraintHeader.h"
#include "fem/Mesh/ClothMesh.h"
class Cloth 
{
public:
	Cloth();
	void Initialize(FEM::World* world);
	void SetMesh();
	ClothMesh* GetMesh() {return mMesh;};
	
private:
	std::vector<FEM::Constraint*>						mConstraints;
	ClothMesh*											mMesh;

	double 												mStretchingStiffness;
	double 												mBendingStiffness;
};

#endif
