#include "Mesh.h"
#include "GridMesh.h"
#include <map>
#include <utility>
#include <iostream>

using namespace std;
using namespace FEM;

typedef pair<int,int> ip;

class ClothMesh : public Mesh {
public:
	//std::vector<Eigen::Vector3d> mParticles;
	//std::vector<Eigen::Vector3d> mSprings;
	//GetParticles(), GetSprings(), Clear()
	std::vector<Eigen::Vector3d> mpp;
	std::vector<std::vector<int> > mSquares;
	std::vector<std::vector<int> > GetSquares(){return mSquares;}
	map<ip,int> d;
	map<ip,int> sp;
	int n, m;
	void f(int a, int b) {
		d[ip(a,b)]=d[ip(b,a)]=1;
		cout << a << ", "<< b<< endl;
	}
	ClothMesh();
};
