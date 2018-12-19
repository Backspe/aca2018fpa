#include "Mesh.h"
#include "GridMesh.h"
#include <map>
#include <utility>

using namespace std;
using namespace FEM;

typedef pair<int,int> ip;

class ClothMesh : public Mesh {
public:
	//std::vector<Eigen::Vector3d> mParticles;
	//std::vector<Eigen::Vector3d> mSprings;
	//GetParticles(), GetSprings(), Clear()
	std::vector<std::vector<int> > mSquares;
	std::vector<std::vector<int> > GetSquares(){return mSquares;}
	map<ip,int> d;
	map<ip,int> sp;
	void f(int a, int b) {
		d[ip(a,b)]=d[ip(b,a)]=1;
	}
	ClothMesh() {
		GridMesh* m1 = new GridMesh(10,10,100.0,100.0,Eigen::Vector3d(8.89406 - 100.0,141.677,-92.409));
		GridMesh* m2 = new GridMesh(10,10,100.0,100.0,Eigen::Vector3d(8.89406,141.677,-92.409));
		const std::vector<Eigen::Vector3d> &p1=m1->GetParticles(), &p2=m2->GetParticles();
		const std::vector<Eigen::Vector2d> &s1=m1->GetSprings(), &s2=m2->GetSprings();
		for(int i=0;i<p1.size();i++)
			mParticles.push_back(p1[i]);
		for(int i=0;i<p2.size();i++)
			mParticles.push_back(p2[i]);
		for(int i=0;i<s1.size();i++)
			mSprings.push_back(s1[i]);
		for(int i=0;i<s2.size();i++) {
			Eigen::Vector2d d;
			d[0] = s2[i][0] + 100; d[1] = s2[i][1] + 100;
			mSprings.push_back(d);
		}
		mParticles[94][1] -= 10000.0;
		mParticles[95][1] -= 10000.0;
		mParticles[104][1] -= 10000.0;
		mParticles[105][1] -= 10000.0;
		for(int j=0;j<10;j++) {
			if(j<=3 || j>=6) {
				Eigen::Vector2d d; d[0] = 90+j; d[1]=100+j;
				mSprings.push_back(d);
				if(j!=0 && j!=6) {
					d[0] = 90+j-1; d[1]=100+j;
					mSprings.push_back(d);
					d[0] = 90+j; d[1]=99+j;
					mSprings.push_back(d);
				}
			}
		}
		f(83,94); f(84,93); f(84,95); f(85,94); f(85,96); f(86,95);
		f(103,114); f(104,113); f(104,115); f(105,114); f(105,116); f(106,115);
		f(84,94); f(85,95); f(104,114); f(105,115); f(93,94); f(94,95); f(95,96);
		f(103,104); f(104,105); f(105,106);
		for(int i=0;i<(int)mSprings.size();i++) {
			ip t=ip(mSprings[i][0],mSprings[i][1]);
			if(d.count(t)) {
				mSprings.erase(mSprings.begin()+i);
				i--;
			}
		}
		for(int i=0;i<9;i++) {
			Eigen::Vector2d d;
			d[0] = i*10; d[1]=(19-i)*10;
			mSprings.push_back(d);
			d[0] = i*10+9; d[1]=(19-i)*10+9;
			mSprings.push_back(d);
		}
		for(int i=1;i<=9;i++) {
			Eigen::Vector2d d;
			d[0] = i*10; d[1]=(20-i)*10;
			mSprings.push_back(d);
			d[0] = (i-1)*10; d[1]=(19-i)*10;
			mSprings.push_back(d);
			d[0] = i*10+9; d[1]=(20-i)*10+9;
			mSprings.push_back(d);
			d[0] = (i-1)*10+9; d[1]=(19-i)*10+9;
			mSprings.push_back(d);
		}
		for(int i=1;i<=9;i++) {
			std::vector<int> sq;
			sq.push_back(i*10); sq.push_back((i-1)*10);
			sq.push_back((20-i)*10); sq.push_back((19-i)*10);
			mSquares.push_back(sq);
			for(int j=0;j<sq.size();j++)
				sq[j]+=9;
			mSquares.push_back(sq);
		}
		for(int i=0;i<mSprings.size();i++)
			sp[ip(mSprings[i][0],mSprings[i][1])]=sp[ip(mSprings[i][1],mSprings[i][0])]=true;
		for(int i=0;i<20;i++) {
			for(int j=0;j<10;j++) {
				if(sp.count(ip(i*10+j,i*10+j+1)) && sp.count(ip((i+1)*10+j,(i+1)*10+j+1)) &&
				sp.count(ip(i*10+j,(i+1)*10+j)) && sp.count(ip(i*10+j+1,(i+1)*10+j+1))) {
					std::vector<int> sq;
					sq.push_back(i*10+j); sq.push_back(i*10+j+1);
					sq.push_back((i+1)*10+j+1); sq.push_back((i+1)*10+j);
					mSquares.push_back(sq);
				}
			}
		}
	}
};
