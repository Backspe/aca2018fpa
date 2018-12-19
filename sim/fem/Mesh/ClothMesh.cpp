#include "ClothMesh.h"

ClothMesh::ClothMesh() {
	n=12; m=12;
	double nSz = 60.0, mSz = 40.0;
	double thick = 8.0;
	GridMesh* m1 = new GridMesh(n,m,nSz,mSz,Eigen::Vector3d(8.89406-mSz/2,141.677-nSz,-88.409+thick),Eigen::Vector3d(0.0,1.0,0.0),Eigen::Vector3d(1.0,0.0,0.0));
	GridMesh* m2 = new GridMesh(n,m,nSz,mSz,Eigen::Vector3d(8.89406-mSz/2,141.677,-88.409-thick),Eigen::Vector3d(0.0,-1.0,0.0),Eigen::Vector3d(1.0,0.0,0.0));
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
		d[0] = s2[i][0] + n*m; d[1] = s2[i][1] + n*m;
		mSprings.push_back(d);
	}
	mParticles[(n-1)*m+m/2-1][1] -= 10000.0;
	mParticles[(n-1)*m+m/2][1] -= 10000.0;
	mParticles[n*m+m/2-1][1] -= 10000.0;
	mParticles[n*m+m/2][1] -= 10000.0;
	int cnt = 0;
	for(int j=0;j<m;j++) {
		if(j<=m/2-2 || j>=m/2+1) {
			Eigen::Vector2d d; d[0] = (n-1)*m+j; d[1]=n*m+j;
			mSprings.push_back(d);
			cnt++;
			if(j!=0 && j!=m/2+1) {
				d[0] = (n-1)*m+j-1; d[1]=n*m+j;
				mSprings.push_back(d);
				d[0] = (n-1)*m+j; d[1]=n*m-1+j;
				mSprings.push_back(d);
				cnt+=2;
			}
		}
	}
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			f((n-2+i)*m + m/2-2+j, (n-1+i)*m + m/2-1+j);
			f((n-2+i)*m + m/2-1+j, (n-1+i)*m + m/2-2+j);
			if(i != 2) {
				f((n-1+i)*m + m/2-2+j, (n-1+i)*m + m/2-1+j);
			}
			if(j != 2) {
				f((n-2+i)*m + m/2-1+j, (n-1+i)*m + m/2-1+j);
			}
		}
	}
	for(int i=0;i<(int)mSprings.size();i++) {
		ip t=ip(mSprings[i][0],mSprings[i][1]);
		if(d.count(t)) {
			mSprings.erase(mSprings.begin()+i);
			i--;
		}
	}
	for(int i=0;i<n-1;i++) {
		Eigen::Vector2d d;
		d[0] = i*m; d[1]=(2*n-1-i)*m;
		mSprings.push_back(d);
		d[0] = i*m+m-1; d[1]=(2*n-1-i)*m+m-1;
		mSprings.push_back(d);
	}
	for(int i=1;i<=n-1;i++) {
		Eigen::Vector2d d;
		d[0] = i*m; d[1]=(2*n-i)*m;
		mSprings.push_back(d);
		d[0] = (i-1)*m; d[1]=(2*n-1-i)*m;
		mSprings.push_back(d);
		d[0] = i*m+m-1; d[1]=(2*n-i)*m+m-1;
		mSprings.push_back(d);
		d[0] = (i-1)*m+m-1; d[1]=(2*n-1-i)*m+m-1;
		mSprings.push_back(d);
	}
	for(int i=1;i<=n-1;i++) {
		std::vector<int> sq;
		sq.push_back(i*m); sq.push_back((i-1)*m);
		sq.push_back((2*n-i)*m); sq.push_back((2*n-1-i)*m);
		mSquares.push_back(sq);
		for(int j=0;j<sq.size();j++)
			sq[j]+=m-1;
		mSquares.push_back(sq);
	}
	for(int i=0;i<mSprings.size();i++)
		sp[ip(mSprings[i][0],mSprings[i][1])]=sp[ip(mSprings[i][1],mSprings[i][0])]=true;
	for(int i=0;i<2*n;i++) {
		for(int j=0;j<m;j++) {
			if(sp.count(ip(i*m+j,i*m+j+1)) && sp.count(ip((i+1)*m+j,(i+1)*m+j+1)) &&
			sp.count(ip(i*m+j,(i+1)*m+j)) && sp.count(ip(i*m+j+1,(i+1)*m+j+1))) {
				std::vector<int> sq;
				sq.push_back(i*m+j); sq.push_back(i*m+j+1);
				sq.push_back((i+1)*m+j+1); sq.push_back((i+1)*m+j);
				mSquares.push_back(sq);
			}
		}
	}
	/*
	for(int i=0;i<mSprings.size();i++) {
		if(mSprings[i][0] >= 2*n*m || mSprings[i][1]>= 2*n*m) {
			std::cout<<"i = "<<i<<std::endl;
			std::cout<<mSprings[i][0]<<" "<<mSprings[i][1]<<std::endl;
		}
	}
	*/
}
