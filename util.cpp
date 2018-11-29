#include "util.h"

//copy from http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon)
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

glm::mat4 fk(BVH* bvh, Joint* current, Frame frame) {	
	
	glm::mat4 pos = glm::mat4(1.0f);
	glm::mat4 defaultmat = glm::mat4(1.0f);
	pos = glm::translate(defaultmat, 
			glm::vec3(current->offset[0], current->offset[1], current->offset[2])
			) * pos;

	
	Joint* par = current->parent;
	while(par != NULL) {
		for(int i = par->channelCount - 1; i >= 0; i--) {
			switch(par->channelOrder[i]) {
				case 0:
					pos = glm::rotate(defaultmat, 
							glm::radians(frame[par->channelNum + i]),
							glm::vec3(1.0f, 0.0f, 0.0f)
							) * pos;
					break;
				case 1:
					pos = glm::rotate(defaultmat, 
							glm::radians(frame[par->channelNum + i]),
							glm::vec3(0.0f, 1.0f, 0.0f)
							) * pos;
					break;
				case 2:
					pos = glm::rotate(defaultmat, 
							glm::radians(frame[par->channelNum + i]),
							glm::vec3(0.0f, 0.0f, 1.0f)
							) * pos;
					break;
				case 3:
					pos = glm::translate(defaultmat, 
							glm::vec3(frame[par->channelNum + i], 0.0f, 0.0f)
							) * pos;
					break;
				case 4:
					pos = glm::translate(defaultmat, 
							glm::vec3(0.0f, frame[par->channelNum + i], 0.0f)
							) * pos;
					break;
				case 5:
					pos = glm::translate(defaultmat, 
							glm::vec3(0.0f, 0.0f, frame[par->channelNum + i])
							) * pos;
					break;
				default:
					printf("error!\n");
					exit(1);
			}
		}
		pos = glm::translate(defaultmat, 
				glm::vec3(par->offset[0], par->offset[1], par->offset[2])
				) * pos;

		par = par->parent;
	}
	for(int i = bvh->channelCount - 1; i >= 0; i--) {
		switch(bvh->channelOrder[i]) {
			case 0:
				pos = glm::rotate(defaultmat, 
						glm::radians(frame[bvh->channelNum + i]),
						glm::vec3(1.0f, 0.0f, 0.0f)
						) * pos;
				break;
			case 1:
				pos = glm::rotate(defaultmat, 
						glm::radians(frame[bvh->channelNum + i]),
						glm::vec3(0.0f, 1.0f, 0.0f)
						) * pos;
				break;
			case 2:
				pos = glm::rotate(defaultmat, 
						glm::radians(frame[bvh->channelNum + i]),
						glm::vec3(0.0f, 0.0f, 1.0f)
						) * pos;
				break;
			case 3:
				pos = glm::translate(defaultmat, 
						glm::vec3(frame[bvh->channelNum + i], 0.0f, 0.0f)
						) * pos;
				break;
			case 4:
				pos = glm::translate(defaultmat, 
						glm::vec3(0.0f, frame[bvh->channelNum + i], 0.0f)
						) * pos;
				break;
			case 5:
				pos = glm::translate(defaultmat, 
						glm::vec3(0.0f, 0.0f, frame[bvh->channelNum + i])
						) * pos;
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}
	pos = glm::translate(defaultmat, 
			glm::vec3(bvh->offset[0], bvh->offset[1], bvh->offset[2])
			) * pos;
	//std::cout << current->name << " " << pos[3][0] << " " << pos[3][1] << " " << pos[3][2] << " " << pos[3][3] << std::endl;

	return pos;
}

void ik(BVH* bvh, Joint* current, Frame frame) {
	glm::mat4 curMat = fk(bvh, current, frame);
	Joint* par = current->parent;
	std::vector< glm::vec3 > jacobianList;
	
	std::vector< int > thetas;
	std::vector< double > dthetas;
	int depth = 1;
	while(par != NULL && depth <= Camera::maxDepth) {
		glm::mat4 parMat = fk(bvh, par, frame);
		glm::vec3 p = glm::vec3(curMat[3] - parMat[3]);
		for(int i = 0; i < par->channelCount; i++) {
			switch(par->channelOrder[i]) {
				case 0:
				case 1:
				case 2:
					jacobianList.push_back(glm::cross(vec3(parMat[par->channelOrder[i]]), p));
					thetas.push_back(par->channelNum + i);
					break;
				default:
					printf("error!\n");
					exit(1);
			}
		}
		par = par->parent;
		depth++;
	}
	int n = thetas.size();
	Eigen::MatrixXd jacobianMat(3, n);
	for(int i = 0; i < n; i++) {
		jacobianMat.col(i) << jacobianList[i].x, jacobianList[i].y, jacobianList[i].z;
	}
	Eigen::MatrixXd psj = pseudoInverse(jacobianMat);
	for(int i = 0; i < n; i++) {
		glm::vec3 r = glm::vec3(psj(i, 0), psj(i, 1), psj(i, 2));

		double dt = glm::degrees(glm::dot(r, Camera::movement));
		if(dt > 45 || dt < -45) { // avoid ill-condition
			std::cout << "ill-conditioned" << std::endl;
			break; 
		}
		dthetas.push_back(dt);
	}
	if(dthetas.size() == n) {
		for(int i = 0; i < n; i++) {
			frame[thetas[i]] += dthetas[i];
		}
	}
	
}

float surface(fMatrix points, float u, float v) {
	fMatrix oldpoints = points;
	while(oldpoints.size() > 1) {
		fMatrix newpoints;
		for(int i = 0; i < oldpoints.size() - 1; i++) {
			fVector row;
			for(int j = 0; j < oldpoints[0].size(); j++) {
				row.push_back((1 - u) * oldpoints[i][j] + u * oldpoints[i+1][j]);
			}
			newpoints.push_back(row);
		}
		oldpoints = newpoints;
	}
	fVector oldrow = oldpoints[0];
	while(oldrow.size() > 1) {
		fVector row;
		for(int i = 0; i < oldrow.size() - 1; i++) {
			row.push_back((1 - v) * oldrow[i] + v * oldrow[i+1]);
		}
		oldrow = row;
	}
	return oldrow[0];
}

Motion interpolateMotions(std::vector< std::vector< Motion > > motions, float u, float v) {
	
	fMatrix motionSizes;
	for(int i = 0; i < motions.size(); i++) {
		fVector row;
		for(int j = 0; j < motions[0].size(); j++) {
			row.push_back(float(motions[i][j].size()));
		}
		motionSizes.push_back(row);
	}

	int frameCount = int(surface(motionSizes, u, v));

	std::vector<std::vector<float> > ret;
	const double Pi = acos(-1.0);
	int preAid = 0, preBid = 0;
	for(int t = 0; t < frameCount; t++) {
		double w = 1 - (cos(Pi / frameCount * t) / 2.0 + 1.0 / 2.0);
		std::vector< std::vector< int > > frameIds;
		for(int i = 0; i < motions.size(); i++) {
			std::vector< int > row;
			for(int j = 0; j < motions[0].size(); j++) {
				int frameId = 1.0 * t / frameCount * motions[i][j].size();
				if (frameId >= motions[i][j].size()) frameId = motions[i][j].size() - 1;
				row.push_back(frameId);
			}
			frameIds.push_back(row);
		}
		std::vector<float> cur;
		std::vector<float> pre;
		for (int idx = 0; idx < motions[0][0][0].size(); idx++) {
			fMatrix fmat;
			for(int ii = 0; ii < motions.size(); ii++) {
				fVector row;
				for(int jj = 0; jj < motions[ii].size(); jj++) {
					row.push_back(motions[ii][jj][frameIds[ii][jj]][idx]);
				}
				fmat.push_back(row);
			}
			cur.push_back(surface(fmat, u, v));
		}
		ret.push_back(cur);
	}
	return ret;
}

Motion interpolateFrames(Motion a, Motion b, int cntA, int cntB, int frameCount) {
	Motion aFrame, bFrame;
	for (int i = (int)a.size() - cntA; i < (int)a.size(); i++) {
		aFrame.push_back(a[i]);
	}
	for (int i = 0; i < cntB; i++) {
		bFrame.push_back(b[i]);
	}
	if (aFrame.empty()) aFrame.push_back(a.back());
	if (bFrame.empty()) bFrame.push_back(b[0]);
	Motion ret;
	const double Pi = acos(-1.0);
	int preAid = 0, preBid = 0;
	for(int t = 0; t < frameCount; t++) {
		double w = 1 - (cos(Pi / (frameCount-1) * t) / 2.0 + 1.0 / 2.0);
		int aid = 1.0*t / (frameCount-1) * aFrame.size(), bid = 1.0*t / (frameCount-1) * bFrame.size();
		if (aid >= aFrame.size()) aid = aFrame.size() - 1;
		if (bid >= bFrame.size()) bid = bFrame.size() - 1;
		std::vector<float> cur;
		std::vector<float> pre;
		/*
		for(int j = 0; j < a->frames.back().size(); j++) {
			if (j == 0 || j == 2) cur.push_back(a->frames.back()[j]);
			else cur.push_back(a->frames.back()[j] * (1.0 - w) + b->frames[60][j] * w);
		}*/
		for (int j = 0; j < aFrame[aid].size(); j++) {
			if (j == 0 || j == 2 || j == 5) {
				if (ret.size() == 0) {
					cur.push_back(aFrame[aid][j]);
				} else {
					cur.push_back(
							ret.back()[j] +   
							(aFrame[aid][j] - aFrame[preAid][j]) * (1 - w) + 
							(bFrame[bid][j] - bFrame[preBid][j]) * w 
							);
					/*
					cur.push_back(
							aFrame[0][j] +   
							(aFrame[aid][j] - aFrame[0][j]) * (1 - w) + 
							(bFrame[bid][j] - bFrame[0][j]) * w 
							);
					*/
				}
			}
			else cur.push_back(aFrame[aid][j] * (1.0 - w) + bFrame[bid][j] * w);
		}
		ret.push_back(cur);
		preAid = aid;
		preBid = bid;
	}
	return ret;
}

std::vector<std::vector<float> > interpolateFrames(BVH* a, BVH* b, int cntA, int cntB, int frameCount) {
	std::vector<std::vector<float> > aFrame, bFrame;
	for (int i = (int)a->frames.size() - cntA; i < (int)a->frames.size(); i++) {
		aFrame.push_back(a->frames[i]);
	}
	for (int i = 0; i < cntB; i++) {
		bFrame.push_back(b->frames[i]);
	}
	if (aFrame.empty()) aFrame.push_back(a->frames.back());
	if (bFrame.empty()) bFrame.push_back(b->frames[0]);
	std::vector<std::vector<float> > ret;
	const double Pi = acos(-1.0);
	int preAid = 0, preBid = 0;
	for(int t = 0; t < frameCount; t++) {
		double w = 1 - (cos(Pi / frameCount * t) / 2.0 + 1.0 / 2.0);
		int aid = 1.0*t / frameCount * aFrame.size(), bid = 1.0*t / frameCount * bFrame.size();
		if (aid >= aFrame.size()) aid = aFrame.size() - 1;
		if (bid >= bFrame.size()) bid = bFrame.size() - 1;
		std::vector<float> cur;
		std::vector<float> pre;
		/*
		for(int j = 0; j < a->frames.back().size(); j++) {
			if (j == 0 || j == 2) cur.push_back(a->frames.back()[j]);
			else cur.push_back(a->frames.back()[j] * (1.0 - w) + b->frames[60][j] * w);
		}*/
		for (int j = 0; j < aFrame[aid].size(); j++) {
			if (j == 0 || j == 2 || j == 5) {
				if (ret.size() == 0) {
					cur.push_back(aFrame[aid][j]);
				} else {
					cur.push_back(
							ret.back()[j] +   
							(aFrame[aid][j] - aFrame[preAid][j]) * (1 - w) + 
							(bFrame[bid][j] - bFrame[preBid][j]) * w 
							);
				}
			}
			else cur.push_back(aFrame[aid][j] * (1.0 - w) + bFrame[bid][j] * w);
		}
		ret.push_back(cur);
		preAid = aid;
		preBid = bid;
	}
	return ret;
}
