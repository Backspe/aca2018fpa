#include "fsm.h"

int timeCurr = 0, timeBasee = 0;

float surface(fMatrix points, float u, float v) {
	fMatrix oldpoints = points;
	while(oldpoints.size() > 1) {
		fMatrix newpoints;
		for(int i = 0; i < oldpoints.size() - 1; i++) {
			fVector row;
			for(int j = 0; j < oldpoints.size(); j++) {
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

std::vector<std::vector<float> > interpolateFrames(BVH* a, BVH* b, int cntA, int cntB, int frameCount = 60) {
	assert(a->joints.size() == b->joints.size());
	assert(1 <= cntA && cntA <= a->frames.size());
	assert(1 <= cntB && cntB <= b->frames.size());
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

FSM::FSM() {
	for(int i = 0; i < STATE_NUM; i++) {
		Parser parser;
		BVH* bvh = parser.parse(stateFile[i]);
		for(int j = 0; j < bvh->frames.size(); j++) {
			float w = j / (bvh->frames.size() - 1);
			bvh->frames[j][1] -= (bvh->frames.back()[1] - bvh->frames[0][1]) * w;
			bvh->frames[j][3] -= (bvh->frames.back()[3] - bvh->frames[0][3]) * w;
			bvh->frames[j][4] -= (bvh->frames.back()[4] - bvh->frames[0][4]) * w;
		}
		bvhs.push_back(bvh);
		motions.push_back(bvh->frames);
	}
	frameIndex = 0;
	stateCur = STAND;
	stateNext = STAND;
	isInterpolate = false;
	offset = glm::mat4(1.0f);

	std::vector<Motion> row1;
	row1.push_back(motions[6]);
	row1.push_back(motions[5]);
	row1.push_back(motions[7]);
	walkMotions.push_back(row1);
	std::vector<Motion> row2;
	row2.push_back(motions[9]);
	row2.push_back(motions[8]);
	row2.push_back(motions[10]);
	walkMotions.push_back(row2);
	std::vector<Motion> row3;
	row3.push_back(motions[12]);
	row3.push_back(motions[11]);
	row3.push_back(motions[13]);
	walkMotions.push_back(row3);
	
}

State FSM::setCommand() {
	char c = Camera::command;
	State ret = stateNext;
	if (c == 'w') {
		switch(stateCur) {
		case STAND:
			ret = START;
			break;
		case STRAIGHT1:
			ret = STRAIGHT2;
			break;
		case STRAIGHT2:
			ret = STRAIGHT3;
			break;
		default:
			break;
		}
	} else if (c == 's') {
		switch(stateCur) {
		case STRAIGHT1:
		case LEFT1:
		case RIGHT1:
			ret = STOP;
			break;
		case STRAIGHT2:
		case LEFT2:
		case RIGHT2:
			ret = STRAIGHT1;
			break;
		case STRAIGHT3:
		case LEFT3:
		case RIGHT3:
			ret = BREAK;
			break;
		default:
			break;
		}
	} else if (c == 'a') {
		switch(stateCur) {
		case STRAIGHT1:
		case LEFT1:
		case RIGHT1:
			ret = LEFT1;
			break;
		case STRAIGHT2:
		case LEFT2:
		case RIGHT2:
			ret = LEFT2;
			break;
		case STRAIGHT3:
		case LEFT3:
		case RIGHT3:
			ret = LEFT3;
			break;
		default:
			break;
		}
	} else if (c == 'd') {
		switch(stateCur) {
		case STRAIGHT1:
		case LEFT1:
		case RIGHT1:
			ret = RIGHT1;
			break;
		case STRAIGHT2:
		case LEFT2:
		case RIGHT2:
			ret = RIGHT2;
			break;
		case STRAIGHT3:
		case LEFT3:
		case RIGHT3:
			ret = RIGHT3;
			break;
		default:
			break;
		}
	} else if (c == 'f') {
		switch(stateCur) {
		case STAND:
			ret = JUMP1;
			break;
		}
	} else if (c == ' ') {
		switch(stateCur) {
		case STAND:
			ret = JUMP2;
			break;
		}
	} else if (c == 'e') {
		ret = NANNAN;
	}
	return ret;
}

void FSM::setOffset(Frame f1, Frame f2) {
	BVH* cur = bvhs[0];
	
	offset = glm::translate(offset, glm::vec3(
			f2[0], 
			f2[1], 
			f2[2])); 
	for(int i = 0; i < cur->channelCount; i++) {
		switch(cur->channelOrder[i]) {
		case 0:
			offset = glm::rotate(offset, 
					glm::radians(f2[i]),
					glm::vec3(1.0f, 0.0f, 0.0f)
					) ;
			break;
		case 1:
			offset = glm::rotate(offset, 
					glm::radians(f2[i]),
					glm::vec3(0.0f, 1.0f, 0.0f)
					) ;
			break;
		case 2:
			offset = glm::rotate(offset, 
					glm::radians(f2[i]),
					glm::vec3(0.0f, 0.0f, 1.0f)
					) ;
			break;
		}
	}
	for(int i = cur->channelCount - 1; i >= 0; i--) {
		switch(cur->channelOrder[i]) {
		case 0:
			offset = glm::rotate(offset, 
					-glm::radians(f1[i]),
					glm::vec3(1.0f, 0.0f, 0.0f)
					) ;
			break;
		case 1:
			offset = glm::rotate(offset, 
					-glm::radians(f1[i]),
					glm::vec3(0.0f, 1.0f, 0.0f)
					) ;
			break;
		case 2:
			offset = glm::rotate(offset, 
					-glm::radians(f1[i]),
					glm::vec3(0.0f, 0.0f, 1.0f)
					) ;
			break;
		}
	}
	offset = glm::translate(offset, glm::vec3(
			-f1[0], 
			-f1[1], 
			-f1[2])); 
}

void FSM::idle() {

	timeCurr = glutGet(GLUT_ELAPSED_TIME);
	if(timeCurr - timeBasee >= bvhs[0]->frameTime * 1000) {
		timeBasee += bvhs[0]->frameTime * 1000;
		frameIndex++;
	}
	frameIndex++; //TODO 시간 고려하기
	if (stateNext == NANNAN && stateCur != NANNAN) {
		stateCur = NANNAN;
		interMotion = interpolateMotions(
				walkMotions,					
				Camera::walkVelocity, Camera::walkAngle);
		frameIndex = 0;
		printf("u, v = %f, %f\n", Camera::walkVelocity, Camera::walkAngle);
	}
	if (stateCur == NANNAN) {
		isInterpolate = true;
		if (frameIndex >= interMotion.size()) {
			interMotion = interpolateMotions(
					walkMotions,					
					Camera::walkVelocity, Camera::walkAngle);
			frameIndex = 0;
			printf("u, v = %f, %f\n", Camera::walkVelocity, Camera::walkAngle);
		}
		if (Camera::command2 == 'x') {
			Camera::cov = glm::vec3(offset[3][0], offset[3][1], offset[3][2]);
			Camera::command2 = '\0';
		}
		return;
	}
	if (isInterpolate && frameIndex >= interMotion.size()) {
		frameIndex = interpolateFrameTable[stateNext][0];
		setOffset(motions[stateNext][frameIndex], interMotion.back());
		stateCur = stateNext;
		isInterpolate = false;
		switch(stateCur) {
		case STAND:
		case JUMP1:
		case JUMP2:
		case STOP:
		case BREAK:
			stateNext = STAND;
			break;
		case START:
		case STRAIGHT1:
		case LEFT1:
		case RIGHT1:
			stateNext = STRAIGHT1;
			break;
		case STRAIGHT2:
		case LEFT2:
		case RIGHT2:
			stateNext = STRAIGHT2;
			break;
		case STRAIGHT3:
		case LEFT3:
		case RIGHT3:
			stateNext = STRAIGHT3;
			break;
		}
	}
	if (!isInterpolate) {
		if (frameIndex < interpolateFrameTable[stateCur][1]) {
			//get new command
			State nxt = setCommand();
			if (nxt != stateNext && frameIndex < interpolateFrameTable[stateCur][1]) {
				stateNext = nxt;
				Camera::command = '\0';
			}
		} else {
			isInterpolate = true;
			int interLength;
			if (int(motions[stateCur].size() - interpolateFrameTable[stateCur][1]) > interpolateFrameTable[stateNext][0]) {
				interLength = int(motions[stateCur].size() - interpolateFrameTable[stateCur][1]);
			} else {
				interLength = interpolateFrameTable[stateNext][0];
			}
			interMotion = interpolateFrames(
					bvhs[stateCur], bvhs[stateNext], 
					motions[stateCur].size() - interpolateFrameTable[stateCur][1], 
					interpolateFrameTable[stateNext][0], 
					interLength);
			frameIndex = 0;
		}
	}
	if (Camera::command2 == 'x') {
		Camera::cov = glm::vec3(offset[3][0], offset[3][1], offset[3][2]);
		Camera::command2 = '\0';
	}
}

std::vector< float > FSM::getFrame() {
	if (!isInterpolate) {
		return motions[stateCur][frameIndex];		
	} else {
		return interMotion[frameIndex];		
	}

}
