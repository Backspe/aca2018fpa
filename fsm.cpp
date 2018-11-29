#include "fsm.h"

int timeCur = 0, timeBase = 0;
std::vector< std::vector< Motion > > walkMotions;

#define STEP_RATIO 0.45


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

	timeCur = glutGet(GLUT_ELAPSED_TIME) / 2;
	if(timeCur - timeBase >= bvhs[0]->frameTime * 1000) {
		timeBase += bvhs[0]->frameTime * 1000;
		frameIndex++;
		printf("u, v = %f, %f\n", Camera::walkVelocity, Camera::walkAngle);
	}
//	frameIndex++;
	if (stateNext == NANNAN && stateCur != NANNAN) {
		stateCur = NANNAN;
		blendMotion = interpolateMotions(
				walkMotions,					
				Camera::walkVelocity, Camera::walkAngle);
		frameIndex = 0;
	}
	if (stateCur == NANNAN) {
		if (!isInterpolate) {
			if (frameIndex >= blendMotion.size() - blendMotion.size() * STEP_RATIO) {
				isInterpolate = true;
				Frame curLast = blendMotion[blendMotion.size() - blendMotion.size() * STEP_RATIO];
				Motion prevMotion = blendMotion;
				blendMotion = interpolateMotions(
						walkMotions,					
						Camera::walkVelocity, Camera::walkAngle);
				interMotion = interpolateFrames(	
					prevMotion, blendMotion,
					prevMotion.size() * STEP_RATIO, blendMotion.size() * STEP_RATIO,
					prevMotion.size() * STEP_RATIO / 2 + blendMotion.size() * STEP_RATIO / 2);


				double offset_b[3], offset_temp[3], offset_n[3];
				for(int j = 0; j < 3; j++) {
					offset_b[j] = interMotion.back()[j+3];
					offset_temp[j] = blendMotion[blendMotion.size() * STEP_RATIO - 1][j+3];
				}
				auto mat_b = eulerAngleXYZ(
						glm::radians(offset_b[0]), 
						glm::radians(offset_b[1]), 
						glm::radians(offset_b[2]));
				auto mat_n = eulerAngleXYZ(
						glm::radians(offset_temp[0]), 
						glm::radians(offset_temp[1]), 
						glm::radians(offset_temp[2]));
				double angle_b = glm::atan(mat_b[0][1], mat_b[0][0]);
				double angle_n = glm::atan(mat_n[0][1], mat_n[0][0]);
				auto mat_rot = eulerAngleZ(angle_b - angle_n);
				glm::extractEulerAngleXYZ(mat_rot * mat_n, offset_n[0], offset_n[1], offset_n[2]);
				for(int j = 0; j < 3; j++) {
					offset_n[j] = glm::degrees(offset_n[j]);
				}
				printf("turn angle: %f\n", glm::degrees(angle_b - angle_n));
				printf("offset_b: %f %f %f\n", offset_b[0], offset_b[1], offset_b[2]);
				printf("offset_temp: %f %f %f\n", offset_temp[0], offset_temp[1], offset_temp[2]);
				printf("offset_n: %f %f %f\n", offset_n[0], offset_n[1], offset_n[2]);
				printf("offset: %f %f %f\n", offset[1][0], offset[1][1], offset[1][2]);

				for(int i = 0; i < interMotion.size(); i++) {
					for(int j = 0; j < 3; j++) {
						interMotion[i][j+3] += (offset_n[j] - offset_b[j]) * i / (interMotion.size() - 1);
					}
				}
				
				puts("set offset 1");
				setOffset(interMotion[0], curLast);
				frameIndex = 0;
				printf("u, v = %f, %f\n", Camera::walkVelocity, Camera::walkAngle);
			}
		} else {
			if (frameIndex >= interMotion.size()) {
				frameIndex = blendMotion.size() * STEP_RATIO;
				Frame curLast = interMotion.back();
				puts("set offset 2");
				setOffset(blendMotion[frameIndex - 1], curLast);
				isInterpolate = false;
			}
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

Motion FSM::getMotion() {
	if (!isInterpolate) {
		if(stateCur == NANNAN) {
			return blendMotion;
		} else {
			return motions[stateCur];		
		}
	} else {
		return interMotion;
	}

}

std::vector< float > FSM::getFrame() {
	if (!isInterpolate) {
		if(stateCur == NANNAN) {
			return blendMotion[frameIndex];
		} else {
			return motions[stateCur][frameIndex];		
		}
	} else {
		return interMotion[frameIndex];
	}

}
