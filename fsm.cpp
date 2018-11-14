#include "fsm.h"

FSM::FSM() {
	for(int i = 0; i < STATE_NUM; i++) {
		Parser parser;
		bvhs.push_back(parser.parse(stateFile[i]));
		motions.push_back(bvhs[i]->frames);
		for(int j = 0; j < STATE_NUM; j++) {
			interpolateFrameTable[i][j] = motions[i].size();
		}
	}
	frameIndex = 0;
	stateCur = STAND;
	stateNext = STAND;
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
	}
	return ret;
}

void FSM::idle() {
	frameIndex++; //TODO 시간 고려하기
	if (frameIndex >= motions[stateCur].size()) {
		frameIndex -= motions[stateCur].size();
		//TODO offset 바꾸기?
		stateCur = stateNext;
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
	if (frameIndex < interpolateFrameTable[stateCur][stateNext]) {
		State nxt = setCommand();
		if (nxt != stateNext && frameIndex < interpolateFrameTable[stateCur][nxt]) {
			stateNext = nxt;
			Camera::command = '\0';
		}
	} else {
		//TODO Interpolate stateCur and stateNext
	}
}

std::vector< float > FSM::getFrame() {
	return motions[stateCur][frameIndex];		
}
