#ifndef FSM_H
#define FSM_H
#include "readBvh.h"
#include "camera.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <vector>
#include <stdlib.h>

#define STATE_NUM 15
typedef std::vector< float > Frame;
typedef std::vector< Frame > Motion;

enum State {
	STAND, JUMP1, JUMP2, START, STOP, STRAIGHT1, LEFT1, RIGHT1, STRAIGHT2, LEFT2, RIGHT2, STRAIGHT3, LEFT3, RIGHT3, BREAK
};

const char stateFile[STATE_NUM][50] = {
		"MotionData/STAND.bvh",
		"MotionData/JUMP1.bvh",
		"MotionData/JUMP2.bvh",
		"MotionData/START.bvh",
		"MotionData/STOP.bvh",
		"MotionData/STRAIGHT1.bvh",
		"MotionData/LEFT1.bvh",
		"MotionData/RIGHT1.bvh",
		"MotionData/STRAIGHT2.bvh",
		"MotionData/LEFT2.bvh",
		"MotionData/RIGHT2.bvh",
		"MotionData/STRAIGHT3.bvh",
		"MotionData/LEFT3.bvh",
		"MotionData/RIGHT3.bvh",
		"MotionData/BREAK.bvh"
};

class FSM {
public:
	int frameIndex;
	State stateCur;
	State stateNext;
	bool isInterpolate;
	Motion interMotion;
	glm::mat4 offset;
	int interpolateFrameTable[STATE_NUM][2]; //이전 state의 몇번째 프레임부터 인터폴레이트하고 나중 스테이트의 몇 프레임까지를 interpolate할 지 저장하는 테이블
	std::vector< Motion > motions;
	std::vector< BVH* > bvhs;

	FSM();
	State setCommand();
	void setOffset(Frame f1, Frame f2);
	void idle();
	Frame getFrame();
};

#endif
