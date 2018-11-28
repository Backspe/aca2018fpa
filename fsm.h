#ifndef FSM_H
#define FSM_H
#include "readBvh.h"
#include "camera.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <vector>
#include <stdlib.h>

#define STATE_NUM 16
typedef std::vector< float > Frame;
typedef std::vector< Frame > Motion;
typedef std::vector< float > fVector;
typedef std::vector< std::vector< float > > fMatrix;

enum State {
	STAND, JUMP1, JUMP2, START, STOP, STRAIGHT1, LEFT1, RIGHT1, STRAIGHT2, LEFT2, RIGHT2, STRAIGHT3, LEFT3, RIGHT3, BREAK, NANNAN
};

const char stateFile[STATE_NUM][50] = {
		"MotionData/STAND.bvh",
		"MotionData/JUMP1_cut.bvh",
		"MotionData/JUMP2_cut.bvh",
		"MotionData/START_cut.bvh",
		"MotionData/STOP_cut.bvh",
		"MotionData/STRAIGHT1_cut.bvh",
		"MotionData/LEFT1_cut.bvh",
		"MotionData/RIGHT1_cut.bvh",
		"MotionData/STRAIGHT2_cut.bvh",
		"MotionData/LEFT2_cut.bvh",
		"MotionData/RIGHT2_cut.bvh",
		"MotionData/STRAIGHT3_cut.bvh",
		"MotionData/LEFT3_cut.bvh",
		"MotionData/RIGHT3_cut.bvh",
		"MotionData/BREAK_cut.bvh",
		"MotionData/STRAIGHT1_cut.bvh"
};


class FSM {
public:
	int frameIndex;
	State stateCur;
	State stateNext;
	bool isInterpolate;
	Motion interMotion, blendMotion;
	glm::mat4 offset;
	const char interpolateFrameTable[STATE_NUM][2]= {
		{1, 4},
		{15, 65},
		{20, 70},
		{10, 67},
		{33, 40},
		{31, 38},
		{30, 32},
		{34, 41}, 
		{26, 29},
		{22, 29},
		{24, 28},
		{23, 25},
		{12, 17},
		{22, 28},
		{20, 48},
		{31, 38}
	};
	//int interpolateFrameTable[STATE_NUM][2]; //이전 state의 몇번째 프레임부터 인터폴레이트하고 나중 스테이트의 몇 프레임까지를 interpolate할 지 저장하는 테이블
	std::vector< Motion > motions;
	std::vector< BVH* > bvhs;

	FSM();
	State setCommand();
	void setOffset(Frame f1, Frame f2);
	void idle();
	Frame getFrame();
};

#endif
