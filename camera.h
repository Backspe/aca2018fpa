#ifndef CAMERA_H
#define CAMERA_H
#define GLM_ENABLE_EXPERIMENTAL
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/quaternion.hpp"
#define PI 3.14159265358979

using namespace glm;

// 카메라 관련 변수
class Camera {

private:
	static vec3 cov;
	static vec3 cameraPlaneVec;
	static float cameraDistance;
	static vec3 cameraUpVec;
	static float fov;
	static quat trackballRot;
	static vec3 v1;
	static vec3 v2;
	static vec3 modelRect[8];
	static bool leftButton;
	static float mousePosX, mousePosY;
	static float movementScale;

public:
	static void resize(int w, int h);
	static void mouseDragHandler(int x, int y);
	static void mouseHandler(int button, int state, int x, int y);
	static void specialKeyboardHandler(int key, int x, int y);
	static void keyboardHandler(unsigned char key, int x, int y);
	static vec3 movement;
	static std::string selectedJointName;
	static int maxDepth;
};

#endif
