#include "camera.h"

vec3 Camera::cov = vec3(0.0f, 100.0f, 0.0f);
vec3 Camera::cameraPlaneVec = vec3(0.0, 0.0, 1.0);
float Camera::cameraDistance = 300;
vec3 Camera::cameraUpVec = vec3(0.0, 1.0, 0.0);
float Camera::fov = 45;
quat Camera::trackballRot = glm::quat();
vec3 Camera::v1 = glm::vec3(1, 0, 0);
vec3 Camera::v2 = glm::vec3(1, 0, 0);
vec3 Camera::modelRect[8] = { vec3(-100, 100-100, -100), vec3(-100, 100-100, 100), vec3(100, 100-100, 100), vec3(100, 100-100, -100),
vec3(-100, 100, -100), vec3(-100, 100, 100), vec3(100, 100, 100), vec3(100, 100, -100) };

bool Camera::leftButton = false;
float Camera::mousePosX = 0.0;
float Camera::mousePosY = 0.0;

float Camera::movementScale = 0.2f;
vec3 Camera::movement = vec3(0.0f, 0.0f, 0.0f);
std::string Camera::selectedJointName = "ltoes";
int Camera::maxDepth = 2;

// 창 크기 조절 및 카메라 위치 및 방향 조절
void Camera::resize(int w, int h)
{
	if (h == 0) h = 1;
	if (w == 0) w = 1;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0, 0, w, h);
	gluPerspective(fov, 1.0 * w / h, cameraDistance/100, cameraDistance*10);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glm::quat rot = glm::rotation(glm::normalize(v2), glm::normalize(v1));
	vec3 nv = glm::rotate(rot, cameraPlaneVec);
	vec3 uv = glm::rotate(rot, cameraUpVec);
	gluLookAt(cov.x + cameraDistance * nv.x, cov.y + cameraDistance * nv.y, cov.z + cameraDistance * nv.z,
		cov.x, cov.y, cov.z,
		uv.x, uv.y, uv.z);
}

// 마우스 드래그 시 트랙볼 회전
void Camera::mouseDragHandler(int x, int y) {
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	double r = cameraDistance * sin(glm::radians(fov / 2));
	double d, theta;
	if (leftButton) {
		//printf("%d %d\n", x, y);
		//v1
		double dx = (x - w / 2.0) / (h / 2.0);
		double dy = (h / 2.0 - y) / (h / 2.0);
		d = sqrt(dx * dx + dy * dy);
		if (dx * dx + dy * dy > 1.0) {
			dx /= d;
			dy /= d;
			d = 1.0;
		}
		theta = atan(r * d / cameraDistance);
		double l1 = cameraDistance * sin(theta);
		if (l1 > r) l1 = r;
		double l2 = sqrt(r * r - l1 * l1);
		v2 = vec3((l1 * cos(theta) - l2 * sin(theta)) * dx / d, (l1 * cos(theta) - l2 * sin(theta)) * dy / d, l1 * sin(theta) + l2 * cos(theta));
		//printf("r: %f, theta = %f, %f %f -> %f %f %f\n", r, glm::degrees(theta), l1, l2, v2.x, v2.y, v2.z);
		v2 = glm::rotate(trackballRot, v2);

		
		mousePosX = x;
		mousePosY = y;
	}
}

// 마우스 클릭 시 트랙볼 회전 초기 위치 계산
void Camera::mouseHandler(int button, int state, int x, int y) {
	switch (button) 
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN) {
			mousePosX = x;
			mousePosY = y;

			leftButton = true;
			glm::quat rot1 = glm::rotation(glm::normalize(v2), glm::normalize(v1));
			//printf("quat: %f %f %f %f\n", glm::angle(rot1), glm::axis(rot1).x, glm::axis(rot1).y, glm::axis(rot1).z);
			trackballRot = rot1 * trackballRot;
			cameraPlaneVec = glm::rotate(rot1, cameraPlaneVec);
			cameraUpVec = glm::rotate(rot1, cameraUpVec);
			

			int w = glutGet(GLUT_WINDOW_WIDTH);
			int h = glutGet(GLUT_WINDOW_HEIGHT);
			double d, theta;
			double r = cameraDistance * sin(glm::radians(fov / 2));
			double dx = (x - w / 2.0) / (h / 2.0);
			double dy = (h / 2.0 - y) / (h / 2.0);
			d = sqrt(dx * dx + dy * dy);
			if (dx * dx + dy * dy > 1.0) {
				dx /= d;
				dy /= d;
				d = 1.0;
			}
			theta = atan(r * d / cameraDistance);
			//printf("d(%f): (%f, %f)\n", d, dx, dy);
			double l1 = cameraDistance * sin(theta);
			if (l1 > r) l1 = r;
			double l2 = sqrt(r * r - l1 * l1);
			v1 = vec3((l1 * cos(theta) - l2 * sin(theta)) * dx / d, (l1 * cos(theta) - l2 * sin(theta)) * dy / d, l1 * sin(theta) + l2 * cos(theta));
			//printf("r: %f, theta = %f, %f %f -> %f %f %f\n", r, glm::degrees(theta), l1, l2, v1.x, v1.y, v1.z);
			
			v1 = glm::rotate(trackballRot, v1);
			v2 = v1;
		}
		if (state == GLUT_UP) {
			leftButton = false;
			//printf("end\n");
		}
	}
}

//시점변환 및 fov 위치 변경
void Camera::specialKeyboardHandler(int key, int x, int y) {
	switch (key)
	{
	case GLUT_KEY_PAGE_UP:
		fov -= 1;
		if (fov < 1) fov = 1;
		break;
	case GLUT_KEY_PAGE_DOWN:
		fov += 1;
		if (fov > 170) fov = 170;
		break;
	case GLUT_KEY_HOME:
		cameraDistance -= 1;
		if (cameraDistance < 1) cameraDistance = 1;
		break;
	case GLUT_KEY_END:
		cameraDistance += 1;
		if (cameraDistance > 1000) cameraDistance = 1000;
		break;
	case GLUT_KEY_DOWN:
		cov = cov - cameraUpVec * 0.5f;
		break;
	case GLUT_KEY_UP:
		cov = cov + cameraUpVec * 0.5f;
		break;
	case GLUT_KEY_LEFT:
		cov = cov - cross(cameraUpVec, cameraPlaneVec) * 0.5f;
		break;
	case GLUT_KEY_RIGHT:
		cov = cov + cross(cameraUpVec, cameraPlaneVec) * 0.5f;
		break;
	default:
		break;
	}
}

void Camera::keyboardHandler(unsigned char key, int x, int y) {
	float newDis = 0.1f;
	switch (key)
	{
	case 'p':
		for (int i = 0; i < 8; i++) {
			glm::vec3 v = modelRect[i] - cov;
			float t = glm::dot(v, normalize(cameraPlaneVec)) + glm::length(glm::cross(v, normalize(cameraPlaneVec))) / tan(glm::radians(fov/2));
			if (t > newDis) newDis = t;
		}
		cameraDistance = newDis;
		break;
	case 'w':
		movement = cameraUpVec * movementScale;		
		break;
	case 'a':
		movement = -cross(cameraUpVec, cameraPlaneVec) * movementScale;
		break;
	case 's':
		movement = -cameraUpVec * movementScale;
		break;
	case 'd':
		movement = cross(cameraUpVec, cameraPlaneVec) * movementScale;
		break;
	case 'b':
		selectedJointName = "rtoes";
		break;
	case 'n':
		selectedJointName = "rfoot";
		break;
	case '.':
		selectedJointName = "lfoot";
		break;
	case '/':
		selectedJointName = "ltoes";
		break;
	case 'g':
		selectedJointName = "rhand";
		break;
	case 'h':
		selectedJointName = "rradius";
		break;
	case 'j':
		selectedJointName = "rhumerus";
		break;
	case 'l':
		selectedJointName = "lhumerus";
		break;
	case ';':
		selectedJointName = "lradius";
		break;
	case '\'':
		selectedJointName = "lhand";
		break;
	case '2':
	case '3':
	case '4':
		maxDepth = key - '0';
		break;
	default:
		movement = glm::vec3(0.0f, 0.0f, 0.0f);		
		break;
	}
}
