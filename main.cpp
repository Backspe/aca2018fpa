#include "readBvh.h"
#include "camera.h"
#include "fsm.h"
#include "util.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/quaternion.hpp"
#include <GL/glut.h>

#include <math.h>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>

#include <unistd.h>

#define PI 3.14159265358979

#include "sim/fem/Constraint/ConstraintHeader.h"
#include "sim/fem/World.h"
#include "sim/Cloth.h"

using namespace glm;

float modelscale = 1.0f;
int slice = 20;


// 새 모델 크기 관련 변수

const double bodyWidth = 25;
const double bodyThick = 15;

const double thoraxRad = 3.0;

const double neckRad = 3.0;
const double headRad = 15.0;

const double armThick = 3.0;
const double handWidth = 5.0;
const double handThick = 1.5;
const double handLength = 10.0;

const double humerusRad = 4.0;
const double radiusRad = 3.0;
const double handRad = 1.5;

const double legThick = 5.0;
const double footWidth = 7.0;
const double footThick = 1.5;
const double toesLength = 8.0;

const double femurRad = 5.0;
const double tibiaRad = 4.0;
const double footRad = 1.5;
const double toesRad = 0.75;

const double eps = 0.1;
double world_timestep = 1.0/150.0;

// 각도 관련 변수

Camera camera = Camera();

FSM fsm = FSM();

BVH* bvh;
std::map< std::string, Joint* > jointMap;

int jointIndex = 0;
int frameIndex = 0;

std::vector< float > frameCur;

Joint* selectedJoint;

FEM::World*	mSoftWorld;
Cloth* mCloth;


// >< 눈
void eye(double rad) {
	glPushMatrix();
	glRotatef(1 * 360 / 20, 0.0f, 1.0f, 0.0f);
	glRotatef(-1 * 360 / 20, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f, 0.0f, rad);
	glRotatef(0.5 * 360 / slice, 0.0f, 1.0f, 0.0f);
	glColor3f(0.1f, 0.1f, 0.1f);
	glScalef(0.3, 0.2, 0.1);
	glScalef(rad, rad, rad);
	glBegin(GL_TRIANGLE_STRIP);
	glVertex3f(1.0f, -1.0f, 0);
	glVertex3f(1.0f, -0.9f, 0);
	glVertex3f(0.0f, 0.0f, 0);
	glVertex3f(0.1f, 0.0f, 0);
	glVertex3f(1.0f, 1.0f, 0);
	glVertex3f(1.0f, 0.9f, 0);
	glEnd();
	glPopMatrix();
}

void mouth(double rad) {
	glPushMatrix();
	glScalef(rad, rad, rad);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_TRIANGLE_STRIP);
	glVertex3f(-sin(2 * PI * -1.5 / slice), -0.25, cos(2 * PI * -1.5 / slice));
	glVertex3f(-sin(2 * PI * -1.5 / slice), -0.2, cos(2 * PI * -1.5 / slice));
	for (int i = -1; i <= 1; i++) {
		glVertex3f(-sin(2 * PI * i / slice), -0.05, cos(2 * PI * i / slice));
		glVertex3f(-sin(2 * PI * i / slice), -0.0, cos(2 * PI * i / slice));
		glVertex3f(-sin(2 * PI * (i + 0.5) / slice), -0.25, cos(2 * PI * (i + 0.5) / slice));
		glVertex3f(-sin(2 * PI * (i + 0.5) / slice), -0.2, cos(2 * PI * (i + 0.5) / slice));
	}
	glEnd();
	glPopMatrix();
}

std::vector< glm::mat4 > getCube(float width, float height, float thick, float offsetx, float offsety, float offsetz) {
	width = width / 2;
	thick = thick / 2;
	glm::mat4 mat = glm::mat4(1.0f);
	std::vector< glm::mat4 > ret;
	ret.push_back(glm::translate(mat, glm::vec3( width, 0.0f, thick)));
	ret.push_back(glm::translate(mat, glm::vec3(-width, 0.0f, thick)));
	ret.push_back(glm::translate(mat, glm::vec3(-width, 0.0f,-thick)));
	ret.push_back(glm::translate(mat, glm::vec3( width, 0.0f,-thick)));
	ret.push_back(glm::translate(mat, glm::vec3( width, height, thick)));
	ret.push_back(glm::translate(mat, glm::vec3( width, height,-thick)));
	ret.push_back(glm::translate(mat, glm::vec3(-width, height,-thick)));
	ret.push_back(glm::translate(mat, glm::vec3(-width, height, thick)));
	mat = glm::translate(mat, glm::vec3(offsetx, offsety, offsetz));
	return ret;
}

// make cube, pivot is centor of bottom
void drawCube(float width, float height, float thick, float offsetx, float offsety, float offsetz) {

	width = width / 2;
	thick = thick / 2;
	glPushMatrix();
	glTranslatef(offsetx, offsety, offsetz);
	glBegin( GL_QUADS );
	
	glVertex3f(-width, height, thick); // { Front }
	glVertex3f(-width, 0.0f, thick); // { Front }
	glVertex3f( width, 0.0f, thick); // { Front }
	glVertex3f( width, height, thick); // { Front }

	glVertex3f( width, height, thick); // { Right }
	glVertex3f( width, 0.0f, thick); // { Right }
	glVertex3f( width, 0.0f,-thick); // { Right }
	glVertex3f( width, height,-thick); // { Right }

	glVertex3f( width, height,-thick); // { Back }
	glVertex3f( width, 0.0f,-thick); // { Back }
	glVertex3f(-width, 0.0f,-thick); // { Back }
	glVertex3f(-width, height,-thick); // { Back }

	glVertex3f(-width, height,-thick); // { Left }
	glVertex3f(-width, 0.0f,-thick); // { Left }
	glVertex3f(-width, 0.0f, thick); // { Left }
	glVertex3f(-width, height, thick); // { Left }

	glVertex3f(-width, height, thick); // { Top }
	glVertex3f( width, height, thick); // { Top }
	glVertex3f( width, height,-thick); // { Top }
	glVertex3f(-width, height,-thick); // { Top }

	glVertex3f( width, 0.0f, thick); // { Bottom }
	glVertex3f(-width, 0.0f, thick); // { Bottom }
	glVertex3f(-width, 0.0f,-thick); // { Bottom }
	glVertex3f( width, 0.0f,-thick); // { Bottom }


    glEnd();	
	glPopMatrix();
}

void glmVertex(glm::mat4 &mat) {
	glm::vec4 v = mat * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glVertex3f(v.x, v.y, v.z);
}

void drawBody() {
	
	float width = bodyWidth / 2;
	float thick = bodyThick / 2;
	Joint* thorax = jointMap.find("thorax")->second;
	Joint* lclavicle = jointMap.find("lclavicle")->second;
	Joint* rclavicle = jointMap.find("rclavicle")->second;
	Joint* lhumerus = jointMap.find("lhumerus")->second;
	Joint* rhumerus = jointMap.find("rhumerus")->second;
	Joint* head = jointMap.find("head")->second;

	glm::mat4 mat = glm::mat4(1.0f);
	glm::mat4 bodyVertex[13];

	bodyVertex[0] = glm::translate(mat, glm::vec3(-width, 0.0f, thick));
	bodyVertex[1] = glm::translate(mat, glm::vec3( width, 0.0f, thick));
	bodyVertex[2] = glm::translate(mat, glm::vec3( width, 0.0f,-thick));
	bodyVertex[3] = glm::translate(mat, glm::vec3(-width, 0.0f,-thick));
	bodyVertex[4] = glm::translate(mat, glm::vec3(-width, thorax->offset[1], thick));
	bodyVertex[5] = glm::translate(mat, glm::vec3( width, thorax->offset[1], thick));
	mat = glm::translate(mat, glm::vec3(thorax->offset[0], thorax->offset[1], thorax->offset[2]));
	for(int i = 0; i < 3; i++) {
		switch(thorax->channelOrder[i]) {
			case 0:
				mat = glm::rotate(mat, glm::radians(frameCur[i+30]), glm::vec3(1.0f, 0.0f, 0.0f));
				break;
			case 1:
				mat = glm::rotate(mat, glm::radians(frameCur[i+30]), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				mat = glm::rotate(mat, glm::radians(frameCur[i+30]), glm::vec3(0.0f, 0.0f, 1.0f));
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}
	bodyVertex[6] = glm::translate(mat, glm::vec3( width, 0.0f,-thick));
	bodyVertex[7] = glm::translate(mat, glm::vec3(-width, 0.0f,-thick));
	bodyVertex[12] = glm::translate(mat, glm::vec3(head->offset[0], head->offset[1], head->offset[2]));
	glm::mat4 matl = glm::translate(mat, glm::vec3(lclavicle->offset[0], lclavicle->offset[1], lclavicle->offset[2]));
	for(int i = 0; i < 3; i++) {
		switch(lclavicle->channelOrder[i]) {
			case 0:
				matl = glm::rotate(matl, glm::radians(frameCur[i+36]), glm::vec3(1.0f, 0.0f, 0.0f));
				break;
			case 1:
				matl = glm::rotate(matl, glm::radians(frameCur[i+36]), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				matl = glm::rotate(matl, glm::radians(frameCur[i+36]), glm::vec3(0.0f, 0.0f, 1.0f));
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}
	matl = glm::translate(matl, glm::vec3(lhumerus->offset[0], lhumerus->offset[1], lhumerus->offset[2]));
	bodyVertex[ 9] = glm::translate(matl, glm::vec3(-humerusRad, 0.0f, thick));
	bodyVertex[10] = glm::translate(matl, glm::vec3(-humerusRad, 0.0f,-thick));

	glm::mat4 matr = glm::translate(mat, glm::vec3(rclavicle->offset[0], rclavicle->offset[1], rclavicle->offset[2]));
	for(int i = 0; i < 3; i++) {
		switch(rclavicle->channelOrder[i]) {
			case 0:
				matr = glm::rotate(matr, glm::radians(frameCur[i+48]), glm::vec3(1.0f, 0.0f, 0.0f));
				break;
			case 1:
				matr = glm::rotate(matr, glm::radians(frameCur[i+48]), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				matr = glm::rotate(matr, glm::radians(frameCur[i+48]), glm::vec3(0.0f, 0.0f, 1.0f));
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}
	matr = glm::translate(matr, glm::vec3(rhumerus->offset[0], rhumerus->offset[1], rhumerus->offset[2]));
	bodyVertex[ 8] = glm::translate(matr, glm::vec3( humerusRad, 0.0f, thick));
	bodyVertex[11] = glm::translate(matr, glm::vec3( humerusRad, 0.0f,-thick));
	
	glPushMatrix();
	glBegin( GL_QUADS );

	glmVertex(bodyVertex[3]);
	glmVertex(bodyVertex[2]);
	glmVertex(bodyVertex[1]);
	glmVertex(bodyVertex[0]);

	glmVertex(bodyVertex[0]);
	glmVertex(bodyVertex[1]);
	glmVertex(bodyVertex[5]);
	glmVertex(bodyVertex[4]);

	glmVertex(bodyVertex[1]);
	glmVertex(bodyVertex[2]);
	glmVertex(bodyVertex[6]);
	glmVertex(bodyVertex[5]);

	glmVertex(bodyVertex[2]);
	glmVertex(bodyVertex[3]);
	glmVertex(bodyVertex[7]);
	glmVertex(bodyVertex[6]);

	glmVertex(bodyVertex[3]);
	glmVertex(bodyVertex[0]);
	glmVertex(bodyVertex[4]);
	glmVertex(bodyVertex[7]);

	glmVertex(bodyVertex[8]);
	glmVertex(bodyVertex[9]);
	glmVertex(bodyVertex[5]);
	glmVertex(bodyVertex[4]);

	glmVertex(bodyVertex[9]);
	glmVertex(bodyVertex[10]);
	glmVertex(bodyVertex[6]);
	glmVertex(bodyVertex[5]);

	glmVertex(bodyVertex[10]);
	glmVertex(bodyVertex[11]);
	glmVertex(bodyVertex[7]);
	glmVertex(bodyVertex[6]);

	glmVertex(bodyVertex[11]);
	glmVertex(bodyVertex[8]);
	glmVertex(bodyVertex[4]);
	glmVertex(bodyVertex[7]);

	glmVertex(bodyVertex[8]);
	glmVertex(bodyVertex[9]);
	glmVertex(bodyVertex[10]);
	glmVertex(bodyVertex[11]);

    glEnd();	

	glBegin( GL_TRIANGLES );
	glmVertex(bodyVertex[8]);
	glmVertex(bodyVertex[9]);
	glmVertex(bodyVertex[12]);
	glmVertex(bodyVertex[9]);
	glmVertex(bodyVertex[10]);
	glmVertex(bodyVertex[12]);
	glmVertex(bodyVertex[10]);
	glmVertex(bodyVertex[11]);
	glmVertex(bodyVertex[12]);
	glmVertex(bodyVertex[11]);
	glmVertex(bodyVertex[8]);
	glmVertex(bodyVertex[12]);
    glEnd();	

	glPopMatrix();
}

std::vector< glm::mat4 > getJointRect(Joint* current) {
	std::vector< glm::mat4 > ret;
	if(current->name.compare("lfemur") == 0 || current->name.compare("rfemur") == 0) {
		ret = getCube(-legThick, jointMap.find("ltibia")->second->offset[1] + femurRad + tibiaRad, legThick, 0, -femurRad, 0);

	}
	if(current->name.compare("ltibia") == 0 || current->name.compare("rtibia") == 0) {
		ret = getCube(-legThick, jointMap.find("lfoot")->second->offset[1] + tibiaRad + footRad, legThick, 0, -tibiaRad, 0);
	}
	if(current->name.compare("lfoot") == 0 || current->name.compare("rfoot") == 0) {
		glm::mat4 matRot = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		ret = getCube(footWidth, jointMap.find("ltoes")->second->offset[2] - footRad - toesRad, footThick, 0, footRad, 0);
		for(int i = 0; i < ret.size(); i++) {
			ret[i] = matRot * ret[i];
		}
	}
	if(current->name.compare("ltoes") == 0 || current->name.compare("rtoes") == 0) {
		glm::mat4 matRot = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		ret = getCube(footWidth, toesLength - toesRad, footThick, 0, toesRad, 0);
		for(int i = 0; i < ret.size(); i++) {
			ret[i] = matRot * ret[i];
		}
	}

	if(current->name.compare("lhumerus") == 0 || current->name.compare("rhumerus") == 0) {
		ret = getCube(-armThick, jointMap.find("lradius")->second->offset[1] + humerusRad + radiusRad, armThick, 0, -humerusRad, 0);

	}
	if(current->name.compare("lradius") == 0 || current->name.compare("rradius") == 0) {
		ret = getCube(-armThick, jointMap.find("lhand")->second->offset[1] + radiusRad + handRad, armThick, 0, -radiusRad, 0);
	}
	if(current->name.compare("lhand") == 0 || current->name.compare("rhand") == 0) {
		ret = getCube(-handThick, -handLength + handRad, handWidth, 0, -handRad, 0);
	}
	if(current->name.compare("head") == 0) {
		ret = getCube(neckRad*2, neckRad + headRad, neckRad*2, 0, 0, 0);
	}
	if(current->name.compare("thorax") == 0) {
		ret = getCube(bodyWidth, jointMap.find("head")->second->offset[1], bodyThick, 0, 0, 0);
	}


	glm::mat4 defaultmat = glm::mat4(1.0f);
	glm::mat4 fkmat = fk(bvh, current, frameCur);

	for(int j = 0; j < ret.size(); j++) {
		for(int i = current->channelCount - 1; i >= 0; i--) {
			switch(current->channelOrder[i]) {
				case 0:
					ret[j] = glm::rotate(defaultmat, 
							glm::radians(frameCur[current->channelNum + i]),
							glm::vec3(1.0f, 0.0f, 0.0f)
							) * ret[j];
					break;
				case 1:
					ret[j] = glm::rotate(defaultmat, 
							glm::radians(frameCur[current->channelNum + i]),
							glm::vec3(0.0f, 1.0f, 0.0f)
							) * ret[j];
					break;
				case 2:
					ret[j] = glm::rotate(defaultmat, 
							glm::radians(frameCur[current->channelNum + i]),
							glm::vec3(0.0f, 0.0f, 1.0f)
							) * ret[j];
					break;
				case 3:
					ret[j] = glm::translate(defaultmat, 
							glm::vec3(frameCur[current->channelNum + i], 0.0f, 0.0f)
							) * ret[j];
					break;
				case 4:
					ret[j] = glm::translate(defaultmat, 
							glm::vec3(0.0f, frameCur[current->channelNum + i], 0.0f)
							) * ret[j];
					break;
				case 5:
					ret[j] = glm::translate(defaultmat, 
							glm::vec3(0.0f, 0.0f, frameCur[current->channelNum + i])
							) * ret[j];
					break;
				default:
					printf("error!\n");
					exit(1);
			}
		}
		ret[j] = fkmat * ret[j];
		ret[j] = fsm.offset * ret[j];
	}
	return ret;
}

void drawJoint(Joint* current) {
	if(current->name.compare("lfemur") == 0 || current->name.compare("rfemur") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(femurRad, slice, 10);

		glColor3f(0.35f, 0.65f, 0.80f);
		drawCube(-legThick, jointMap.find("ltibia")->second->offset[1] + femurRad + tibiaRad, legThick, 0, -femurRad, 0);

	}
	if(current->name.compare("ltibia") == 0 || current->name.compare("rtibia") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(tibiaRad, slice, 10);
		glColor3f(0.35f, 0.65f, 0.80f);
		drawCube(-legThick, jointMap.find("lfoot")->second->offset[1] + tibiaRad + footRad, legThick, 0, -tibiaRad, 0);
	}
	if(current->name.compare("lfoot") == 0 || current->name.compare("rfoot") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(footRad, slice, 10);
		glColor3f(0.38f, 0.69f, 0.85f);
		glRotatef(90, 1.0f, 0.0f, 0.0f);
		drawCube(footWidth, jointMap.find("ltoes")->second->offset[2] - footRad - toesRad, footThick, 0, footRad, 0);
		glRotatef(-90, 1.0f, 0.0f, 0.0f);
	}
	if(current->name.compare("ltoes") == 0 || current->name.compare("rtoes") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(toesRad, slice, 10);
		glColor3f(0.38f, 0.70f, 0.87f);
		glRotatef(90, 1.0f, 0.0f, 0.0f);
		drawCube(footWidth, toesLength - toesRad, footThick, 0, toesRad, 0);
		glRotatef(-90, 1.0f, 0.0f, 0.0f);
	}

	if(current->name.compare("lhumerus") == 0 || current->name.compare("rhumerus") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(humerusRad, slice, 10);

		glColor3f(0.38f, 0.69f, 0.85f);
		drawCube(-armThick, jointMap.find("lradius")->second->offset[1] + humerusRad + radiusRad, armThick, 0, -humerusRad, 0);

	}
	if(current->name.compare("lradius") == 0 || current->name.compare("rradius") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(radiusRad, slice, 10);
		glColor3f(0.38f, 0.69f, 0.85f);
		drawCube(-armThick, jointMap.find("lhand")->second->offset[1] + radiusRad + handRad, armThick, 0, -radiusRad, 0);
	}
	if(current->name.compare("lhand") == 0 || current->name.compare("rhand") == 0) {
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(handRad, slice, 10);
		glColor3f(0.38f, 0.69f, 0.85f);
		drawCube(-handThick, -handLength + handRad, handWidth, 0, -handRad, 0);
	}
	if(current->name.compare("head") == 0) {
		glColor3f(0.0, 1.0, 0.0);
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(neckRad, slice, 10);

		glPushMatrix();
		glTranslatef(0.0f, neckRad + headRad, 0.0f);
		glColor3f(0.38f, 0.70f, 0.87f);
		glRotatef(90, 1.0f, 0.0f, 0.0f);
		glutSolidSphere(headRad, slice, 10);
		glRotatef(-90, 1.0f, 0.0f, 0.0f);
		mouth(headRad);
		eye(headRad);
		glScalef(-1.0, 1.0, 1.0);
		eye(headRad);
		glPopMatrix();
	}
	if(current->name.compare("thorax") == 0) {
		glColor3f(0.0, 1.0, 0.0);
		glColor3f(0.35f, 0.35f, 0.80f);
		glutSolidSphere(thoraxRad, slice, 10);

		glColor3f(0.35f, 0.65f, 0.80f);
		//drawCube(bodyWidth, jointMap.find("lclavicle")->second->offset[1] - thoraxRad, bodyThick, 0, thoraxRad, 0);

	}

}

void drawBVHEndSite(EndSite* current) {
	glPushMatrix();

	glLineWidth(1.5); 
	glColor3f(0.5, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(current->offset[0], current->offset[1], current->offset[2]);
	glEnd();

	glTranslatef(current->offset[0], current->offset[1], current->offset[2]);

	glColor3f(0.0, 0.5, 0.0);
	glutSolidSphere(0.5, slice, 10);

	glPopMatrix();
}

std::vector<BVH*> interpolated;

void drawBVHJoint(Joint* current) {
	glPushMatrix();

	glLineWidth(1.5); 
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(current->offset[0], current->offset[1], current->offset[2]);
	glEnd();

	glTranslatef(current->offset[0], current->offset[1], current->offset[2]);

	for(int i = 0; i < current->channelCount; i++) {
		switch(current->channelOrder[i]) {
			case 0:
				glRotatef(frameCur[jointIndex++], 1.0f, 0.0f, 0.0f);
				break;
			case 1:
				glRotatef(frameCur[jointIndex++], 0.0f, 1.0f, 0.0f);
				break;
			case 2:
				glRotatef(frameCur[jointIndex++], 0.0f, 0.0f, 1.0f);
				break;
			case 3:
				glTranslatef(frameCur[jointIndex++], 0.0f, 0.0f);
				break;
			case 4:
				glTranslatef(0.0f, frameCur[jointIndex++], 0.0f);
				break;
			case 5:
				glTranslatef(0.0f, 0.0f, frameCur[jointIndex++]);
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}

	glColor3f(0.0, 1.0, 0.0);
	glutSolidSphere(0.5, slice, 10);
	drawJoint(current);
	
	for(Joint* joint : current->joints) {
		drawBVHJoint(joint);
	}

	for(EndSite* endSite : current->ends) {
		drawBVHEndSite(endSite);
	}

	glPopMatrix();
}

int drawIdx = 0;
std::vector<std::vector<float> > frames;
int prevfi = -1;
int frameC = 0;

void drawBVH() {
	BVH* current = bvh;
	//frameCur = frames[drawIdx++];
	//if (drawIdx == frames.size()) drawIdx = 0;

	fsm.idle();
	if(fsm.isInterpolate) {
		if(fsm.frameIndex != prevfi)
			printf("state: %d->%d, frameIndex: %d/%lu, command: '%c'\n", fsm.stateCur, fsm.stateNext, fsm.frameIndex, fsm.getMotion().size(), Camera::command);
	} else {
		if(fsm.frameIndex != prevfi)
			printf("state: %d, frameIndex: %d/%lu, command: '%c'\n", fsm.stateCur, fsm.frameIndex, fsm.getMotion().size(), Camera::command);
	}
	prevfi = fsm.frameIndex;
	frameCur = fsm.getFrame();

//	BVH* current = interpolated[drawIdx++];
//	if (drawIdx == interpolated.size()) drawIdx = 0;
//	usleep(1000);


    /* hw2
	timeCur = glutGet(GLUT_ELAPSED_TIME);
	while(timeCur - timeBase >= bvh->frameCount * bvh->frameTime * 1000) {
		timeBase += bvh->frameCount * bvh->frameTime * 1000;
	}
	frameIndex = (int) ((timeCur - timeBase) / bvh->frameTime / 1000); 

	frameCur = bvh->frames[frameIndex]; // set current frame's channel in hw2
	*/

    /* hw3 */
	//frameCur recalculate by jacobian
	/*
	if(selectedJoint->name.compare(camera.selectedJointName) != 0) {
		std::cout << camera.selectedJointName << std::endl;
		selectedJoint = jointMap.find(camera.selectedJointName)->second;
	}*/
		
//	ik(bvh, selectedJoint, frameCur); // set current frame's channel in hw3



	jointIndex = 0;

	camera.resize(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	//뒷배경 
	glClearColor(0.3, 0.3, 0.3, 1);

	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);

	glPushMatrix();
	glScalef(modelscale, modelscale, modelscale);

	glPushMatrix();


	glTranslatef(current->offset[0], current->offset[1], current->offset[2]);

	glMultMatrixf(glm::value_ptr(fsm.offset));

	for(int i = 0; i < current->channelCount; i++) {
		switch(current->channelOrder[i]) {
			case 0:
				glRotatef(frameCur[jointIndex++], 1.0f, 0.0f, 0.0f);
				break;
			case 1:
				glRotatef(frameCur[jointIndex++], 0.0f, 1.0f, 0.0f);
				break;
			case 2:
				glRotatef(frameCur[jointIndex++], 0.0f, 0.0f, 1.0f);
				break; 
			case 3:
				glTranslatef(frameCur[jointIndex++], 0.0f, 0.0f);
				break;
			case 4:
				glTranslatef(0.0f, frameCur[jointIndex++], 0.0f);
				break;
			case 5:
				glTranslatef(0.0f, 0.0f, frameCur[jointIndex++]);
				break;
			default:
				printf("error!\n");
				exit(1);
		}
	}


	glColor3f(1.0, 1.0, 1.0);
	glutSolidSphere(1, slice, 10);

	// 허리
	glColor3f(0.33f, 0.61f, 0.75f);
	/*
	drawCube(bodyWidth, jointMap.find("thorax")->second->offset[1], bodyThick, 0, 0, 0);
	*/
	drawBody();
	
	for(Joint* joint : current->joints) {
		drawBVHJoint(joint);
	}

	for(EndSite* endSite : current->ends) {
		drawBVHEndSite(endSite);
	}

	glPopMatrix();

	glPopMatrix();

	glutSwapBuffers();
	//std::cout << camera.movement.x << " " << camera.movement.y << " " << camera.movement.z << std::endl;
	camera.movement = glm::vec3(0.0f, 0.0f, 0.0f);

//	glutPostRedisplay();
}

std::map< std::string, std::vector< glm::mat4 >  > PreVersMap;

void drawWorld() {

	BVH* current = bvh;
	frameCur = bvh->frames[0];
	
	//if (drawIdx == frames.size()) drawIdx = 0;

	fsm.idle();
	/*
	if(fsm.isInterpolate) {
		if(fsm.frameIndex != prevfi)
			printf("state: %d->%d, frameIndex: %d/%lu, command: '%c'\n", fsm.stateCur, fsm.stateNext, fsm.frameIndex, fsm.getMotion().size(), Camera::command);
	} else {
		if(fsm.frameIndex != prevfi)
			printf("state: %d, frameIndex: %d/%lu, command: '%c'\n", fsm.stateCur, fsm.frameIndex, fsm.getMotion().size(), Camera::command);
	}
	*/
	prevfi = fsm.frameIndex;
	//fsm.frameIndex = 0;
	frameCur = fsm.getFrame();

	jointIndex = 0;

	camera.resize(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	//뒷배경 
	glClearColor(0.3, 0.3, 0.3, 1);

	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);

	glPushMatrix();
	glScalef(modelscale, modelscale, modelscale);

	glPushMatrix();


	printf("JOINT CHECK -------\n");
	
	class p {
	public:
		double x, y, z;
		p()
		{}
		p(double x, double y, double z)
		:x(x), y(y), z(z)
		{}
		double operator*(p b) {
			return x*b.x + y*b.y + z*b.z;
		}
		double sz() {
			return x*x + y*y + z*z;
		}
		void normalize() {
			double s = sqrt(sz());
			if(s < 1e-9) return;
			x /= s; y /= s; z /= s;
		}
		p operator^(p b) {
			return p(y*b.z - z*b.y, -x*b.z + z*b.x, x*b.y - y*b.x);
		}
		p operator*(double c) {
			return p(x*c, y*c, z*c);
		}
		p operator+(p b) {
			return p(x + b.x, y + b.y, z + b.z);
		}
		p operator-(p b) {
			return p(x - b.x, y - b.y, z - b.z);
		}
	};
	int ridx[6][3] = {
		{0, 1, 2}, {3, 5, 4}, {3, 2, 6}, {2, 1, 7}, {0, 4, 7}, {4, 5, 6}
	};
	const auto& particles = mSoftWorld->mX;

	std::map< std::string, std::vector< glm::mat4 >  > CurVersMap;
	for(auto const& x : current->jointMap) {
		auto* joint = x.second;
		std::vector< glm::mat4 > vers = getJointRect(joint);
		CurVersMap.insert(std::make_pair(x.first, vers));
	}

	if(PreVersMap.empty()) {
		std::cout << "prev empty\n";
		PreVersMap = CurVersMap;
	}

	int maxCount = int(bvh->frameTime/world_timestep);
	std::cout<< "maxcount: " << maxCount << std::endl;
	
	for(int timecount = 1; timecount <= int(bvh->frameTime/world_timestep); timecount++){

		std::vector<int> attachIdx;
		std::vector<Eigen::Vector3d> attachTarget;
		for(auto const& x : current->jointMap) {
			auto* joint = x.second;
			string name_ = x.first;
			std::vector< glm::mat4 >& vers1 = PreVersMap[x.first], &vers2 = CurVersMap[x.first];
			
			if(name_ == "thorax") {
				attachIdx.push_back(132); attachIdx.push_back(143); attachIdx.push_back(144); attachIdx.push_back(155);
				attachTarget.push_back(Eigen::Vector3d(vers2[7][3][0],vers2[7][3][1],vers2[7][3][2]));
				attachTarget.push_back(Eigen::Vector3d(vers2[4][3][0],vers2[4][3][1],vers2[4][3][2]));
				attachTarget.push_back(Eigen::Vector3d(vers2[6][3][0],vers2[6][3][1],vers2[6][3][2]));
				attachTarget.push_back(Eigen::Vector3d(vers2[5][3][0],vers2[5][3][1],vers2[5][3][2]));
			}
			
			if(vers1.size()!=vers2.size() || vers1.empty() || vers2.empty()) continue;
			std::vector< glm::mat4 > vers;
			vers.resize(8);
			for(int i = 0; i < 8; i++) {
				for(int j = 0; j < 3; j++) {
					vers[i][3][j] = (vers1[i][3][j] * (maxCount - timecount) / maxCount + vers2[i][3][j] * timecount / maxCount);
				}
			}
			
			for(int i = 0; i < particles.size()/3; i++) {
				if(i==132 || i==143 || i==144 || i==155) continue;
				bool in = true;
				double dis = 1e9;
				p pn;
				auto pb = mSoftWorld->mX.block<3,1>(3*i, 0);
				double x = pb[0], y = pb[1], z = pb[2];
				for(int j = 0; j < 6; j++) {
					p A(vers[ridx[j][0]][3][0], vers[ridx[j][0]][3][1], vers[ridx[j][0]][3][2]),
					  B(vers[ridx[j][1]][3][0], vers[ridx[j][1]][3][1], vers[ridx[j][1]][3][2]),
					  C(vers[ridx[j][2]][3][0], vers[ridx[j][2]][3][1], vers[ridx[j][2]][3][2]);
					//A=A*10000; B=B*10000; C=C*10000;
					double a, b, c, d;
					p N = (C - B) ^ (A - B);
					a = N.x; b = N.y; c = N.z;
					d = -A.x*a - A.y*b - A.z*c;
					if (a*x + b*y + c*z + d > 0.0) {
						in = false;
						break;
					}
					
					double w = abs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);
					if (w < dis) {
						dis = w;
						pn = p(a,b,c);
					}
				}
				if (in) {
					/*
					double cx = 0.0, cy = 0.0, cz = 0.0;
					for(int i = 0; i < 8; i++) {
						cx += vers[i][3][0];
						cy += vers[i][3][1];
						cz += vers[i][3][2];
					}
					cx /= 8; cy /= 8; cz /= 8;
					p nn=p(x, y, z) - p(cx, cy, cz);
					nn.normalize();
					double mxt = 1e9;
					for(int i = 0; i < 6; i++) {
						p A(vers[ridx[i][0]][3][0], vers[ridx[i][0]][3][1], vers[ridx[i][0]][3][2]),
						  B(vers[ridx[i][1]][3][0], vers[ridx[i][1]][3][1], vers[ridx[i][1]][3][2]),
						  C(vers[ridx[i][2]][3][0], vers[ridx[i][2]][3][1], vers[ridx[i][2]][3][2]);
						double a, b, c, d;
						p N = (C - B) ^ (A - B);
						a = N.x; b = N.y; c = N.z;
						d = -A.x*a - A.y*b - A.z*c;
						double tw = -(a*x+b*y+c*z+d)/(a*nn.x+b*nn.y+c*nn.z);
						if(tw < 0) continue;
						if(tw < mxt) mxt = tw;
					}
					if (mxt < 1e9) {
						std::cout << "collision with " << name_ << ": " << i << " move to " << nn.x << ", " << nn.y << ", " << nn.z << " with dist " << mxt << "\n";
						mSoftWorld->mX[3*i] += mxt*nn.x;
						mSoftWorld->mX[3*i+1] += mxt*nn.y;
						mSoftWorld->mX[3*i+2] += mxt*nn.z;
						
						double tx = mSoftWorld->mX[3*i], ty = mSoftWorld->mX[3*i+1], tz = mSoftWorld->mX[3*i+2];
					//	mSoftWorld->mExternalForces[3*i] += mxt*pn.x;
					//	mSoftWorld->mExternalForces[3*i+1] += mxt*pn.y;
					//	mSoftWorld->mExternalForces[3*i+2] += mxt*pn.z;
						attachIdx.push_back(i);
						attachTarget.push_back(Eigen::Vector3d(tx, ty, tz));
					}
					*/
					
					pn.normalize();
					double vx = mSoftWorld->mV[3*i], vy = mSoftWorld->mV[3*i+1], vz = mSoftWorld->mV[3*i+2];
					p v(vx,vy,vz);
					p nv = v-pn*(pn*v)*(1.0+eps);
					std::cout << "collision with " << name_ << " - " << i << " : move to " << pn.x << ", " << pn.y << ", " << pn.z << " with dist " << dis << "\n";
					std::cout << "v : " << v.x << ", " << v.y << ", " << v.z << "\n";
					std::cout << "nv : " << nv.x << ", " << nv.y << ", " << nv.z << "\n";

//					mSoftWorld->mV[3*i] = nv.x;
//					mSoftWorld->mV[3*i+1] = nv.y;
//					mSoftWorld->mV[3*i+2] = nv.z;
					mSoftWorld->mX[3*i] += dis*pn.x * 1.0;
					mSoftWorld->mX[3*i+1] += dis*pn.y * 1.0;
					mSoftWorld->mX[3*i+2] += dis*pn.z * 1.0;

					double tx = mSoftWorld->mX[3*i], ty = mSoftWorld->mX[3*i+1], tz = mSoftWorld->mX[3*i+2];
					attachIdx.push_back(i);
					attachTarget.push_back(Eigen::Vector3d(tx, ty, tz));
//					mSoftWorld->AddConstraint(new FEM::AttachmentConstraint(500000,i,Eigen::Vector3d(tx, ty, tz)));

//					mSoftWorld->mExternalForces[3*i] = 0.0;
//					mSoftWorld->mExternalForces[3*i+1] = 0.0;
//					mSoftWorld->mExternalForces[3*i+2] = 0.0;
//					mSoftWorld->mExternalForces[3*i] += dis*pn.x * 200.0;
//					mSoftWorld->mExternalForces[3*i+1] += dis*pn.y * 200.0;
//					mSoftWorld->mExternalForces[3*i+2] += dis*pn.z * 200.0;
					
				}
			}
		}
		mSoftWorld->mConstraintDofs = 0;
		for(int i=0;i<attachIdx.size();i++) {
			FEM::AttachmentConstraint* ac = new FEM::AttachmentConstraint(500000, attachIdx[i], attachTarget[i]);
			mSoftWorld->mConstraints.push_back(ac);
		}
		for(auto c : mSoftWorld->mConstraints) {
			mSoftWorld->mConstraintDofs += c->GetDof();
		}
		mSoftWorld->PreComputation();
		mSoftWorld->TimeStepping();
		for(int i=0;i<attachIdx.size();i++)
			mSoftWorld->mConstraints.pop_back();
		mSoftWorld->mConstraintDofs = 0;
		for(auto c : mSoftWorld->mConstraints) {
			mSoftWorld->mConstraintDofs += c->GetDof();
		}
		mSoftWorld->PreComputation();
	}

	printf("CLOTH DRAW -------\n");
	for(int i = 0; i < particles.size()/3; i++) {
		auto p = mSoftWorld->mX.block<3,1>(3*i,0);
		glColor3f(0.35f, 0.80f, 0.35f);
		if(i > 132 && i < 156) {
			glColor3f(0.89f, 0.80f, 0.35f);
		}
		glBegin(GL_POINTS);
		glPointSize(100.0f);
		glVertex3d(p[0], p[1], p[2]);
		glEnd();
		glTranslated(p[0], p[1], p[2]);
		glutSolidSphere(handRad, slice, 10);
		glTranslated(-p[0], -p[1], -p[2]);
	}

	for(auto const& x : current->jointMap) {
		auto* joint = x.second;
		std::vector< glm::mat4 >& vers = CurVersMap[x.first];
		/*
		std::cout << "JOINT " << joint->name << std::endl;
		if(vers.size() > 0) {
			std::cout << vers[0][3][0] << ", "<< vers[0][3][1] << ", " << vers[0][3][2] << std::endl;
		}
		*/
		for(auto ver : vers) {
			auto p = ver[3];
			glColor3f(0.35f, 0.35f, 0.80f);
			glTranslated(p[0], p[1], p[2]);
			glutSolidSphere(handRad, slice, 10);
			glTranslated(-p[0], -p[1], -p[2]);
		}
		if (vers.size() != 8) {
			continue;
		} else {
			glColor3f(0.1f, 0.1f, 0.1f);
			glBegin( GL_QUADS );
			glVertex3f(vers[0][3][0], vers[0][3][1], vers[0][3][2]);
			glVertex3f(vers[1][3][0], vers[1][3][1], vers[1][3][2]);
			glVertex3f(vers[2][3][0], vers[2][3][1], vers[2][3][2]);
			glVertex3f(vers[3][3][0], vers[3][3][1], vers[3][3][2]);
			glVertex3f(vers[4][3][0], vers[4][3][1], vers[4][3][2]);
			glVertex3f(vers[5][3][0], vers[5][3][1], vers[5][3][2]);
			glVertex3f(vers[6][3][0], vers[6][3][1], vers[6][3][2]);
			glVertex3f(vers[7][3][0], vers[7][3][1], vers[7][3][2]);
			glEnd();
			glColor3f(0.1f, 0.1f, 0.1f);
			glBegin( GL_QUADS );
			glVertex3f(vers[0][3][0], vers[0][3][1], vers[0][3][2]);
			glVertex3f(vers[3][3][0], vers[3][3][1], vers[3][3][2]);
			glVertex3f(vers[5][3][0], vers[5][3][1], vers[5][3][2]);
			glVertex3f(vers[4][3][0], vers[4][3][1], vers[4][3][2]);
			glVertex3f(vers[1][3][0], vers[1][3][1], vers[1][3][2]);
			glVertex3f(vers[7][3][0], vers[7][3][1], vers[7][3][2]);
			glVertex3f(vers[6][3][0], vers[6][3][1], vers[6][3][2]);
			glVertex3f(vers[2][3][0], vers[2][3][1], vers[2][3][2]);
			glEnd();
		}
	}
	const auto& constraints = mSoftWorld->mConstraints;
	double stiff = constraints[0]->GetStiffness();
	std::cout << "stiff: " << stiff << std::endl;
	if(Camera::command2 == 'z') {
		Camera::command2 = '\0';
		stiff = stiff * 10;
		if(stiff > 1E8) stiff = 1E8;
		for(auto s : constraints) {
			s->SetStiffness(stiff);
		}
		mSoftWorld->PreComputation();
	}
	else if(Camera::command2 == 'c') {
		Camera::command2 = '\0';
		stiff = stiff / 10;
		if(stiff < 1E-1) stiff = 1E-1;
		for(auto s : constraints) {
			s->SetStiffness(stiff);
		}
		mSoftWorld->PreComputation();
	}
	

	PreVersMap = CurVersMap;
	glPopMatrix();

	glPopMatrix();

	glutSwapBuffers();
	//std::cout << camera.movement.x << " " << camera.movement.y << " " << camera.movement.z << std::endl;
	camera.movement = glm::vec3(0.0f, 0.0f, 0.0f);
}

void initParam() {
}


// 메인 함수
int main(int argc, char **argv) {
	
	initParam();

	Parser parser1;

	BVH *bvh1, *bvh2;
	char file1[50] = "MotionData/STAND.bvh";
	bvh1=parser1.parse(file1);
	
	bvh = bvh1;
	jointMap = bvh->jointMap;

	//init world
	mSoftWorld = new FEM::World(
		FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		FEM::OptimizationMethod::OPTIMIZATION_METHOD_NEWTON,
		FEM::LinearSolveType::SOLVER_TYPE_LDLT,
		world_timestep,										//time_step
		100, 											//max_iteration	
		0.99											//damping_coeff
		);

	mCloth = new Cloth();
	mCloth->Initialize(mSoftWorld);
	mSoftWorld->Initialize();
	
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Animation HW6 2018-23598, 2017-25969");

	glutSpecialFunc(camera.specialKeyboardHandler);
	glutKeyboardFunc(camera.keyboardHandler);
	//glutDisplayFunc(drawWorld);
	glutIdleFunc(drawWorld);
	glutReshapeFunc(camera.resize);
	glutMouseFunc(camera.mouseHandler);
	glutMotionFunc(camera.mouseDragHandler);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
	return 0;
}
