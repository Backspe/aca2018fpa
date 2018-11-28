#include "readBvh.h"
#include "camera.h"
#include "fsm.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/quaternion.hpp"
#include <GL/glut.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <math.h>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>

#include <unistd.h>

#define PI 3.14159265358979


using namespace glm;
using namespace Eigen;

float modelscale = 1.0f;
int timeCur = 0, timeBase;
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

// 각도 관련 변수

Camera camera = Camera();

FSM fsm = FSM();

BVH* bvh;
std::map< std::string, Joint* > jointMap;

int jointIndex = 0;
int frameIndex = 0;

std::vector< float > frameCur;

Joint* selectedJoint;

//copy from http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

glm::mat4 fk(Joint* current) {	
	
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
							glm::radians(frameCur[par->channelNum + i]),
							glm::vec3(1.0f, 0.0f, 0.0f)
							) * pos;
					break;
				case 1:
					pos = glm::rotate(defaultmat, 
							glm::radians(frameCur[par->channelNum + i]),
							glm::vec3(0.0f, 1.0f, 0.0f)
							) * pos;
					break;
				case 2:
					pos = glm::rotate(defaultmat, 
							glm::radians(frameCur[par->channelNum + i]),
							glm::vec3(0.0f, 0.0f, 1.0f)
							) * pos;
					break;
				case 3:
					pos = glm::translate(defaultmat, 
							glm::vec3(frameCur[par->channelNum + i], 0.0f, 0.0f)
							) * pos;
					break;
				case 4:
					pos = glm::translate(defaultmat, 
							glm::vec3(0.0f, frameCur[par->channelNum + i], 0.0f)
							) * pos;
					break;
				case 5:
					pos = glm::translate(defaultmat, 
							glm::vec3(0.0f, 0.0f, frameCur[par->channelNum + i])
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
						glm::radians(frameCur[bvh->channelNum + i]),
						glm::vec3(1.0f, 0.0f, 0.0f)
						) * pos;
				break;
			case 1:
				pos = glm::rotate(defaultmat, 
						glm::radians(frameCur[bvh->channelNum + i]),
						glm::vec3(0.0f, 1.0f, 0.0f)
						) * pos;
				break;
			case 2:
				pos = glm::rotate(defaultmat, 
						glm::radians(frameCur[bvh->channelNum + i]),
						glm::vec3(0.0f, 0.0f, 1.0f)
						) * pos;
				break;
			case 3:
				pos = glm::translate(defaultmat, 
						glm::vec3(frameCur[bvh->channelNum + i], 0.0f, 0.0f)
						) * pos;
				break;
			case 4:
				pos = glm::translate(defaultmat, 
						glm::vec3(0.0f, frameCur[bvh->channelNum + i], 0.0f)
						) * pos;
				break;
			case 5:
				pos = glm::translate(defaultmat, 
						glm::vec3(0.0f, 0.0f, frameCur[bvh->channelNum + i])
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

void ik(Joint* current) {
	glm::mat4 curMat = fk(current);
	Joint* par = current->parent;
	std::vector< glm::vec3 > jacobianList;
	
	std::vector< int > thetas;
	std::vector< double > dthetas;
	int depth = 1;
	while(par != NULL && depth <= camera.maxDepth) {
		glm::mat4 parMat = fk(par);
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

		double dt = glm::degrees(glm::dot(r, camera.movement));
		if(dt > 45 || dt < -45) { // avoid ill-condition
			std::cout << "ill-conditioned" << std::endl;
			break; 
		}
		dthetas.push_back(dt);
	}
	if(dthetas.size() == n) {
		for(int i = 0; i < n; i++) {
			frameCur[thetas[i]] += dthetas[i];
		}
	}
	
}


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
		
//	ik(selectedJoint); // set current frame's channel in hw3



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


void initParam() {
}

void jointMapping(Joint* current) {
	jointMap.insert(std::make_pair(current->name, current));
	for(Joint* joint : current->joints) {
		jointMapping(joint);
	}

}

EndSite* sumTwoEndsite(EndSite* a, EndSite* b, double w) {
	EndSite* ret = new EndSite();
	ret = a;
	for(int i = 0; i < 3; i++) {
		ret->offset[i] = a->offset[i] * (1.0 - w) + b->offset[i] * w;
	}
	return ret;
}

Joint* sumTwoJoint(Joint* a, Joint* b, double w) {
	Joint* ret = new Joint(a->name);
	ret = a;
	for(int i = 0; i < 3; i++) {
		ret->offset[i] = a->offset[i] * (1.0 - w) + b->offset[i] * w;
		ret->rotation[i] = a->rotation[i] * (1.0 - w) + b->rotation[i] * w;
	}
	for(int i = 0; i < a->joints.size(); i++) {
		ret->joints[i] = sumTwoJoint(a->joints[i], b->joints[i], w);
	}
	for(int i = 0; i < a->ends.size(); i++) {
		ret->ends[i] = sumTwoEndsite(a->ends[i], b->ends[i], w);
	}
	return ret;
}

BVH* sumTwoBVH(BVH* a, BVH* b, double w) {
	BVH* ret = new BVH(a->name);
	ret = a;
	for(int i = 0; i < 3; i++) {
		ret->offset[i] = a->offset[i] * (1.0 - w) + b->offset[i] * w;
		ret->position[i] = a->position[i] * (1.0 - w) + b->position[i] * w;
		ret->rotation[i] = a->rotation[i] * (1.0 - w) + b->position[i] * w;
	}
	for(int i = 0; i < a->joints.size(); i++) {
		ret->joints[i] = sumTwoJoint(a->joints[i], b->joints[i], w);
	}
	for(int i = 0; i < a->ends.size(); i++) {
		ret->ends[i] = sumTwoEndsite(a->ends[i], b->ends[i], w);
	}
/*	
	for(int i = 0; i < 6; i++) {
		ret->channelOrder[i] = a->channelOrder[i];
	}
	ret->channelCount = a->channelCount;
	ret->channelCountAll = a->channelCountAll;
	ret->channelNum = a->channelNum;
	for(int i = 0; i < a->joints.size(); i++) {
		Joint* newJoint = sumTwoJoint(a->joints[i], b->joints[i], w);
		ret->joints.push_back(newJoint);
	}
*/	
	return ret;
}

// 메인 함수
int main(int argc, char **argv) {
	
	initParam();

	Parser parser1;

	BVH *bvh1, *bvh2;
	char file1[50] = "MotionData/STAND.bvh";
	bvh1=parser1.parse(file1);
	
	bvh = bvh1;

	for(Joint* joint : bvh->joints) {
		jointMapping(joint);
	}
	
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Animation HW4 2018-23598, 2017-259690");

	glutSpecialFunc(camera.specialKeyboardHandler);
	glutKeyboardFunc(camera.keyboardHandler);
	glutDisplayFunc(drawBVH);
	glutIdleFunc(drawBVH);
	glutReshapeFunc(camera.resize);
	glutMouseFunc(camera.mouseHandler);
	glutMotionFunc(camera.mouseDragHandler);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
	return 0;
}
