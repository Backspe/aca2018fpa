#ifndef UTIL_H
#define UTIL_H

#include "readBvh.h"
#include "camera.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/quaternion.hpp"
#include <GL/glut.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <vector>

using namespace glm;
using namespace Eigen;

typedef std::vector< float > fVector;
typedef std::vector< std::vector< float > > fMatrix;

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon());
glm::mat4 fk(BVH* bvh, Joint* current, Frame frame);
void ik(BVH* bvh, Joint* current, Frame frame);
float surface(fMatrix points, float u, float v);
Motion interpolateMotions(std::vector< std::vector< Motion > > motions, float u, float v);
Motion interpolateFrames(Motion a, Motion b, int cntA, int cntB, int frameCount = 60);
std::vector<std::vector<float> > interpolateFrames(BVH* a, BVH* b, int cntA, int cntB, int frameCount = 60);

#endif
