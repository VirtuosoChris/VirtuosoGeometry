#ifndef TRANSFORMATIONS_H_INCLUDED
#define TRANSFORMATIONS_H_INCLUDED

#include <Eigen/Geometry>

///\file These functions create transformation matrices using the Eigen library.  What these particular transforms are and what their arguments
///do should all hopefully be obvious.  All angles are in radians and sane default arguments are provided where possible


///lookat()
///orthoProjection()


Eigen::Matrix4f perspectiveProjection(float fovRads = 3.14159 / 4.0, float aspect = 4.0/3.0, float zNear = .01, float zFar = 1000.0 );

Eigen::Matrix4f rotationMatrixY(float theta);

Eigen::Matrix4f rotationMatrixZ(float theta);

Eigen::Matrix4f rotationMatrixX(float theta);

Eigen::Matrix4f rotationMatrixXYZ(float rotX, float rotY, float rotZ);

Eigen::Matrix4f rotateAxisAngle(Eigen::Vector3f axis, float angle);

Eigen::Matrix4f scalingMatrix(float x, float y, float z);

Eigen::Matrix4f translationMatrix(float x, float y, float z);

///creates a matrix to transform normals from object space to eye space while maintaining their perpindicularity to the surface
Eigen::Matrix3f normalMatrix(const Eigen::Matrix4f& modelViewMatrix);

#endif
