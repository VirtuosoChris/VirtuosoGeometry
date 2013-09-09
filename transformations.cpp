#include "transformations.h"


Eigen::Matrix4f perspectiveProjection(float fovRads, float aspect, float zNear, float zFar ){

	Eigen::Matrix4f temp;

	float f = 1.0f / tan(  fovRads * .5f );

	temp.col(0) = Eigen::Vector4f(f / aspect, 0.0f, 0.0f , 0.0f);
	temp.col(1) = Eigen::Vector4f(0.0f,  f , 0.0f  ,0.0f);
	temp.col(2) = Eigen::Vector4f(0.0f, 0.0f, (zFar + zNear) / (zNear - zFar) ,-1.0f );
	temp.col(3) = Eigen::Vector4f(0.0f, 0.0f, 2.0f*zFar * zNear / (zNear-zFar) ,0.0f);

	return temp;
}

Eigen::Matrix4f rotationMatrixY(float theta){

	Eigen::Matrix4f temp;

	temp.col(0) = Eigen::Vector4f(cos(theta), 0.0f, -sin(theta), 0.0f);
	temp.col(1) = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.0f);
	temp.col(2) = Eigen::Vector4f(sin(theta), 0.0f, cos(theta), 0.0f);
	temp.col(3) = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);

	return temp;

}

Eigen::Matrix4f rotationMatrixZ(float theta){

	Eigen::Matrix4f temp;

	temp.col(0) = Eigen::Vector4f(cos(theta), sin(theta), 0.0f, 0.0f);
	temp.col(1) = Eigen::Vector4f(-sin(theta), cos(theta), 0,0);
	temp.col(2) = Eigen::Vector4f(0.0f,0.0f,1.0f,0.0f);
	temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0f);

	return temp;

}

Eigen::Matrix4f rotationMatrixX(float theta){

	Eigen::Matrix4f temp;

	temp.col(0) = Eigen::Vector4f(1.0f,0.0f,0.0f,0.0f);
	temp.col(1) = Eigen::Vector4f(0.0f,cos(theta), sin(theta), 0.0f);
	temp.col(2) = Eigen::Vector4f(0.0f,-sin(theta), cos(theta), 0.0f);
	temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1);

	return temp;

}

Eigen::Matrix4f scalingMatrix(float x, float y, float z){

	Eigen::Matrix4f temp;

	temp.col(0) = Eigen::Vector4f(x,0.0f,0.0f,0.0f);
	temp.col(1) = Eigen::Vector4f(0.0f,y,0.0f,0.0f);
	temp.col(2) = Eigen::Vector4f(0.0f,0.0f,z,0.0f);
	temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);

	return temp;

}

Eigen::Matrix4f translationMatrix(float x, float y, float z){

	Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();

	temp.col(3) = Eigen::Vector4f(x,y,z,1.0f);

	return temp;
}

Eigen::Matrix3f normalMatrix(const Eigen::Matrix4f& modelViewMatrix){
	return modelViewMatrix.topLeftCorner<3,3>().inverse().transpose();
}

Eigen::Matrix4f rotationMatrixXYZ(float rotX, float rotY, float rotZ)
{

float sinZ = sin(rotZ);
float sinX = sin(rotX);
float sinY = sin(rotY);

float cosZ = cos(rotZ);
float cosX = cos(rotX);
float cosY = cos(rotY);


Eigen::Matrix4f temp;


temp.col(0) = Eigen::Vector4f(cosY * cosZ,
                              -cosY * sinZ,
                              sinY,
                              0.0f);

temp.col(1) = Eigen::Vector4f(cosX*sinZ + sinX * sinY * cosZ,
                              cosX*cosZ + sinX * sinY * sinZ,
                              -sinX * cosY,
                              0.0
                              );

temp.col(2) = Eigen::Vector4f(sinX*sinZ - cosX * sinY * cosZ,
                              sinX*cosZ + cosX * sinY * sinZ,
                              cosX * sinY,
                              0.0
                              );


temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);



return temp;

}



Eigen::Matrix4f rotateAxisAngle(Eigen::Vector3f axis, float angle){

    throw std::runtime_error("Implement me exception");
}
