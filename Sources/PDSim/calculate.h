#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <eigen/Core>
#include <Eigen/src/Geometry/Quaternion.h>

typedef double ValType;
typedef Eigen::Matrix<ValType, 3, 3> mat3;
typedef Eigen::Matrix<ValType, 3, 1> vec3;
typedef Eigen::Matrix<ValType, 6, 1> vec6;
typedef Eigen::Matrix<ValType, 6, 6> mat6;


using namespace std;

double l2dis(vec3 p1, vec3 p2 = vec3(0, 0, 0));
void print_line();
void print_vec3(vec3 v);
void print_vec6(vec6 v);
void print_vec6(vector<float> v);
void print_mat3(mat3 m);

class calculator
{
public:
	calculator();
	~calculator();
	
	void set_k(ValType k_vc, ValType k_c, ValType k_vct=1);
	void set_k();

	vec3 calculateFVC(vec3 Xg, vec3 Xh);
	mat3 calculatePartialFVC_X(vec3 qg, vec3 qh);
	mat3 calculatePartialFVC_Omega();
	
	ValType saturationMapping(ValType r);
	ValType saturationMappingDerivative(ValType r);

	Eigen::Matrix<double,6,1> solve_6DOF(vec3 Xh, vec3 Xg, vec3 Omega_h, vec3 Omega_g,
		float graspL,
		vec3 F_c, vec3 T_c,
		mat3 partial_FC_X, mat3 partial_FC_Omega,
		mat3 partial_TC_X, mat3 partial_TC_Omega, 
		vec3& updatedX, vec3& updatedDir, vec3& T_vc,
		int collisionNum, double* times);

	mat3 SkewSymmetricMatrix(vec3 v);

	Eigen::Quaterniond calculate_qt(vec3 Omega_h, vec3 Omega_g);
	vec3 calculateTVC(Eigen::Quaterniond qt);
	vec3 calculateTVC(vec3 Omega_h, vec3 Omega_g);
	mat3 calculatePartialTVC_Omega(Eigen::Quaterniond qt);
	mat3 calculatePartialTVC_X();

	mat3 genTildeMatrix(vec3 r);

	vec3 calculateToolDir(vec3 Omega);
	vec3 calculateGraspPoint(vec3 X, vec3 Omega, float l);
	Eigen::Quaterniond Omega2Quaternion(vec3 Omega);
	vec3 Quaternion2Omega(Eigen::Quaterniond q);

	vec3 calculateR(vec3 p, vec3 Xg_grasp);
	vec3 calculateFC(vec3 N0, double d);
	vec3 calculateTC(vec3 r, vec3 Fc);
	mat3 calculatePartialFC_X(vec3 N0);
	mat3 calculatePartialFC_Omega(vec3 r, vec3 N0, double d);
	mat3 calculatePartialTC_X(vec3 Fc, vec3 r, vec3 N0);
	mat3 calculatePartialTC_Omega(vec3 Fc, vec3 r, vec3 N0, double d);

	void calculateForce6DOF(vector<float> qh, vector<float> qg, float graspL, vec3& Fvc, vec3& Tvc);
private:

	double _r_lin;
	ValType m_k_vc;
	ValType m_k_c;
	ValType m_k_vct;
};