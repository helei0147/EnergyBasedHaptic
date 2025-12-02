#include "calculate.h"
#include <Windows.h>

int L_MAX = 20;
double damping = 0.5;
double DELTA_X_MAX = 0.013;
double DELTA_ANGLE_MAX = 0.3 * 3.1415926 / 180;
double FVC_MAX = 3.2;
double EPS = 1e-8;

using namespace Eigen;
using namespace std;

double l2dis(vec3 p1, vec3 p2)// 欧氏距离
{
	vec3 v = p1 - p2;
	return sqrt(v.dot(v));
}

void print_line()
{
	cout << "-----------------------" << endl;
}

void print_vec3(vec3 v)
{
	printf_s("[%lf %lf %lf]\n", v(0), v(1), v(2));
}

void print_vec6(vec6 v)
{
	printf_s("[%lf %lf %lf %lf %lf %lf]\n", v(0), v(1), v(2), v(3), v(4), v(5));
}

void print_vec6(vector<float> v)
{
	printf_s("[%lf %lf %lf %lf %lf %lf]\n", v[0], v[1], v[2], v[3], v[4], v[5]);
}

void print_mat3(mat3 m)
{
	cout << m << endl;
}

double TimeCalculation(LARGE_INTEGER t0, LARGE_INTEGER t1, LARGE_INTEGER nFreq)
{
	double t = (t1.QuadPart - t0.QuadPart) / (double)nFreq.QuadPart * 1000;
	return t;
}

calculator::calculator()
{
	m_k_vc = 5;
	m_k_c = 5;
	m_k_vct = 1;

	_r_lin = FVC_MAX / m_k_vc / 2;
}

calculator::~calculator()
{
}

void calculator::set_k(ValType k_vc, ValType k_c, ValType k_vct)
{
	m_k_vc = k_vc;
	m_k_c = k_c;

	_r_lin = FVC_MAX / k_vc / 2;

	m_k_vct = k_vct;
}

void calculator::set_k() {
	_r_lin = FVC_MAX / m_k_vc / 2;
}


mat3 calculator::calculatePartialFVC_X(vec3 qg, vec3 qh)
{
	double r = l2dis(qg, qh);
	mat3 result;
	if (r < _r_lin)
	{
		mat3 identity;
		identity.setIdentity();
		result = -m_k_vc * identity;
	}
	else
	{
		vec3 e_r = (qg - qh) / r;
		double f_r = saturationMapping(r);
		double f_r_derivative = saturationMappingDerivative(r);

		mat3 identity;
		identity.setIdentity();
		mat3 e_r_matrix = e_r * e_r.transpose();

		auto partial = -f_r / r * identity + (f_r / r - f_r_derivative) * e_r_matrix;
		result = partial;
	}
	return result;
}


mat3 calculator::calculatePartialFVC_Omega()
{
	mat3 result;
	result.setZero();
	return result;
}

ValType calculator::saturationMapping(ValType r)
{
	ValType result;
	if (r < _r_lin)
	{
		result = m_k_vc * r;
	}
	else
	{
		double weight;
		double upper_index = 2 * m_k_vc * (_r_lin - r) / FVC_MAX;
		weight = 1 - 0.5 * exp(upper_index);
		result = FVC_MAX * weight;
	}
	return ValType(result);
}

ValType calculator::saturationMappingDerivative(ValType r)
{
	ValType result = m_k_vc * exp(2 * m_k_vc * (_r_lin - r) / FVC_MAX);
	return result;
}

Quaterniond calculator::calculate_qt(vec3 Omega_h, vec3 Omega_g)
{
	Quaterniond result;
	Quaterniond quat_g = Omega2Quaternion(Omega_g);
	Quaterniond quat_h = Omega2Quaternion(Omega_h);
	result = quat_g * quat_h.inverse();

	if (result.w() < 0)
	{
		quat_g = Quaterniond(-quat_g.w(), -quat_g.x(), -quat_g.y(), -quat_g.z());
		result = quat_g * quat_h.inverse();
	}
	Quaterniond t(-m_k_vct * result.w(), -m_k_vct * result.x(), -m_k_vct * result.y(), -m_k_vct * result.z());

	return t;
}

vec3 calculator::calculateTVC(Quaterniond qt)
{
	vec3 result(qt.x(), qt.y(), qt.z());
	return result;
}

vec3 calculator::calculateTVC(vec3 Omega_h, vec3 Omega_g)
{
	auto qt = calculate_qt(Omega_h, Omega_g);
	vec3 TVC = calculateTVC(qt);
	return TVC;
}

mat3 calculator::calculatePartialTVC_Omega(Quaterniond qt)
{
	mat3 result;
	vec3 TVC = calculateTVC(qt);
	double scalar_qT = qt.w();
	mat3 part0;
	part0.setIdentity();
	mat3 part2 = genTildeMatrix(TVC);
	result = 0.5 * (scalar_qT * part0 - part2);
	return result;
}

mat3 calculator::calculatePartialTVC_X()
{
	mat3 result;
	result.setZero();
	return result;
}

mat3 calculator::genTildeMatrix(vec3 r)
{
	mat3 result;
	result << 0, -r(2), r(1),
		r(2), 0, -r(0),
		-r(1), r(0), 0;
	return result;
}

vec3 calculator::calculateToolDir(vec3 Omega)
{
	Quaterniond q = Omega2Quaternion(Omega);
	mat3 rot = q.toRotationMatrix();
	vec3 dir(0, 0, 1);
	return rot * dir;
}

vec3 calculator::calculateGraspPoint(vec3 X, vec3 Omega, float l)
{
	auto toolDir = calculateToolDir(Omega);
	auto grasp = X + toolDir * l;
	return grasp;
}

Eigen::Quaterniond calculator::Omega2Quaternion(vec3 Omega)
{
	double theta = Omega.norm();
	vec3 axis = Omega.normalized();
	double halfTheta = theta / 2;
	double w = cos(halfTheta);
	double x = sin(halfTheta) * axis.x();
	double y = sin(halfTheta) * axis.y();
	double z = sin(halfTheta) * axis.z();
	Quaterniond q;
	if (w > 0)
	{
		q = Quaterniond(w, x, y, z);
	}
	else
	{
		q = Quaterniond(-w, -x, -y, -z);
	}

	return q;
}

vec3 calculator::Quaternion2Omega(Eigen::Quaterniond q)
{
	double l = q.norm();
	if (abs(l - 1) > 1e-6)
		cout << "quaternion have not been normalized" << endl;
	double halfTheta = acos(q.w());
	vec3 rotAxis(q.x(), q.y(), q.z());
	vec3 omega = rotAxis.normalized() * (halfTheta * 2);

	return omega;
}

Eigen::Matrix<double, 6, 1> calculator::solve_6DOF(vec3 Xh, vec3 Xg, vec3 Omega_h, vec3 Omega_g,
	float graspL,
	vec3 F_c, vec3 T_c,
	mat3 partial_FC_X, mat3 partial_FC_Omega,
	mat3 partial_TC_X, mat3 partial_TC_Omega,
	vec3& updatedX, vec3& updatedOmega, vec3& T_vc,
	int collisionNum, double* times)
{
	float l = graspL;
	vec3 Dir_g = calculateToolDir(Omega_g);
	vec3 Dir_h = calculateToolDir(Omega_h);
	vec3 Xg_grasp = calculateGraspPoint(Xg, Omega_g, l);
	vec3 Xh_grasp = calculateGraspPoint(Xh, Omega_h, l);

	LARGE_INTEGER Freq;
	QueryPerformanceFrequency(&Freq);
	LARGE_INTEGER T1;
	LARGE_INTEGER T2;
	QueryPerformanceCounter(&T1);
	vec3 F_VC = calculateFVC(Xg_grasp, Xh_grasp);
	auto qT = calculate_qt(Omega_h, Omega_g);

	T_vc = calculateTVC(qT);
	QueryPerformanceCounter(&T2);
	double time_cal_F_and_T = TimeCalculation(T1, T2, Freq);
	times[0] = time_cal_F_and_T;
	if (collisionNum > L_MAX)
	{
		F_c = F_c / collisionNum * L_MAX;
		partial_FC_X = partial_FC_X / collisionNum * L_MAX;
		partial_FC_Omega = partial_FC_Omega / collisionNum * L_MAX;
		T_c = T_c / collisionNum * L_MAX;
		partial_TC_X = partial_TC_X / collisionNum * L_MAX;
		partial_TC_Omega = partial_TC_Omega / collisionNum * L_MAX;
	}

	QueryPerformanceCounter(&T1);
	mat3 partial_FVC_X = calculatePartialFVC_X(Xg_grasp, Xh_grasp);
	mat3 partial_TVC_Omega = calculatePartialTVC_Omega(qT);
	mat3 partial_FVC_Omega = calculatePartialFVC_Omega();// zero matrix
	mat3 partial_TVC_X = calculatePartialTVC_X();// zero matrix
	QueryPerformanceCounter(&T2);
	double time_cal_partial = TimeCalculation(T1, T2, Freq);
	times[1] = time_cal_partial;

	Matrix<double, 6, 6> mat;

	mat.block(0, 0, 3, 3) = partial_FC_X + partial_FVC_X;
	mat.block(0, 3, 3, 3) = partial_FC_Omega + partial_FVC_Omega;
	mat.block(3, 0, 3, 3) = partial_TC_X + partial_TVC_X;
	mat.block(3, 3, 3, 3) = partial_TC_Omega + partial_TVC_Omega;

	vec6 vec;
	vec.block(0, 0, 3, 1) = F_c + F_VC;

	vec.block(3, 0, 3, 1) = T_c + T_vc;
	vec = -vec; // 移动到等号右边，符号取负

	QueryPerformanceCounter(&T1);
	vec6 delta = mat.lu().solve(vec);
	QueryPerformanceCounter(&T2);
	double time_simple_solve = TimeCalculation(T1, T2, Freq);
	times[2] = time_simple_solve;

	QueryPerformanceCounter(&T1);

	QueryPerformanceCounter(&T2);
	double time_solve_degen = TimeCalculation(T1, T2, Freq);
	times[3] = time_solve_degen;

	QueryPerformanceCounter(&T1);
	// damping and limit max move speed
	vec3 delta_3dof = delta.block(0, 0, 3, 1);
	delta_3dof = delta_3dof * damping;
	if (l2dis(delta_3dof) > DELTA_X_MAX)
	{
		delta_3dof = delta_3dof / l2dis(delta_3dof) * DELTA_X_MAX;
	}
	// limit max Omega 限制最大角速度
	vec3 dir = Dir_g;
	vec3 deltaOmega = delta.block(3, 0, 3, 1);
	vec3 new_g_omega = Omega_g;
	vec3 limitedDeltaOmega;
	double theta = deltaOmega.norm();


	if (theta > 1e-6)
	{
		// delta omega物理定义为一个旋转kAxis
		if (theta > DELTA_ANGLE_MAX)
			limitedDeltaOmega = deltaOmega.normalized() * DELTA_ANGLE_MAX;
		else
			limitedDeltaOmega = deltaOmega;
		auto deltaQ = Omega2Quaternion(limitedDeltaOmega);
		auto quat_g = Omega2Quaternion(Omega_g);
		auto new_quat_g = deltaQ * quat_g;
		if (new_quat_g.w() < 0)
		{
			new_quat_g = Quaterniond(-new_quat_g.w(), -new_quat_g.x(), -new_quat_g.y(), -new_quat_g.z());
		}
		vec3 axis = new_quat_g.vec().normalized();
		double quat_g_theta = acos(new_quat_g.w()) * 2;
		new_g_omega = axis * quat_g_theta;
		new_g_omega = Quaternion2Omega(new_quat_g);

	}

	delta.block(0, 0, 3, 1) = delta_3dof;
	QueryPerformanceCounter(&T2);
	double time_solve_rotation = TimeCalculation(T1, T2, Freq);
	times[4] = time_solve_rotation;

	auto Xg_grasp_new = Xg_grasp + delta_3dof;
	auto Xg_dir_new = calculateToolDir(new_g_omega);
	auto Xg_tip_new = Xg_grasp_new - Xg_dir_new * l;
	updatedX = Xg_tip_new;
	updatedOmega = new_g_omega;

	return delta;
}

mat3 calculator::SkewSymmetricMatrix(vec3 v)
{
	mat3 m;
	m << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return m;
}

vec3 calculator::calculateFVC(vec3 Xg, vec3 Xh)
{
	double r = l2dis(Xg, Xh);
	vec3 e_r;
	if (r > EPS)// 避免除零错误
		e_r = (Xg - Xh) / r;
	else
		e_r = vec3(0, 0, 0);
	double f_r = saturationMapping(r);
	vec3 F_VC = -f_r * e_r;
	return F_VC;
}

vec3 calculator::calculateR(vec3 p, vec3 Xg_grasp)
{
	vec3 r = p - Xg_grasp;
	return r;
}

vec3 calculator::calculateFC(vec3 N0, double d)
{
	if (d > 0)
	{
		cout << "depth should less than 0!" << endl;
		return vec3(0, 0, 0);
	}

	vec3 Fc = m_k_c * d * N0;
	return Fc;
}

vec3 calculator::calculateTC(vec3 r, vec3 Fc)
{
	mat3 r_tilde = SkewSymmetricMatrix(r);
	vec3 Tc = r_tilde * Fc;
	return Tc;
}

mat3 calculator::calculatePartialFC_X(vec3 N0)
{
	mat3 partialFC_X = -m_k_c * N0 * N0.transpose();
	return partialFC_X;
}

mat3 calculator::calculatePartialFC_Omega(vec3 r, vec3 N0, double d)
{
	mat3 r_tilde = SkewSymmetricMatrix(r);
	mat3 part1 = m_k_c * N0 * N0.transpose() * r_tilde;
	mat3 part2 = m_k_c * d * SkewSymmetricMatrix(N0);
	return part1 + part2;
}

mat3 calculator::calculatePartialTC_X(vec3 Fc, vec3 r, vec3 N0)
{
	mat3 part1 = SkewSymmetricMatrix(Fc);
	mat3 part2 = -m_k_c * SkewSymmetricMatrix(r) * (N0 * N0.transpose());
	return part1 + part2;
}

mat3 calculator::calculatePartialTC_Omega(vec3 Fc, vec3 r, vec3 N0, double d)
{
	mat3 r_tilde = SkewSymmetricMatrix(r);
	mat3 part1 = -SkewSymmetricMatrix(Fc) * r_tilde;
	mat3 part2 = m_k_c * r_tilde * (N0 * N0.transpose()) * r_tilde;
	mat3 part3 = m_k_c * d * r_tilde * SkewSymmetricMatrix(N0);
	return part1 + part2 + part3;
}

void calculator::calculateForce6DOF(vector<float> qh, vector<float> qg, float graspL, vec3& Fvc, vec3& Tvc)
{
	vec3 Xg(qg[0], qg[1], qg[2]);
	vec3 Omega_g(qg[3], qg[4], qg[5]);
	vec3 Xh(qh[0], qh[1], qh[2]);
	vec3 Omega_h(qh[3], qh[4], qh[5]);
	vec3 Xg_grasp = calculateGraspPoint(Xg, Omega_g, graspL);
	vec3 Xh_grasp = calculateGraspPoint(Xh, Omega_h, graspL);
	Fvc = calculateFVC(Xg_grasp, Xh_grasp);
	Tvc = calculateTVC(Omega_h, Omega_g);
}

