#include "gpuvar.h"
#include "gpufun.h"

#pragma region 四面体与表面三角形相互作用的
int* tetVert2SpringVertMapping_d;			// 距离最近的表面三角顶点下标， tetVertNum_d

int* springVert2TetVertMapping_d;  // 距离最近的四面体顶点下标， springVertNum_d * 2
float* springVertfromTetStiffness_d;				//表面网格对四面体的restpos刚度系数
#pragma endregion 


#pragma region 四面体形变
int				tetVertNum_d;				//四面体网格顶点数量
int				tetNum_d;					//四面体数量
float             gravityX_d;//重力
float				gravityY_d;//重力
float				gravityZ_d;//重力
float* tetVertPos_d;				//当前位置，tetVertNum_d*3
float* tetVertRestPos_d;				//初始位置，tetVertNum_d*3
float* tetVertPos_last_d;			//上一时刻位置，tetVertNum_d*3
float* tetVertPos_old_d;			//st，tetVertNum_d*3
float* tetVertPos_prev_d;			//上一次迭代，tetVertNum_d*3
float* tetVertPos_next_d;			//下一步位置，tetVertNum_d*3
float* tetVertVelocity_d;			//速度，tetVertNum_d*3
float* tetVertExternForce_d;		//外力，tetVertNum_d*3
float* tetVertMass_d;				//质量，tetVertNum_d*3
int* tetIndex_d;					//四面体索引
float* tetVertFixed_d;				//四面体顶点是否固定，0.0f表示没有固定，tetVertNum_d
bool* tetActive_d;				//四面体是否是激活的，是否参与变形，tetNum_d
float* tetInvD3x3_d;				//逆矩阵, tetNum_d*9
float* tetInvD3x4_d;				//Ac阵， tetNum_d*12
float* tetVolume_d;				//四面体体积，tetNum_d
float* tetVolumeDiag_d;			//四面体顶点形变梯度，tetVertNum_d
float* tetVertCollisionForce_d;
float* tetVertCollisionDiag_d;		//四面体顶点上的碰撞力梯度，tetVertNum_d*3
float* tetVertForce_d;					//顶点受力, tetVertNum_d*3
float* tetStiffness_d;				//四面体弹性系数，tetNum_d
float* tetVertToolDistance_d; // 四面体顶点离最近的碰撞体的距离
int* mapTetVertIdx2TriVertIdx_d; // 从四面体顶点下标映射到绑定的表面三角形顶点下标。内部的对应为-1；
//四面体顶点是否处于有效状态
bool* tetVertActive_d;
float* tetVertPos_forTri;


float* tetVertNorm_d; // 外表面三角网格顶点的法向量，对应数组中的部分元素，tetVertNum_d*3
float* tetVertNormAccu_d; // tetVertNum_d
#pragma endregion 


#pragma region 几何模型渲染
int		triNum_d; 				  //三角网格三角数量
// triangle
int* triIndex_d;		  // 三角网格上三角形三个顶点对应的顶点下标，3*triNum_d
#pragma endregion

#pragma region 弹簧模型形变
// vertex
int		springVertNum_d;		  //弹簧模型顶点数量
int		springNum_d;			  //弹簧数量

float* springVertPos_d;			  //弹簧模型顶点，3*springVertNum_d
float* springVertPos_old_d;		  // 3*springVertNum_d
float* springVertPos_prev_d;		  // 3*springVertNum_d
float* springVertPos_next_d;		  // 3*springVertNum_d
float* springVertVelocity_d;		  // 3*springVertNum_d
float* springVertExternForce_d;	  // 3*springVertNum_d
float* springVertMass_d;			  // 弹簧模型顶点质量，triVertNum_d

bool* springActive_d;
bool* springVertActive_d;

float* springVertInsertionDepth_d; // 存储表面顶点在工具中的嵌入深度
float* springVertProjectedPos_d; // 三角网格顶点碰撞之后被投影到的工具表面位置，如果未发生碰撞，维持顶点原位， triVertNum_d*3
float* springVertToolDistance_d; // 弹簧顶点离最近的碰撞体的距离
float* springVertNonPenetrationDir_d;// 弹簧模型顶点指导向量, springVertNum_d*3
//spring
unsigned int* springIndex_d;	  // 弹簧对应的顶点下标， 2*springNum_d
float* springOrgLength_d;		  // 弹簧原长， springNum_d
float* springDiag_d;			  // 弹簧顶点上因弹簧力产生的梯度，3*springVertNum_d
float* springVertCollisionDiag_d;	  // 弹簧顶点上因碰撞力产生的梯度，3*springVertNum_d

float* springVertFixed_d;			  // 弹簧顶点是否是固定的，0.0为非固定顶点，triVertNum_d
float* springVertForce_d;			  // 弹簧顶点上的力，3*springVertNum_d
float* springStiffness_d;		  // 弹簧的弹簧刚度， springNum_d
#pragma endregion 



#pragma region 碰撞相关
unsigned char* tetisCollide_d;
unsigned char* tetVertisCollide_d;
int* tetVertCollisionCnt_d;       // 每个顶点的碰撞计数，因为和四面体碰撞时有可能碰撞多次
int* tetVertCollisionToolFlag_d; // 顶点是和左工具还是右工具碰撞的, left = 1, right = 2
float* tetVertCollisionPos_d;
float* tetVertCollisionDepth_d;
float* tetVertCollidedBuffer_d;
float* tetVertCollidedNonPenetration_d;
float* tetVertCollidedDepth_d;
float* tetVertCollidedPos_d;
int* tetVertCollidedToolFlag_d;
unsigned int* tetVertQueueIndex_d;
unsigned int* tetVertAuxSumArray_d;
float* tetVertColVelocityMax_d;

unsigned char* springVertisCollide_forTet;
float* springVertPos_forTet;
float* springVertCollisionDepth_forTet;
float* springVertCollisionPos_forTet;
int* springVertCollisionToolFlag_forTet;
int* springVert2TetVertMapping_forTet;

unsigned char* springVertisCollide_d; // 三角网格顶点是否碰撞， springVertNum_d
unsigned int* springVertQueueIndex_d; // 三角形碰撞队列编号， springVertNum_d
unsigned int* springVertAuxSumArray_d; // 三角形碰撞信息前缀和，用于整合碰撞信息到一个队列中 springVertNum_d
float* springVertCollidedBuffer_d; // 发生碰撞的表面三角顶点， springVertNum_d*3
float* springVertCollidedNonPenetration_d; // 发生碰撞的表面三角顶点指导向量，与表面三角网格法向量方向相反， springVertNum_d*3
float* springVertCollidedDepth_d; // 发生碰撞的表面顶点的嵌入深度，为负数， springVertNum_d
float* springVertCollidedPos_d; // 发生碰撞的表面顶点的投影位置 springVertNum_d*3
int* springVertCollidedToolFlag_d;// 表面三角形顶点被碰撞的工具编号。

float* springVertCollisionPos_d; // 表面三角形顶点位置
float* springVertCollisionForce_d;
float* springVertCollisionNormal_d;// 表面三角形顶点被碰撞后的投影位置
float* springVertCollisionDepth_d;//// 表面三角形顶点被碰撞之后的
unsigned int* springVertCollisionCnt_d;//// 表面三角形顶点碰撞数量？
int* springVertCollisionToolFlag_d;// 表面三角形被碰撞的工具编号。
#pragma endregion 

float* springVertNorms_d;

#pragma region 碰撞体参数
// 圆柱体-CPU输入参数
int cylinder_num;			// 圆柱体的数量
char* cylinder_active;		// 圆柱体是否激活，sizeof(char) * cylinder_num
float* cylinder_pos;		// 圆柱体位置，3 * sizeof(float) * cylinder_num
float* cylinder_dir;		// 圆柱体长轴的方向， 3 * sizeof(float) * cylinder_num
float* cylinder_radius;		// 圆柱体半径，sizeof(float) * cylinder_num
float* cylinder_length;		// 圆柱体长度，sizeof(float) * cylinder_num

float* cylinder_pos_haptic;	// 力反馈碰撞使用的圆柱体位置
float* cylinder_dir_haptic; // 力反馈碰撞使用的圆柱体方向
float* cylinder_last_pos_haptic;
// 圆柱体-GPU中间变量
float* cylinder_shift_tv;		// 圆柱体的偏移向量， 3 * sizeof(float) * cylinder_num
float* cylinder_shift_sv;		// 圆柱体的偏移向量， 3 * sizeof(float) * cylinder_num
float* cylinder_last_pos;	// 上一帧的位置，3 * sizeof(float) * cylinder_num
// 不是很有用
unsigned char* cylinder_collide_flag; // 圆柱体是否碰撞（两层网格共用），sizeof(char) * cylinder_num

// 球-CPU输入参数
int sphere_num;				// 球的数量
char* sphere_active;		// 球是否激活，sizeof(cahr) * sphere_num
float* sphere_pos;			// 球心位置， 3 * sizeof(float) * sphere_num
float* sphere_radius;		// 球半径，sizeof(float) * sphere_num
// 球-GPU中间变量
float* sphere_shift;		// 球的偏移向量， 3 * sizeof(float) * sphere_num
float* sphere_last_pos;		// 上一帧的位置，3 * sizeof(float) * sphere_num
#pragma endregion

#pragma region 夹取参数
// CPU输入
int gripper_num;			// 夹钳器械的数量
char* gripper_active;		// 夹钳是否激活，sizeof(char) * gripper_num
float* gripper_pos;			// 就是杆子的原点，3 * sizeof(float) * gripper_num
float* gripper_scale;		// 夹子（长方体）的边长，3 * sizeof(float) * gripper_num
float* gripper_pivot_x;		// 主柄的x轴方向，3 * sizeof(float) * gripper_num
float* gripper_pivot_y;		// 主柄的y轴方向，3 * sizeof(float) * gripper_num
float* gripper_pivot_z;		// 主柄的z轴方向，3 * sizeof(float) * gripper_num
float* gripper_upper_x;		// 上夹子的x轴方向，3 * sizeof(float) * gripper_num
float* gripper_upper_y;		// 上夹子的y轴方向，3 * sizeof(float) * gripper_num
float* gripper_upper_z;		// 上夹子的z轴方向，3 * sizeof(float) * gripper_num
float* gripper_lower_x;		// 下夹子的x轴方向，3 * sizeof(float) * gripper_num
float* gripper_lower_y;		// 下夹子的y轴方向，3 * sizeof(float) * gripper_num
float* gripper_lower_z;		// 下夹子的z轴方向，3 * sizeof(float) * gripper_num
float* gripper_angle;		// 上下夹子的夹角，sizeof(float) * gripper_num

// GPU中间变量
bool* gripper_unclose_tv;	// 夹钳第一次闭合的标志，1 * sizeof(bool) * gripper_num
unsigned int* tv_grab_flag;		// 四面体顶点处于夹取区域的标志，1 * sizeof(unsigned int) * gripper_num * tetVertNum
unsigned int* tv_grab_half_flag;	// 四面体顶点在中间过程的碰撞位，1/2代表两个夹子，1 * sizeof(unsigned int) * gripper_num * tetVertNum
float* tv_relative_pos;			// 四面体在夹钳中的局部坐标，3 * sizeof(float) * gripper_num * tetVertNum
unsigned int* sv_grab_flag;

// 输出
float* gripper_adsorb_force; // 夹取力，3 * sizeof(float) * gripper_num
int* gripper_grab_num; // 夹取区域的点的数量，sizeof(int) * gripper_num

#pragma endregion

#pragma region 力反馈参数
int* hapticCollisionNum_d;
int* hapticCollisionNumForSim_d;
int* tvCollisionNum_d;
#pragma endregion

