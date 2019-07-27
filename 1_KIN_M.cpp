/////////////////////////////////////////////////////////////////////////
// 작성일 : 2005.10.26
// 제  목 : 기구학 관련 함수 모음
// 내  용 : 정/역 기구학 풀이 및 좌표 변환용 Util등.
//          현재 사용중인 뭉치(Mungchi)를 위한 기구학 풀이 내용이 있다.
/////////////////////////////////////////////////////////////////////////
#include <stdint.h>

#include "define.h"
#include "debug.h"
#include "kinhead.h"

//***************************************************************************************/
// 공유 변수
//***************************************************************************************/
// 로봇의 현재 위치를 지정
int g_LEG_Position[2][6];	// AX-12의 위치로 저장.
int g_ARM_Position[2][4];	// AX-12의 위치로 저장.

// 각 관절의 무게중심의 위치
const double P_C_MC[3] = { -5,-37,  0};
const double P_3_M3[3] = { 16, 17,  0};
const double P_4_M4[3] = { 14,-14,  0};
const double P_6_M6[3] = {-16,  0,-17};

// P_A1_MA1 = P_AR1_MAR1
// P_AL1_MAL1 = { 60,-27,-76.2}; // 하지만 여기서는 P_AR1_MAR1을 이용한다.	
const double P_A1_MA1[3] = { 60,-27, 76.2};
// P_A3_MA3 = P_AR3_MAR3 =  P_AL3_MAL3 
const double P_A3_MA3[3]   = { 24,  0,  0.0};
// P_A4_MA4 = P_AR4_MAR4 =  P_AL4_MAL4	// 1mm차이, 그냥 0으로 단순화 
// P_AR4_MAR4={26,-1,0} / P_AL4_MAL4={ 26, 1,0} 
const double P_A4_MA4[3]   = { 26,  0,  0.0};




// 다리의 각도 -> AX-12위치 변환식 -> 상수 이다!!!
const double LEG_BASE[][6]   = {{   0,   0, 360,   0,   0, 360}, {   0,   0,   0, 360, 360, 360}};
const double LEG_SIGN[][6]   = {{   1,   1,  -1,   1,   1,  -1}, {   1,   1,   1,  -1,  -1,  -1}};
const double LEG_DtoP[][6]   = {{ 180, 180, 169.2417, 111.5165, 169.2417, 180},
                                { 180, 180, 169.2417, 111.5165, 169.2417, 180}};

const double ARM_BASE[][4]   = {{   0, 360, 360, 360}, {   0,   0, 360, 360}};
const double ARM_SIGN[][4]   = {{   1,  -1,  -1,  -1}, {   1,   1,  -1,  -1}};
// Forward [+], Forward [-], RIGHT Outside [+] / LEFT Outside [-] 
// 조립을 잘못해서 좌우팔의 ID를 바꿨다.
// const double ARM_DtoP[][4]   = {{  96.254, 173.746, 270, 180},
//                                {  96.254, 173.746,  90, 180}};
const double ARM_DtoP[][4]   = {{  96.254, 173.746,  90, 180},
                                {  96.254, 173.746, 270, 180}};
/*
//const double LEG_MIN_DEG[][6]    = {{ -30, -40,-110, -20, -80, -40}, { -30, -40,-110, -20, -80, -40}};
//const double LEG_MAX_DEG[][6]    = {{  30,  40,  30, 120,  30,  40}, {  30,  40,  30, 120,  30,  40}};
const double LEG_MIN_RAD[][6] = {{-0.5236,-0.6981,-1.9199,-0.3491,-1.3963,-0.6981},
								 {-0.5236,-0.6981,-1.9199,-0.3491,-1.3963,-0.6981}};
const double LEG_MAX_RAD[][6] = {{ 0.5236, 0.6981, 0.5236, 2.0944, 0.5236, 0.6981},
								 { 0.5236, 0.6981, 0.5236, 2.0944, 0.5236, 0.6981}};
*/
// 각도 제한 없음.
const double LEG_MIN_RAD[][6] = {{-100,-100,-100,-100,-100,-100},
								 {-100,-100,-100,-100,-100,-100}};
const double LEG_MAX_RAD[][6] = {{ 100, 100, 100, 100, 100, 100},
								 { 100, 100, 100, 100, 100, 100}};

const double LEG_WEIGHT[]  = {G_C_M6, G_C_M4, G_C_M3, G_C_MC, G_C_M3, G_C_M4, G_C_M6};
const double BODY_WEIGHT[] = {G_C_MA1,G_C_MA3,G_C_MA4,G_C_MA3,G_C_MA4};
const double ALL_WEIGHT[]  = {G_C_M6, G_C_M4, G_C_M3, G_C_MC, G_C_M3, G_C_M4, G_C_M6,
							  G_C_MA1,G_C_MA3,G_C_MA4,G_C_MA3,G_C_MA4};

//************************************************
// 각종 변환 매트릭스
//************************************************
// 다리
// T_C_R0 = T_C_0[RIGHT_LEG]
// T_C_L0 = T_C_0[LEFT_LEG]
const double T_C_0[2][3][4]  = {{{ 0, 0,-1,-50.0},
				 				 { 0, 1, 0,-14.5},
						     	 { 1, 0, 0,-32.7}},
							 	{{ 0, 0,-1,-50.0},
						     	 { 0, 1, 0,-14.5},
						     	 { 1, 0, 0, 32.7}}};

const double T_R0_C[3][4]  = {{ 0, 0, 1, 32.7},
				 		      { 0, 1, 0, 14.5},
						      {-1, 0, 0,-50.0}};
const double T_L0_C[3][4]  = {{ 0, 0, 1,-32.7},
						      { 0, 1, 0, 14.5},
						      {-1, 0, 0,-50.0}};

// T_R6_F = T_L6_F
const double T_R6_F[3][4] = {{ 1, 0, 0, 32.2},
						     { 0, 1, 0,  0.0},
						     { 0, 0, 1,  0.0}};

// T_F_R6 = T_F_L6
const double T_F_R6[3][4] = {{ 1, 0, 0,-32.2},
						     { 0, 1, 0,  0.0},
						     { 0, 0, 1,  0.0}};

// 중심에서 발까지의 회전 매트릭스
const double R_C_F[][3]  = {{-1, 0, 0},
						    { 0, 0, 1},
						    { 0, 1, 0}};
// 상체
// T_R6_F = T_L6_F
const double T_BC_R0[3][4] = {{ 1, 0, 0,  0.0},
						      { 0, 1, 0,  0.0},
						      { 0, 0, 1,-76.2}};
const double T_BC_L0[3][4] = {{ 1, 0, 0,  0.0},
						      { 0, 1, 0,  0.0},
						      { 0, 0, 1, 76.2}};

//***************************************************************************************/
//                Forward Kinematics
//
// joint 각도(degree), Transform Matrix(result)
//***************************************************************************************/
// Case 2 : 책의 방법대로 좌표 정의
// T03
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : T03
void T03_for_M2(const double jt_deg[6], double mtx[3][4])
{
	double jt_rad[6];
	double c1, c2, c3;
	double s1, s2, s3;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = (jt_deg[1]-90) * DEGTORAD;
	jt_rad[2] = jt_deg[2] * DEGTORAD;

	// sin, cos 값 계산.
	c1 = cos(jt_rad[0]);
	c2 = cos(jt_rad[1]);
	c3 = cos(jt_rad[2]);

	s1 = sin(jt_rad[0]);
	s2 = sin(jt_rad[1]);
	s3 = sin(jt_rad[2]);

	// T03
	mtx[0][0] =  c1*c2*c3 + s1*s3;
	mtx[0][1] = -c1*c2*s3 + s1*c3;
	mtx[0][2] = -c1*s2;
	mtx[0][3] =  0;

	mtx[1][0] =  s1*c2*c3 - c1*s3;
	mtx[1][1] = -s1*c2*s3 - c1*c3;
	mtx[1][2] = -s1*s2;
	mtx[1][3] =  0;

	mtx[2][0] = -s2*c3;
	mtx[2][1] =  s2*s3;
	mtx[2][2] = -c2;
	mtx[2][3] =  0;
}

// T_0_4
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : T04
void T04_for_M2(const double jt_deg[6], double mtx[3][4])
{
	double jt_rad[6];
	double c1, c2, c3, c34;
	double s1, s2, s3, s34;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = (jt_deg[1]-90) * DEGTORAD;
	// jt_rad[1] = jt_deg[1] * DEGTORAD;
	jt_rad[2] = jt_deg[2] * DEGTORAD;
	jt_rad[3] = jt_deg[3] * DEGTORAD;

	// sin, cos 값 계산.
	c1 = cos(jt_rad[0]);
	c2 = cos(jt_rad[1]);
	c3 = cos(jt_rad[2]);
	c34 = cos(jt_rad[2]+jt_rad[3]);

	s1 = sin(jt_rad[0]);
	s2 = sin(jt_rad[1]);
	s3 = sin(jt_rad[2]);
	s34 = sin(jt_rad[2]+jt_rad[3]);

	// T04
	mtx[0][0] =  c1*c2*c34 + s1*s34;
	mtx[0][1] = -c1*c2*s34 + s1*c34;
	mtx[0][2] = -c1*s2;
	mtx[0][3] =  DH_a3*(c1*c2*c3 + s1*s3) - DH_d2*s1;

	mtx[1][0] =  s1*c2*c34-c1*s34;
	mtx[1][1] = -s1*c2*s34-c1*c34;
	mtx[1][2] = -s1*s2;
	mtx[1][3] =  DH_a3*(s1*c2*c3 - c1*s3) + DH_d2*c1;

	mtx[2][0] = -s2*c34;
	mtx[2][1] =  s2*s34;
	mtx[2][2] = -c2;
	mtx[2][3] = -DH_a3*s2*c3;
}

void T06_for_M2(const double jt_deg[6], double mtx[3][4])
{
	double jt_rad[6];
	double c1, c2, c3, c34, c345, c6;
	double s1, s2, s3, s34, s345, s6;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = (jt_deg[1]-90) * DEGTORAD;
	// jt_rad[1] = jt_deg[1] * DEGTORAD;
	jt_rad[2] = jt_deg[2] * DEGTORAD;
	jt_rad[3] = jt_deg[3] * DEGTORAD;
	jt_rad[4] = jt_deg[4] * DEGTORAD;
	jt_rad[5] = jt_deg[5] * DEGTORAD;

	// sin, cos 값 계산.
	c1 = cos(jt_rad[0]);
	c2 = cos(jt_rad[1]);
	c3 = cos(jt_rad[2]);
	c34 = cos(jt_rad[2]+jt_rad[3]);
	c345 = cos(jt_rad[2]+jt_rad[3]+jt_rad[4]);
	c6 = cos(jt_rad[5]);

	s1 = sin(jt_rad[0]);
	s2 = sin(jt_rad[1]);
	s3 = sin(jt_rad[2]);
	s34 = sin(jt_rad[2]+jt_rad[3]);
	s345 = sin(jt_rad[2]+jt_rad[3]+jt_rad[4]);
	s6 = sin(jt_rad[5]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c1*c2*c6*c345 + s1*c6*s345 - c1*s2*s6;
	mtx[0][1] = -c1*c2*s6*c345 - s1*s6*s345 - c1*s2*c6;
	mtx[0][2] =  c1*c2*s345 - s1*c345;
	mtx[0][3] =  DH_a3*(c1*c2*c34 + s1*s34 + c1*c2*c3 + s1*s3) - DH_d2*s1;

	mtx[1][0] =  s1*c2*c6*c345 - c1*c6*s345 - s1*s2*s6;
	mtx[1][1] = -s1*c2*s6*c345 + c1*s6*s345 - s1*s2*c6;
	mtx[1][2] =  s1*c2*s345 + c1*c345;
	mtx[1][3] =  DH_a3*(s1*c2*c34 - c1*s34 + s1*c2*c3 - c1*s3) + DH_d2*c1;

	mtx[2][0] = -s2*c6*c345 - c2*s6;
	mtx[2][1] =  s2*s6*c345 - c2*c6;
	mtx[2][2] = -s2*s345;
	mtx[2][3] =  DH_a3*(-s2*c34 - s2*c3);
}

// T_6_4
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : T64
void T64_for_M2(const double jt_deg[6], double mtx[3][4])
{
	double jt_rad[6];
	double c5, c6;
	double s5, s6;

	// radian으로 변환.
	jt_rad[4] = jt_deg[4] * DEGTORAD;
	jt_rad[5] = jt_deg[5] * DEGTORAD;

	// sin, cos 값 계산.
	c5 = cos(jt_rad[4]);
	c6 = cos(jt_rad[5]);

	s5 = sin(jt_rad[4]);
	s6 = sin(jt_rad[5]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c5*c6;
	mtx[0][1] =  s5*c6;
	mtx[0][2] =  s6;
	mtx[0][3] = -DH_a3*(c5*c6);

	mtx[1][0] = -c5*s6;
	mtx[1][1] = -s5*s6;
	mtx[1][2] =  c6;
	mtx[1][3] =  DH_a3*(c5*s6);

	mtx[2][0] =  s5;
	mtx[2][1] = -c5;
	mtx[2][2] =  0;
	mtx[2][3] = -DH_a3*(s5);
}

// T_4_3
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : T43
void T43_for_M2(const double jt_deg[6], double mtx[3][4])
{
	double jt_rad[6];
	double c4;
	double s4;

	// radian으로 변환.
	jt_rad[3] = jt_deg[3] * DEGTORAD;

	// sin, cos 값 계산.
	c4 = cos(jt_rad[3]);

	s4 = sin(jt_rad[3]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c4;
	mtx[0][1] =  s4;
	mtx[0][2] =  0;
	mtx[0][3] = -DH_a3*(c4);

	mtx[1][0] = -s4;
	mtx[1][1] =  c4;
	mtx[1][2] =  0;
	mtx[1][3] =  DH_a3*(s4);

	mtx[2][0] =  0;
	mtx[2][1] =  0;
	mtx[2][2] =  1;
	mtx[2][3] =  0;
}

// T_C_F
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : TCF
void TCF_for_M2(const double jt_deg[6], double mtx[3][4], int LEG)
{
    double mtx2[3][4];

    T06_for_M2(jt_deg, mtx);
    Mul_TT(T_C_0[LEG], mtx, mtx2);     // mtx2 = TC6
    Mul_TT(mtx2,T_R6_F, mtx);        // mtx1 = TC6*T6F = TCF
}

/////////////////////////////////////
// 상체의 정기구학
/////////////////////////////////////
// Case 2 : 책의 방법대로 좌표 정의
// T05
// [입력] jt_deg : 각 관절의 각도(deg)
// [출력] mtx : T03
void T05_for_M2_BODY(const double jt_deg[4], double mtx[3][4])
{
	double jt_rad[4];
	double c1, c3, c12, c34;
	double s1, s3, s12, s34;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = jt_deg[1] * DEGTORAD;
	jt_rad[2] = (jt_deg[2]+180) * DEGTORAD;
	// jt_rad[2] = jt_deg[2] * DEGTORAD;
	jt_rad[3] = jt_deg[3] * DEGTORAD;

	// sin, cos 값 계산.
	c1  = cos(jt_rad[0]);
	c3  = cos(jt_rad[2]);
	c12 = cos(jt_rad[0]+jt_rad[1]);
	c34 = cos(jt_rad[2]+jt_rad[3]);

	s1  = sin(jt_rad[0]);
	s3  = sin(jt_rad[2]);
	s12 = sin(jt_rad[0]+jt_rad[1]);
	s34 = sin(jt_rad[2]+jt_rad[3]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c12*c34;
	mtx[0][1] = -c12*s34;
	mtx[0][2] = -s12;
	mtx[0][3] =  B_DH_a4*c12*c34 + B_DH_a3*c12*c3 + B_DH_a2*c12 + B_DH_a1*c1;

	mtx[1][0] =  s12*c34;
	mtx[1][1] = -s12*s34;
	mtx[1][2] =  c12;
	mtx[1][3] =  B_DH_a4*s12*c34 + B_DH_a3*s12*c3 + B_DH_a2*s12 + B_DH_a1*s1;

	//mtx[2][0] = s34;
	mtx[2][0] = -s34;
	mtx[2][1] = -c34;
	mtx[2][2] =  0;
	mtx[2][3] =  B_DH_a4*s34 - B_DH_a3*s3;
}

/* <<오류예감>> ******************************
* 웬지 풀이랑 다른 것 같다. 왜지?
* 상체는 한번도 검증한 적이 없다.
* 좌우 팔 바뀐것도 해결해야 함.
**********************************************/   
    
void T01_for_M2_BODY(const double jt_deg[4], double mtx[3][4])
{
	double jt_rad[4];
	double c1;
	double s1;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;

	// sin, cos 값 계산.
	c1  = cos(jt_rad[0]);
	s1  = sin(jt_rad[0]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c1;
	mtx[0][1] = -s1;
	mtx[0][2] =  0;
	mtx[0][3] =  0;

	mtx[1][0] =  s1;
	mtx[1][1] =  c1;
	mtx[1][2] =  0;
	mtx[1][3] =  0;

	mtx[2][0] =  0;
	mtx[2][1] =  0;
	mtx[2][2] =  1;
	mtx[2][3] =  0;
}

void T02_for_M2_BODY(const double jt_deg[4], double mtx[3][4])
{
	double jt_rad[4];
	double c1, c12;
	double s1, s12;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = jt_deg[1] * DEGTORAD;

	// sin, cos 값 계산.
	c1  = cos(jt_rad[0]);
	c12 = cos(jt_rad[0]+jt_rad[1]);

	s1  = sin(jt_rad[0]);
	s12 = sin(jt_rad[0]+jt_rad[1]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c12;
	mtx[0][1] = -s12;
	mtx[0][2] =  0;
	mtx[0][3] =  B_DH_a1*c1;

	mtx[1][0] =  s12;
	mtx[1][1] =  c12;
	mtx[1][2] =  0;
	mtx[1][3] =  B_DH_a1*s1;

	mtx[2][0] =  0;
	mtx[2][1] =  0;
	mtx[2][2] =  1;
	mtx[2][3] =  0;
}

void T03_for_M2_BODY(const double jt_deg[4], double mtx[3][4])
{
	double jt_rad[4];
	double c1, c3, c12;
	double s1, s3, s12;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = jt_deg[1] * DEGTORAD;
	jt_rad[2] = (jt_deg[2]+180) * DEGTORAD;
	// jt_rad[2] = jt_deg[2] * DEGTORAD;
	jt_rad[3] = jt_deg[3] * DEGTORAD;

	// sin, cos 값 계산.
	c1  = cos(jt_rad[0]);
	c3  = cos(jt_rad[2]);
	c12 = cos(jt_rad[0]+jt_rad[1]);

	s1  = sin(jt_rad[0]);
	s3  = sin(jt_rad[2]);
	s12 = sin(jt_rad[0]+jt_rad[1]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c12*c3;
	mtx[0][1] = -c12*s3;
	mtx[0][2] = -s12;
	mtx[0][3] =  B_DH_a2*c12 + B_DH_a1*c1;

	mtx[1][0] =  s12*c3;
	mtx[1][1] = -s12*s3;
	mtx[1][2] =  c12;
	mtx[1][3] =  B_DH_a2*s12 + B_DH_a1*s1;

	mtx[2][0] = -s3;
	mtx[2][1] = -c3;
	mtx[2][2] =  0;
	mtx[2][3] =  0;
}

void T04_for_M2_BODY(const double jt_deg[4], double mtx[3][4])
{
	double jt_rad[4];
	double c1, c3, c12, c34;
	double s1, s3, s12, s34;

	// radian으로 변환.
	jt_rad[0] = jt_deg[0] * DEGTORAD;
	jt_rad[1] = jt_deg[1] * DEGTORAD;
	jt_rad[2] = (jt_deg[2]+180) * DEGTORAD;
	// jt_rad[2] = jt_deg[2] * DEGTORAD;
	jt_rad[3] = jt_deg[3] * DEGTORAD;

	// sin, cos 값 계산.
	c1  = cos(jt_rad[0]);
	c3  = cos(jt_rad[2]);
	c12 = cos(jt_rad[0]+jt_rad[1]);
	c34 = cos(jt_rad[2]+jt_rad[3]);

	s1  = sin(jt_rad[0]);
	s3  = sin(jt_rad[2]);
	s12 = sin(jt_rad[0]+jt_rad[1]);
	s34 = sin(jt_rad[2]+jt_rad[3]);

	// Forword Kinematics 계산.
	mtx[0][0] =  c12*c34;
	mtx[0][1] = -c12*s34;
	mtx[0][2] = -s12;
	mtx[0][3] =  B_DH_a3*c12*c3 + B_DH_a2*c12 + B_DH_a1*c1;

	mtx[1][0] =  s12*c34;
	mtx[1][1] = -s12*s34;
	mtx[1][2] =  c12;
	mtx[1][3] =  B_DH_a3*s12*c3 + B_DH_a2*s12 + B_DH_a1*s1;

	mtx[2][0] = -s34;
	mtx[2][1] = -c34;
	mtx[2][2] =  0;
	mtx[2][3] = -B_DH_a3*s3;
}

//***************************************************************************************
//                Inverse Kinematics
//
// x방향의 peg길이, Transform Matrix, joint 각도(degree, result), state[](해가 존재하는지에 대한 응답)
// 입력 : mtx[3][4]
// 출력 : jt_deg, state
//
// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
//***************************************************************************************
// Case 2 : 책의 방법대로 좌표 정의
int Inverse_Kin_M2(const double mtx[3][4], double jt_deg[MAX_SOL][6],int state[MAX_SOL])
{
	int i,j,k;
	double x_pos, y_pos, z_pos;
	double joint_rad[4][6];

	double th4_a, th4_b, th4_c;

	double s4, c4, c34, c3;
	double th3_a, th3_b, th3_c, th3_abc, th2_c, th2_s;

	int error_count;


	// 초기화
	for (i=0;i<MAX_SOL;i++)	state[i] = 1;

	for (i = 0;i<MAX_SOL;i++)
	{
		for (j = 0;j<6;j++)	jt_deg[i][j] = joint_rad[i][j] = 0;
	}

	// EndEffector Position
	x_pos = mtx[0][3];
	y_pos = mtx[1][3];
	z_pos = mtx[2][3];


	//********************************************************
	// Joint 4,(?deg ~ ?deg, ?rad ~ ?rad)
	// Theta 4
	//********************************************************
	th4_a = (x_pos*x_pos + y_pos*y_pos + z_pos*z_pos - (DH_a3*DH_a3 + DH_a4*DH_a4)) / (2*DH_a3*DH_a4);
	th4_b = 1 - th4_a*th4_a;
	if (th4_b < 0)
	{
		// 해 없음.
		printf("th4_b is less then 0\r\n");
		for (i=0;i<MAX_SOL;i++)	state[i] = 0;
		return 0;
	}
	else
	{
		th4_c = sqrt(th4_b);
	}

	// theta 4의 계산, -pi에서 +pi사이의 값으로 변환.
	joint_rad[SOL_1][3] = atan2(th4_c,th4_a);
	joint_rad[SOL_1][3] = range_rad(joint_rad[SOL_1][3]);

	joint_rad[SOL_2][3] = atan2(-th4_c,th4_a);
	joint_rad[SOL_2][3] = range_rad(joint_rad[SOL_2][3]);

	// ?도 범위를 넘어서면 solution 없음.
	error_count = 0;
	for(i=0;i<MAX_SOL;i=i+2)
	{
		if ((joint_rad[i][3] > LEG_MAX_RAD[RIGHT_LEG][3]) || (joint_rad[i][3] < LEG_MIN_RAD[RIGHT_LEG][3]))
		{
			state[i+1] = state[i] = 0;
			error_count++;
		}
		else
		{
			joint_rad[i+1][3] = joint_rad[i][3];
			// joint_rad[3][3] = joint_rad[2][3] = joint_rad[1][3] = joint_rad[0][3];
			// joint_rad[7][3] = joint_rad[6][3] = joint_rad[5][3] = joint_rad[4][3];
		}
	}
	if (error_count == MAX_SOL/2) 	return 0;

	//********************************************************
	// Joint 3,(?deg ~ ?deg, ?rad ~ ?rad)
	// Theta 3
	//********************************************************
	// Case 1
	for (i=0;i<MAX_SOL;i=i+2)
	{
		if (state[i] != 0)
		{
			s4 = sin(joint_rad[i][3]);
			c4 = cos(joint_rad[i][3]);

			th3_a = -DH_a3*s4;
			th3_b = -DH_a3*c4-DH_a3;
			th3_c = y_pos;

			th3_abc = th3_a*th3_a + th3_b*th3_b - th3_c*th3_c;

			if (th3_abc < 0)
			{
				// 해 없음.
				printf("th3_abc1 is less then 0\r\n");
				state[i+1] = state[i] = 0;
			}
			else
			{
				// theta 3의 계산.
				joint_rad[i][2] = atan2(th3_b,th3_a)+atan2(sqrt(th3_abc),th3_c);
				joint_rad[i][2] = range_rad(joint_rad[i][2]);

				// theta 3의 계산.
				joint_rad[i+1][2] = atan2(th3_b,th3_a)-atan2(sqrt(th3_abc),th3_c);
				joint_rad[i+1][2] = range_rad(joint_rad[i+1][2]);
			}
		}
	}

	// ?도 범위를 넘어서면 solution 없음.
	error_count = 0;
	for(i=0;i<MAX_SOL;i++)
	{
		if ((joint_rad[i][2] > LEG_MAX_RAD[RIGHT_LEG][2]) || (joint_rad[i][2] < LEG_MIN_RAD[RIGHT_LEG][2]))
		{
			state[i] = 0;
			error_count++;
		}
	}
	if (error_count == MAX_SOL) 	return 0;


	//********************************************************
	// Joint 2,(?deg ~ ?deg, ?rad ~ ?rad)
	// Theta 2
	//********************************************************
	// Case 1 ~ MAX_SOL
	for (i=0;i<MAX_SOL;i++)
	{
		if (state[i] != 0)
		{
			c34 = cos(joint_rad[i][3]+joint_rad[i][2]);
			c3 = cos(joint_rad[i][2]);

			th2_c = x_pos / (DH_a3*c34 + DH_a3*c3);
			th2_s =-z_pos / (DH_a3*c34 + DH_a3*c3);

			// theta 2의 계산.
			joint_rad[i][1] = atan2(th2_s,th2_c)+PI/2;
			joint_rad[i][1] = range_rad(joint_rad[i][1]);
		}
	}

	// ?도 범위를 넘어서면 solution 없음.
	error_count = 0;
	for (i=0;i<MAX_SOL;i++)
	{
		if ((joint_rad[i][1] > LEG_MAX_RAD[RIGHT_LEG][1]) || (joint_rad[i][1] < LEG_MIN_RAD[RIGHT_LEG][1]))
		{
			state[i] = 0;
			error_count++;
		}
	}
	if (error_count == MAX_SOL) 	return 0;

	//********************************************************
	// Joint 5, (?deg ~ ?deg, ?rad ~ ?rad)
	//
	//********************************************************
	for(i=0;i<MAX_SOL;i++)
	{
		// 구속조건에 의한 Theta 5
		joint_rad[i][4] = -(joint_rad[i][2]+joint_rad[i][3]);
		// -pi에서 +pi사이의 값으로 변환.
		joint_rad[i][4] = range_rad(joint_rad[i][4]);
	}

	// ?도 범위를 넘어서면 solution 없음.
	error_count = 0;
	for (i=0;i<MAX_SOL;i++)
	{
		if ((joint_rad[i][4] > LEG_MAX_RAD[RIGHT_LEG][4]) || (joint_rad[i][4] < LEG_MIN_RAD[RIGHT_LEG][4]))
		{
			state[i] = 0;
			error_count++;
		}
	}
	if (error_count == MAX_SOL) 	return 0;

	//********************************************************
	// Joint 6, (?deg ~ ?deg, ?rad ~ ?rad)
	//
	//********************************************************
	for(i=0;i<MAX_SOL;i++)
	{
		// 구속조건에 의한 Theta 6
		joint_rad[i][5] = -joint_rad[i][1];
		// -pi에서 +pi사이의 값으로 변환.
		joint_rad[i][5] = range_rad(joint_rad[i][5]);
	}
	// theta 2에 의해 결정되며 theta 2의 범위가 theta 6과 동일하므로
	// 제한 범위가 넘어가지 않는다.

	//********************************************************
	// return result
	//
	//********************************************************
	k = 0;
	for (i = 0;i<MAX_SOL;i++)
	{
		if (state[i] == 1)
		{
			for (j = 0;j<6;j++)	jt_deg[i][j] = joint_rad[i][j] * RADTODEG;
			k++;
		}
		else
		{
			for (j = 0;j<6;j++)	jt_deg[i][j] = -999;
		}
	}
	return k;
}

// +pi에서 -pi사이의 값으로 변환.
double range_rad(double a)
{
	int i;
	for(i=0;i<10;i++)
	{
		if (a > PI)	a = a - 2.0*PI;
		else if(a < (-1*PI)) a = a + 2.0*PI;
		else 	return a;
	}
	printf("Cannot convert rad\r\n");
	return -999;
}

// 0도에서 360도사이의 값으로 변환.
double range_deg(double a)
{
	int i;
	for(i=0;i<10;i++)
	{
		if (a > PI_DEG)
		{
			a = a - 2.0*PI_DEG;
		}
		else if(a <= (-1*PI_DEG))
		{
			a = a + 2.0*PI_DEG;
		}
		else
		{
			if(a < 0)	{a = a + 2.0*PI_DEG;}	// 0 ~ 360
			return a;
		}
	}
	printf("Cannot convert degree\r\n");
	return -999;
}

// conversion from user specification into matrix
// input  : abg[6] : roll(z), pitch(y) and yaw(x) deg_angle, px, py, pz
// output : mtx[3][4]
void UtoiXyz(double abg[], double mtx[][4])
{
	double a_rdn, b_rdn, g_rdn;
	double sa,ca,sb,cb,sg,cg;

	a_rdn=abg[0]*DEGTORAD;
	b_rdn=abg[1]*DEGTORAD;
	g_rdn=abg[2]*DEGTORAD;

	sa=sin(a_rdn); ca=cos(a_rdn);
	sb=sin(b_rdn); cb=cos(b_rdn);
	sg=sin(g_rdn); cg=cos(g_rdn);

	mtx[0][0]=ca*cb; 			mtx[1][0]=sa*cb; 			mtx[2][0]=-sb; 		mtx[0][3]=abg[3];
	mtx[0][1]=ca*sb*sg-sa*cg;	mtx[1][1]=sa*sb*sg+ca*cg;	mtx[2][1]=cb*sg; 	mtx[1][3]=abg[4];
	mtx[0][2]=ca*sb*cg+sa*sg;	mtx[1][2]=sa*sb*cg-ca*sg;	mtx[2][2]=cb*cg;	mtx[2][3]=abg[5];
}

// conversion from matrix into roll, picth and yaw angle
// input  : mtx[3][4] : matrix
// output : abgxyz[6] : roll(z), picth(y), yaw(x) rad_angle, px, py, pz
void ItouXyz(double mtx[][4], double abgxyz[])
{
	double t1, beta, alpa, gamma, cb;

	t1 = sqrt(mtx[0][0]*mtx[0][0]+mtx[1][0]*mtx[1][0]);
	if (t1 > EPSI)
	{
		beta = atan2(-mtx[2][0],t1);
		cb = cos(beta);
		alpa = atan2(mtx[1][0]/cb, mtx[0][0]/cb);
		gamma = atan2(mtx[2][1]/cb, mtx[2][2]/cb);
	}
	else
	{
		if (-mtx[2][0] > 0)
		{
			beta=PI/2;
			alpa = 0;
			gamma = atan2(mtx[0][1],mtx[1][1]);
		}
		else
		{
			beta=-PI/2;
			alpa = 0;
			gamma = -atan2(mtx[0][1],mtx[1][1]);
		}
	}

	abgxyz[0]=alpa; abgxyz[1]=beta; abgxyz[2]=gamma;
	abgxyz[3]=mtx[0][3]; abgxyz[4]=mtx[1][3]; abgxyz[5]=mtx[2][3];
}

// Absolute Value of Double Variable
double Double_Abs(double x)
{
	return (x<0.0)? -x : x;
}

// AP = ABT*BP --> rotate BP to AP(1)
// BP = (APT)'*AP --> rotate AP to BP(0)
// ABT : describe the fradm{B} relative to the frame {A}
void RotateAB(double AP[6], double ABT[3][4], double BP[6], int inv)
{
	int i,j;

	if (inv == 1)
	// AP = ABT*BP --> rotate BP to AP(1)
	{
		for (i=0;i<6;i++)	AP[i] = 0;

		for (i=0;i<3;i++)
			for(j=0;j<3;j++)	AP[i] = AP[i] + ABT[i][j]*BP[j];

		for (i=0;i<3;i++)
			for(j=0;j<3;j++)	AP[i+3] = AP[i+3] + ABT[i][j]*BP[j+3];
	}
	else
	// BP = BAT*AP --> rotate AP to BP(0)
	{
		for (i=0;i<6;i++)	BP[i] = 0;

		for (i=0;i<3;i++)
			for(j=0;j<3;j++)	BP[i] = BP[i] + ABT[j][i]*AP[j];

		for (i=0;i<3;i++)
			for(j=0;j<3;j++)	BP[i+3] = BP[i+3] + ABT[j][i]*AP[j+3];
	}
}

// T1T2 = T1 * T2
// T[3][4] = 	mtx[0][0]	mtx[0][1]	mtx[0][2]	mtx[0][3]
// 				mtx[1][0]	mtx[1][1]	mtx[1][2]	mtx[1][3]
// 				mtx[2][0]	mtx[2][1]	mtx[2][2]	mtx[2][3]
void Mul_TT(const double T1[3][4],const double T2[3][4],double T1T2[3][4])
{
	// T1T2 = T1 * T2
	T1T2[0][0] = T1[0][0]*T2[0][0] + T1[0][1]*T2[1][0] + T1[0][2]*T2[2][0];
	T1T2[0][1] = T1[0][0]*T2[0][1] + T1[0][1]*T2[1][1] + T1[0][2]*T2[2][1];
	T1T2[0][2] = T1[0][0]*T2[0][2] + T1[0][1]*T2[1][2] + T1[0][2]*T2[2][2];
	T1T2[0][3] = T1[0][0]*T2[0][3] + T1[0][1]*T2[1][3] + T1[0][2]*T2[2][3] + T1[0][3];

	T1T2[1][0] = T1[1][0]*T2[0][0] + T1[1][1]*T2[1][0] + T1[1][2]*T2[2][0];
	T1T2[1][1] = T1[1][0]*T2[0][1] + T1[1][1]*T2[1][1] + T1[1][2]*T2[2][1];
	T1T2[1][2] = T1[1][0]*T2[0][2] + T1[1][1]*T2[1][2] + T1[1][2]*T2[2][2];
	T1T2[1][3] = T1[1][0]*T2[0][3] + T1[1][1]*T2[1][3] + T1[1][2]*T2[2][3] + T1[1][3];

	T1T2[2][0] = T1[2][0]*T2[0][0] + T1[2][1]*T2[1][0] + T1[2][2]*T2[2][0];
	T1T2[2][1] = T1[2][0]*T2[0][1] + T1[2][1]*T2[1][1] + T1[2][2]*T2[2][1];
	T1T2[2][2] = T1[2][0]*T2[0][2] + T1[2][1]*T2[1][2] + T1[2][2]*T2[2][2];
	T1T2[2][3] = T1[2][0]*T2[0][3] + T1[2][1]*T2[1][3] + T1[2][2]*T2[2][3] + T1[2][3];
}

// P02 = T01 * P12
void Mul_TP(const double T01[3][4],const double P12[3],double P02[3])
{
	P02[0] = T01[0][0]*P12[0] + T01[0][1]*P12[1] + T01[0][2]*P12[2] + T01[0][3];
	P02[1] = T01[1][0]*P12[0] + T01[1][1]*P12[1] + T01[1][2]*P12[2] + T01[1][3];
	P02[2] = T01[2][0]*P12[0] + T01[2][1]*P12[1] + T01[2][2]*P12[2] + T01[2][3];
}

// T10 = (inv)[T01]
void INV_T(const double T01[3][4],double T10[3][4])
{
	T10[0][0] = T01[0][0];
	T10[0][1] = T01[1][0];
	T10[0][2] = T01[2][0];
	T10[0][3] = -(T01[0][0]*T01[0][3] + T01[1][0]*T01[1][3] + T01[2][0]*T01[2][3]);

	T10[1][0] = T01[0][1];
	T10[1][1] = T01[1][1];
	T10[1][2] = T01[2][1];
	T10[1][3] = -(T01[0][1]*T01[0][3] + T01[1][1]*T01[1][3] + T01[2][1]*T01[2][3]);

	T10[2][0] = T01[0][2];
	T10[2][1] = T01[1][2];
	T10[2][2] = T01[2][2];
	T10[2][3] = -(T01[0][2]*T01[0][3] + T01[1][2]*T01[1][3] + T01[2][2]*T01[2][3]);
}

// Degree(30 ~ 330) -> Position(0 ~ 1023)
int D2P(double Degree)
{
	if ((Degree < 30) || (Degree > 330)) 	return 9999;
	return ((Degree-30) * CONV_D2P);
}

// Position(0 ~ 1023) -> Degree(30 ~ 330)
double P2D(int P)
{
	if ((P<0) || (P>1023))	return 9999;
	return (P * CONV_P2D + 300);
}

// [입력]theta -> 좌표계상의 회전각
// [출력]position_ax -> AX-12의 위치
// [인자]LEG -> 좌(1), 우측(0) 다리를 결정.
void LEG_D2P(double theta[6], int position_ax[6], int LEG)
{
	int i;
	double theta_ax[6];		// degree

	/////////////////////////////////////
	// 각도를 AX-12의 위치 Count로 변환
	/////////////////////////////////////
	for (i=0;i<6;i++)
	{
		// 기준점 변환. theta가 0일때 theta_ax는 ???
		theta_ax[i] = theta[i] + LEG_DtoP[LEG][i];
		// 회전 각도에 따른 처리
		theta_ax[i] = LEG_BASE[LEG][i] + LEG_SIGN[LEG][i]*theta_ax[i];
		// 각도를 0 ~ 360도 사이로 변환.
		theta_ax[i] = range_deg(theta_ax[i]);
		// AX-12의 PulseCount로 변환
		position_ax[i] = (int)((theta_ax[i] - 30) * CONV_D2P);
	}
	return;
}

// [입력]theta -> 좌표계상의 회전각
// [출력]position_ax -> AX-12의 위치
// [인자]LEG -> 좌(1), 우측(0) 팔을 결정.
// void ARM_D2P(double waist, double theta[4], int position_ax[4], int ARM)
void ARM_D2P(double theta[4], int position_ax[4], int ARM)
{
	int i;
	double theta_ax[4];		// degree

	/////////////////////////////////////
	// 각도를 AX-12의 위치 Count로 변환
	/////////////////////////////////////
	for (i=0;i<4;i++)
	{
		// 기준점 변환. theta가 0일때 theta_ax는 ???
		theta_ax[i] = theta[i] + ARM_DtoP[ARM][i];
		// 회전 각도에 따른 처리
		theta_ax[i] = ARM_BASE[ARM][i] + ARM_SIGN[ARM][i]*theta_ax[i];
		// 각도를 0 ~ 360도 사이로 변환.
		theta_ax[i] = range_deg(theta_ax[i]);
		// AX-12의 PulseCount로 변환
		position_ax[i] = (int)((theta_ax[i] - 30) * CONV_D2P);
	}
	return;
}

// [입력]position_ax -> AX-12의 위치
// [출력]theta -> 좌표계상의 회전각
// [인자]LEG -> 좌(1), 우측(0) 다리를 결정.
void LEG_P2D(int position_ax[6], double theta[6], int LEG)
{
	int i;
	double theta_ax[6];		// degree

	/////////////////////////////////////
	// AX-12의 위치 Count를 각도로 변환
	/////////////////////////////////////
	for (i=0;i<6;i++)
	{
		// theta_ax로 변환
		theta_ax[i] = position_ax[i] * CONV_P2D + 30;
		// 회전 각도에 따른 처리
		theta_ax[i] = LEG_BASE[LEG][i] + LEG_SIGN[LEG][i]*theta_ax[i];
		
		// 190718, range_deg branch
		// position(0~1023)을 각도(0~360)으로 변경하는데
		// 지금은 0~360으로 변환 후 기준점을 변환시켜 음수가 나올때가 있다.
		// 출력이 0~360으로 나오도록 순서를 바꿨다. 
		// -180에서 180으로 바꾸려면, range_deg를 수정하자.
		
		// 기준점 변환
		theta_ax[i] = theta_ax[i] - LEG_DtoP[LEG][i];

		// 각도를 0 ~ 360도 사이로 변환.
		theta[i] = range_deg(theta_ax[i]);
	}
	return;
}


// [입력]position_ax -> AX-12의 위치
// [출력]theta -> 좌표계상의 회전각
// [인자]LEG -> 좌(1), 우측(0) 팔을 결정.
void ARM_P2D(int position_ax[4], double theta[4], int ARM)
{
	int i;
	double theta_ax[4];		// degree

	/////////////////////////////////////
	// AX-12의 위치 Count를 각도로 변환
	/////////////////////////////////////
	for (i=0;i<4;i++)
	{
		// theta_ax로 변환
		theta_ax[i] = position_ax[i] * CONV_P2D + 30;
		// 회전 각도에 따른 처리
		theta_ax[i] = ARM_BASE[ARM][i] + ARM_SIGN[ARM][i]*theta_ax[i];
		
		// 190718, range_deg branch
		// position(0~1023)을 각도(0~360)으로 변경하는데
		// 지금은 0~360으로 변환 후 기준점을 변환시켜 음수가 나올때가 있다.
		// 출력이 0~360으로 나오도록 순서를 바꿨다. 
		// -180에서 180으로 바꾸려면, range_deg를 수정하자.
		
		// 기준점 변환
		theta_ax[i] = theta_ax[i] - ARM_DtoP[ARM][i];
		
		// 각도를 0 ~ 360도 사이로 변환.
		theta[i] = range_deg(theta_ax[i]);
	}
	return;
}

// 현재 위치에서 다음 위치로의 이동 속도 를 계산
// 다음 위치를 현재 위치로 저장.
void Cal_LEG_Speed(int position_ax[2][6],int speed_ax[2][6], double SPEED_CONV)
{
	int i,k;
	int speed_temp;

	// k = RIGHT_LEG(0), LEFT_LEG(1)
	for (k=0;k<2;k++)
	{
		for (i=0;i<6;i++)
		{
			// 이동 속도 계산
			speed_temp = abs(g_LEG_Position[k][i] - position_ax[k][i]) * SPEED_CONV;
			speed_temp = max(speed_temp,SPEED_MIN);
			if (speed_temp > SPEED_MAX)
			{
				printf("The AX-12 speed is exceed 1023(Max_Speed)\r\n");
			}
			speed_ax[k][i] = min(speed_temp,SPEED_MAX);

			// 현재 위치를 저장.
			g_LEG_Position[k][i] = position_ax[k][i];
		}
	}
}

// 현재 위치에서 다음 위치로의 이동 속도 를 계산
// 다음 위치를 현재 위치로 저장.
void Cal_ARM_Speed(int position_ax[2][4],int speed_ax[2][4], double SPEED_CONV)
{
	int i,k;
	int speed_temp;

	// k = RIGHT_ARM(0), LEFT_ARM(1)
	for (k=0;k<2;k++)
	{
		for (i=0;i<4;i++)
		{
			// 이동 속도 계산
			// speed_temp = abs(g_LEG_Position[k][i] - position_ax[k][i]) * SPEED_CONV;
			speed_temp = abs(g_ARM_Position[k][i] - position_ax[k][i]) * SPEED_CONV;
			speed_temp = max(speed_temp,SPEED_MIN);
			if (speed_temp > SPEED_MAX)
			{
				printf("The AX-12 speed is exceed 1023(Max_Speed)\r\n");
			}
			speed_ax[k][i] = min(speed_temp,SPEED_MAX);

			// 현재 위치를 저장.
			g_ARM_Position[k][i] = position_ax[k][i];
		}
	}
}

// [입력]TCF -> {BC}{F}T
// [출력]T06 -> {0}{6}T
// [인자]LEG -> 좌, 우측 다리를 결정.
void Cal_LEG_BASE(double TCF[3][4], double T06[3][4], int LEG)
{
	double mtx[3][4];

	if (LEG == RIGHT_LEG)	{Mul_TT(T_R0_C,TCF,mtx);}
	else					{Mul_TT(T_L0_C,TCF,mtx);}
	Mul_TT(mtx,T_F_R6,T06);
}

// ******************************************************
// BASE FOOT를 기준으로 한 Torque와 Pcog를 계산
// 하체와 상체를 모두 고려하여 역진자 원리에 의해 계산
// ******************************************************
// [입력] BASE_FOOT : TORQUE를 계산하기 위한 기준 발.
//		  arm_theta : 허리와 팔의 각도
//        theta 	: 다리의 현재 위치(각도)
// [출력] Torque 	: 각 방향의 Torque의 크기. 단위는 [kg.m]임.
//        Pcog   	: BASE_FOOT 자료를 기준으로 한 무게 중심의 위치
// ******************************************************
void Cal_Torque_ALL(double arm_theta[2][4], const double theta[2][6],
			        double Torque[3], double Pcog[3], const int BASE_FOOT)
{
	int i;
	int foot1, foot2;
	double mtx1[3][4], mtx2[3][4], T_RF[ALL_TORQUE][3][4], P_RF[ALL_TORQUE][3];
	double ALL_WEIGHT_SUM;

	// ******************************
	// 다리
	// ******************************
	// {BASE_FOOT}을 기준으로 한 각 Link의 무게중심 위치 계산
	if (BASE_FOOT == RIGHT_LEG)
	{
		foot1 = RIGHT_LEG;
		foot2 = LEFT_LEG;
	}
	else
	{
		foot1 = LEFT_LEG;
		foot2 = RIGHT_LEG;
	}
	
	////////////////////////////////////////////////////////////////////////////
	// 변수설명: T_기준좌표_목표좌표, P_기준좌표_목표좌표 
	// MAR = Mess_Arm_Right
	// C = Center
	// BC = Body Center
	// RF = Right Foot
	////////////////////////////////////////////////////////////////////////////
	
	// P_RF_R6(M6): RF를 기준으로 한 M6(오른쪽 6번의 무게중심)의 좌표
	/* <<오류예감>> ******************************
	* Fto6(F를 기준으로 한 6번)의 위치는 고정이다.
	* 그런데, 무게중심은 {5}(5번 좌표계)를 기준으로 측정했다.(그럴꺼다)
	* 따라서, 무게중심을 계산할 때 T_RF_R5로 변환을 해야 한다.
    * (단, 크게 차이는 나지 않을것같다.)
    * 나중에 이걸로 {5}번? 뭉치의 무게중심을 계산할 것 같다.
    * 왼쪽도 동일 문제 있다.
    **********************************************/   
	INV_T(T_R6_F, T_RF[R6]); 				// T_RF_R6 = (INV)T_R6_RF
	Mul_TP(T_RF[R6],P_6_M6,P_RF[R6]);		// P_RF_R6(M6) = T_RF_R6 x P_6_M6

	// P_RF_R4(M4) : RF를 기준으로 한 M4(오른쪽 4번의 무게중심)의 좌표
	T64_for_M2(theta[foot1], mtx1);			// 관절각을 이용해서 T_{6}_{4} 계산
	Mul_TT(T_RF[R6],mtx1,T_RF[R4]);			// T_RF_R4 = T_RF_R6 x T_R6_R4
	Mul_TP(T_RF[R4],P_4_M4,P_RF[R4]);		// P_RF_R4(M4) = T_RF_R4 x P_4_M4

	// P_RF_R3(M3) : RF를 기준으로 한 M3(오른쪽 3번의 무게중심)의 좌표
	T43_for_M2(theta[foot1], mtx1);			// 관절각을 이용해서 T_{4}_{3} 계산
	Mul_TT(T_RF[R4],mtx1,T_RF[R3]);			// T_RF_R3 = T_RF_R4 x T_R4_R3
	Mul_TP(T_RF[R3],P_3_M3,P_RF[R3]);		// P_RF_R3(M3) = T_RF_R3 x P_3_M3

	// P_RF_BC(MBC) : RF를 기준으로 한 MBC(BC번의 무게중심)의 좌표
	TCF_for_M2(theta[foot1], mtx1, foot1);	// 관절각을 이용해서 T_BC_RF
	INV_T(mtx1, T_RF[BC]);					// T_RF_BC = (INV)T_BC_RF
	Mul_TP(T_RF[BC],P_C_MC,P_RF[BC]);		// P_RF_BC(MBC) = T_RF_BC x P_BC_MBC

	// P_RF_L3(M3)	: RF를 기준으로 한 M3(왼쪽 3번의 무게중심)의 좌표
	T03_for_M2(theta[foot2], mtx1);			// 관절각을 이용해서 T_{0}_{3} 계산
	Mul_TT(T_C_0[foot2],mtx1,mtx2);			// T_BC_L3 = T_BC_L0 x T_L0_L3
	Mul_TT(T_RF[BC],mtx2,T_RF[L3]);			// T_RF_L3 = T_RF_BC x T_BC_L3
	Mul_TP(T_RF[L3],P_3_M3,P_RF[L3]);		// P_RF_L3 = T_RF_L3 x P_3_M3

	// P_RF_L4(M4)	: RF를 기준으로 한 M4(왼쪽 4번의 무게중심)의 좌표
	T43_for_M2(theta[foot2], mtx1);			// 관절각을 이용해서 T_{4}_{3} 계산
	INV_T(mtx1, mtx2);						// T_L3_L4 = (INV)T_L4_L3
	Mul_TT(T_RF[L3], mtx2, T_RF[L4]);		// T_RF_L4 = T_RF_L3 x T_L3_L4
	Mul_TP(T_RF[L4],P_4_M4,P_RF[L4]);		// P_RF_L4 = T_RF_L4 x P_4_M4

	// P_RF_L6(M6)	: RF를 기준으로 한 M6(왼쪽 6번의 무게중심)의 좌표
	T64_for_M2(theta[foot2], mtx1);			// 관절각을 이용해서 T_{6}_{4} 계산
	INV_T(mtx1, mtx2);						// T_L4_L6 = (INV)T_L6_L4
	Mul_TT(T_RF[L4], mtx2, T_RF[L6]);		// T_RF_L6 = T_RF_L4 x T_L4_L6
	Mul_TP(T_RF[L6],P_6_M6,P_RF[L6]);		// P_RF_L6 = T_RF_L6 x P_6_M6

	// ***************************
	// 몸통
	// ***************************
	/* <<오류예감>> ******************************
	* 웬지 풀이랑 다른 것 같다. 왜지?
	* 상체는 한번도 검증한 적이 없다.
	* 좌우 팔 바뀐것도 해결해야 함.
	**********************************************/  
	// P_RF_AR1(MA1)	: RF를 기준으로 한 MA1(상체 몸통의 무게중심)의 좌표
	T01_for_M2_BODY(arm_theta[RIGHT], mtx1);// 관절각을 이용해서 T_{A0}_{A1} 계산 
	Mul_TT(T_BC_R0, mtx1, mtx2);			// T_BC_AR1 = T_BC_AR0 x T_AR0_AR1
	Mul_TT(T_RF[BC], mtx2, T_RF[AR1]);		// T_RF_AR1 = T_RF_BC  x T_BC_AR1
	Mul_TP(T_RF[AR1],P_A1_MA1,P_RF[AR1]);	// P_RF_AR1 = T_RF_AR1 x P_A1_MA1
	
	// ***************************
	// 팔
	// ***************************
	// P_RF_AR3(MAR3)	: RF를 기준으로 한 MAR3(우측 팔 3번의 무게중심)의 좌표
	T03_for_M2_BODY(arm_theta[RIGHT], mtx1);// 관절각을 이용해서 T_{A0}_{AR3} 계산
	Mul_TT(T_BC_R0, mtx1, mtx2);			// T_BC_AR3 = T_BC_AR0 x T_AR0_AR3
	Mul_TT(T_RF[BC], mtx2, T_RF[AR3]);		// T_RF_AR3 = T_RF_BC  x T_BC_AR3
	Mul_TP(T_RF[AR3],P_A3_MA3,P_RF[AR3]);	// P_RF_AR3 = T_RF_AR3 x P_A3_MA3
	
	// P_RF_AR4(MAR4)	: RF를 기준으로 한 MAR4(우측 팔 4번의 무게중심)의 좌표
	T04_for_M2_BODY(arm_theta[RIGHT], mtx1);// 관절각을 이용해서 T_{A0}_{AR4} 계산
	Mul_TT(T_BC_R0, mtx1, mtx2);			// T_BC_AR4 = T_BC_AR0 x T_AR0_AR4
	Mul_TT(T_RF[BC], mtx2, T_RF[AR4]);		// T_RF_AR4 = T_RF_BC  x T_BC_AR4
	Mul_TP(T_RF[AR4],P_A4_MA4,P_RF[AR4]);	// P_RF_AR4 = T_RF_AR4 x P_A4_MA4

	// P_RF_AL3(MAL3)	: RF를 기준으로 한 MAL3(좌측 팔 3번의 무게중심)의 좌표
	T03_for_M2_BODY(arm_theta[LEFT], mtx1);	// 관절각을 이용해서 T_{A0}_{AL3} 계산
	Mul_TT(T_BC_L0, mtx1, mtx2);			// T_BC_AL3 = T_BC_AL0 x T_AL0_AL3
	Mul_TT(T_RF[BC], mtx2, T_RF[AL3]);		// T_RF_AL3 = T_RF_BC  x T_BC_AL3
	Mul_TP(T_RF[AL3],P_A3_MA3,P_RF[AL3]);	// P_RF_AL3 = T_RF_AL3 x P_A3_MA3
	
	// P_RF_AL4(MAL4)	: RF를 기준으로 한 MAL4(좌측 팔 4번의 무게중심)의 좌표
	T04_for_M2_BODY(arm_theta[LEFT], mtx1);	// 관절각을 이용해서 T_{A0}_{AL4} 계산
	Mul_TT(T_BC_L0, mtx1, mtx2);			// T_BC_AL4 = T_BC_AL0 x T_AL0_AL4
	Mul_TT(T_RF[BC], mtx2, T_RF[AL4]);		// T_RF_AL4 = T_RF_BC  x T_BC_AL4
	Mul_TP(T_RF[AL4],P_A4_MA4,P_RF[AL4]);	// P_RF_AL3 = T_RF_AL4 x P_A4_MA4

	///////////////////////////////
	// 토크 계산(BASE FOOT 좌표계 기준)
	///////////////////////////////
	Torque[X] = Torque[Y] = Torque[Z] = 0;
	for (i=0;i<ALL_TORQUE;i++)
	{
		Torque[X] += P_RF[i][X] * ALL_WEIGHT[i];
		Torque[Y] += P_RF[i][Y] * ALL_WEIGHT[i];
		Torque[Z] += P_RF[i][Z] * ALL_WEIGHT[i];
	}
	Torque[X] = Torque[X]*G_Acc / 1000000L;
	Torque[Y] = Torque[Y]*G_Acc / 1000000L;
	Torque[Z] = Torque[Z]*G_Acc / 1000000L;

	//////////////////////////////
	// 무게중심 계산(BASE FOOT 좌표계 기준)
	//////////////////////////////
	ALL_WEIGHT_SUM = 0;
	Pcog[X] = Pcog[Y] = Pcog[Z] = 0;

	for (i=0;i<ALL_TORQUE;i++)	{ALL_WEIGHT_SUM += ALL_WEIGHT[i];}

	for (i=0;i<ALL_TORQUE;i++)
	{
		Pcog[X] += P_RF[i][X] * ALL_WEIGHT[i];
		Pcog[Y] += P_RF[i][Y] * ALL_WEIGHT[i];
		Pcog[Z] += P_RF[i][Z] * ALL_WEIGHT[i];
	}

	Pcog[X] = Pcog[X] / ALL_WEIGHT_SUM;
	Pcog[Y] = Pcog[Y] / ALL_WEIGHT_SUM;
	Pcog[Z] = Pcog[Z] / ALL_WEIGHT_SUM;
}

//*********************
// 다리
//*********************
// [입력]theta : 다리의 현재 위치(각도)
//       BASE_FOOT : TORQUE를 계산하기 위한 기준 발.
// [출력]Torque : 각 방향의 Torque의 크기. 단위는 [kg.m]임.
//       Pcog   : 무게 중심의 위치
// 역진자 원리에 의한 Torque 계산.
void Cal_Torque_LEG(const double theta[2][6], double Torque[3], double Pcog[3], const int BASE_FOOT)
{
	int i;
	int foot1, foot2;
	double mtx1[3][4], mtx2[3][4], T_RF[LEG_TORQUE][3][4], P_RF[LEG_TORQUE][3];
	double LEG_WEIGHT_SUM;

	// {BASE_FOOT}을 기준으로 한 각 Link의 무게중심 위치 계산
	if (BASE_FOOT == RIGHT_LEG)
	{
		foot1 = RIGHT_LEG;
		foot2 = LEFT_LEG;
	}
	else
	{
		foot1 = LEFT_LEG;
		foot2 = RIGHT_LEG;
	}

	// T_RF_R6 = (INV)T_R6_RF
	// T_LF_L6 = (INV)T_L6_LF
	INV_T(T_R6_F, T_RF[R6]);
	Mul_TP(T_RF[R6],P_6_M6,P_RF[R6]);		// {3}

	// T_RF_R4 = T_RF_R6 * T_R6_R4
	// T_LF_L4 = T_LF_L6 * T_L6_L4
	T64_for_M2(theta[foot1], mtx1);
	Mul_TT(T_RF[R6],mtx1,T_RF[R4]);
	Mul_TP(T_RF[R4],P_4_M4,P_RF[R4]);		// {3}

	// T_RF_R3 = T_RF_R4 * T_R4_R3
	// T_LF_L3 = T_LF_L4 * T_L4_L3
	T43_for_M2(theta[foot1], mtx1);
	Mul_TT(T_RF[R4],mtx1,T_RF[R3]);
	Mul_TP(T_RF[R3],P_3_M3,P_RF[R3]);		// {3}

	// T_RF_BC = (INV)T_BC_RF
	// T_LF_BC = (INV)T_BC_LF
	TCF_for_M2(theta[foot1], mtx1, foot1);
	INV_T(mtx1, T_RF[BC]);
	Mul_TP(T_RF[BC],P_C_MC,P_RF[BC]);		// {3}

	// T_RF_L3 = T_RF_BC * T_BC_L3
	// T_LF_R3 = T_LF_BC * T_BC_R3
	T03_for_M2(theta[foot2], mtx1);
	Mul_TT(T_C_0[foot2],mtx1,mtx2);
	Mul_TT(T_RF[BC],mtx2,T_RF[L3]);
	Mul_TP(T_RF[L3],P_3_M3,P_RF[L3]);		// {3}

	// T_RF_L4 = T_RF_L3 * T_L3_L4
	// T_LF_R4 = T_RF_R3 * T_R3_R4
	T43_for_M2(theta[foot2], mtx1);
	INV_T(mtx1, mtx2);
	Mul_TT(T_RF[L3], mtx2, T_RF[L4]);
	Mul_TP(T_RF[L4],P_4_M4,P_RF[L4]);		// {3}

	// T_RF_L6 = T_RF_L4 * T_L4_L6
	T64_for_M2(theta[foot2], mtx1);
	INV_T(mtx1, mtx2);
	Mul_TT(T_RF[L4], mtx2, T_RF[L6]);
	Mul_TP(T_RF[L6],P_6_M6,P_RF[L6]);		// {3}

	///////////////////////////////
	// 토크의 계산.
	///////////////////////////////
	Torque[X] = Torque[Y] = Torque[Z] = 0;
	for (i=0;i<LEG_TORQUE;i++)
	{
		/*
		Torque[X] += T_RF[i][X][POS] * LEG_WEIGHT[i];
		Torque[Y] += T_RF[i][Y][POS] * LEG_WEIGHT[i];
		Torque[Z] += T_RF[i][Z][POS] * LEG_WEIGHT[i];
		*/
		Torque[X] += P_RF[i][X] * LEG_WEIGHT[i];
		Torque[Y] += P_RF[i][Y] * LEG_WEIGHT[i];
		Torque[Z] += P_RF[i][Z] * LEG_WEIGHT[i];
	}
	Torque[X] = Torque[X]*G_Acc / 1000000L;
	Torque[Y] = Torque[Y]*G_Acc / 1000000L;
	Torque[Z] = Torque[Z]*G_Acc / 1000000L;


	//////////////////////////////
	// 무게중심 계산
	//////////////////////////////
	LEG_WEIGHT_SUM = 0;
	Pcog[X] = Pcog[Y] = Pcog[Z] = 0;

	for (i=0;i<LEG_TORQUE;i++)	{LEG_WEIGHT_SUM += LEG_WEIGHT[i];}

	for (i=0;i<LEG_TORQUE;i++)
	{
		Pcog[X] += P_RF[i][X] * LEG_WEIGHT[i];
		Pcog[Y] += P_RF[i][Y] * LEG_WEIGHT[i];
		Pcog[Z] += P_RF[i][Z] * LEG_WEIGHT[i];
	}

	Pcog[X] = Pcog[X] / LEG_WEIGHT_SUM;
	Pcog[Y] = Pcog[Y] / LEG_WEIGHT_SUM;
	Pcog[Z] = Pcog[Z] / LEG_WEIGHT_SUM;
}

//*********************
// 상체
//*********************
// [입력]theta  : 상체의 현재 위치(각도)
//                arm_theta[RIGHT][0] = arm_theta[LEFT][0]이어야 함.(허리)
// [출력]Torque : 각 방향의 Torque의 크기. 단위는 [kg.m]임.
//       Pcog   : 무게 중심의 위치(BC를 기준으로 함)
// 역진자 원리에 의한 Torque 계산.
void Cal_Torque_BODY(const double arm_theta[2][4], double Torque[3], double Pcog[3])
{
	int i;
	double mtx1[3][4], T_BC[BODY_TORQUE][3][4], P_BC[BODY_TORQUE][3];
	double BODY_WEIGHT_SUM;

	// ***************************
	// 몸통
	// ***************************
	// TB0 x T01 x P1M1
	T01_for_M2_BODY(arm_theta[RIGHT], mtx1);
	Mul_TT(T_BC_R0, mtx1, T_BC[BR1]);
	Mul_TP(T_BC[BR1],P_A1_MA1,P_BC[BR1]);

	// ***************************
	// 팔
	// ***************************
	T03_for_M2_BODY(arm_theta[RIGHT], mtx1);
	Mul_TT(T_BC_R0, mtx1, T_BC[BR3]);
	Mul_TP(T_BC[BR3],P_A3_MA3,P_BC[BR3]);

	T04_for_M2_BODY(arm_theta[RIGHT], mtx1);
	Mul_TT(T_BC_R0, mtx1, T_BC[BR4]);
	Mul_TP(T_BC[BR4],P_A4_MA4,P_BC[BR4]);

	T03_for_M2_BODY(arm_theta[LEFT], mtx1);
	Mul_TT(T_BC_L0, mtx1, T_BC[BL3]);
	Mul_TP(T_BC[BL3],P_A3_MA3,P_BC[BL3]);

	T04_for_M2_BODY(arm_theta[LEFT], mtx1);
	Mul_TT(T_BC_L0, mtx1, T_BC[BL4]);
	Mul_TP(T_BC[BL4],P_A4_MA4,P_BC[BL4]);

	///////////////////////////////
	// 토크의 계산.
	///////////////////////////////
	Torque[X] = Torque[Y] = Torque[Z] = 0;
	for (i=0;i<BODY_TORQUE;i++)
	{
		Torque[X] += P_BC[i][X] * BODY_WEIGHT[i];
		Torque[Y] += P_BC[i][Y] * BODY_WEIGHT[i];
		Torque[Z] += P_BC[i][Z] * BODY_WEIGHT[i];
	}
	Torque[X] = Torque[X]*G_Acc / 1000000L;
	Torque[Y] = Torque[Y]*G_Acc / 1000000L;
	Torque[Z] = Torque[Z]*G_Acc / 1000000L;


	//////////////////////////////
	// 무게중심 계산
	//////////////////////////////
	BODY_WEIGHT_SUM = 0;
	Pcog[X] = Pcog[Y] = Pcog[Z] = 0;

	for (i=0;i<BODY_TORQUE;i++)	{BODY_WEIGHT_SUM += BODY_WEIGHT[i];}

	for (i=0;i<BODY_TORQUE;i++)
	{
		Pcog[X] += P_BC[i][X] * BODY_WEIGHT[i];
		Pcog[Y] += P_BC[i][Y] * BODY_WEIGHT[i];
		Pcog[Z] += P_BC[i][Z] * BODY_WEIGHT[i];
	}

	Pcog[X] = Pcog[X] / BODY_WEIGHT_SUM;
	Pcog[Y] = Pcog[Y] / BODY_WEIGHT_SUM;
	Pcog[Z] = Pcog[Z] / BODY_WEIGHT_SUM;
}

void Get_R_C_F(double mtx[3][4])
{
	int i;
	for (i=0;i<3;i++)
	{
		mtx[i][0] = R_C_F[i][0];
		mtx[i][1] = R_C_F[i][1];
		mtx[i][2] = R_C_F[i][2];
	}
}

// [입력]theta : 다리의 현재 위치(각도)
// [출력]position : 다리의 현재 위치 -> [RIGHT_LEG/LEFT_LEG][{0},{4},{6},{F}][x,y,z]
//                  다리의 현재 위치는 {BC}를 기준으로 한 위치임.
void Cal_LEG_Position(double theta[2][6], double position[2][4][3])
{
	int i, j;
	double mtx1[3][4], mtx2[3][4];

	// {BC}를 기준으로 한 각 Link의 위치 계산
	// 오른발, 왼발
	for (i=0;i<2;i++)
	{
		// {0}의 위치
		for(j=0;j<3;j++)	{ position[i][P_BC_0][j] = T_C_0[i][j][3]; }

		// {4}의 위치
		T04_for_M2(theta[i],mtx1);			// mtx1 = T04
		Mul_TT(T_C_0[i], mtx1, mtx2);		// mtx2 = TC0*T04 = TC4
		for(j=0;j<3;j++)	{ position[i][P_BC_4][j] = mtx2[j][3]; }

		// {6}의 위치
		T06_for_M2(theta[i],mtx1);			// mtx1 = T06
		Mul_TT(T_C_0[i], mtx1, mtx2);		// mtx2 = TC0*T06 = TC6
		for(j=0;j<3;j++)	{ position[i][P_BC_6][j] = mtx2[j][3]; }

		// {F}의 위치
		// 참고 : T_L6_F = T_R6_F
		Mul_TT(mtx2,T_R6_F, mtx1);		// mtx1 = TC6*T6F = TCF
		for(j=0;j<3;j++)	{ position[i][P_BC_F][j] = mtx1[j][3]; }
	}
}

// 해 중에서 최적의 해를 찾음. 가장 적게 이동하는 거리.
void BestSol_M(int state[4], double jt_resol[4][6], double jt_curr[6], double best_sol[6])
{
	int j,best;
	double sofar_best;
	double deviation,dev_t1,dev_t2,dev_t3,dev_t4,dev_t5,dev_t6;

	best = 0; sofar_best = LARGE;

	for (j=0; j<4; j++)
	{
		if (state[j])
		{
			dev_t1 = jt_resol[j][0] - jt_curr[0];
            dev_t2 = jt_resol[j][1] - jt_curr[1];
			dev_t3 = jt_resol[j][2] - jt_curr[2];
			dev_t4 = jt_resol[j][3] - jt_curr[3];
			dev_t5 = jt_resol[j][4] - jt_curr[4];
			dev_t6 = jt_resol[j][5] - jt_curr[5];

			deviation = dev_t1*dev_t1+ dev_t2*dev_t2
					+dev_t3*dev_t3+dev_t4*dev_t4
					+dev_t5*dev_t5+dev_t6*dev_t6;
			if (deviation < sofar_best)
			{
				best = j;
				sofar_best = deviation;
			}
		}
	}

	/* compute delta theta */
	for (j=0; j<6; j++)
		best_sol[j] = jt_resol[best][j];
}
