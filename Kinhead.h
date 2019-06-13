#include <stdio.h>
#include <math.h>
// #include <dos.h>
// #include <conio.h>
#include <stdlib.h>
#include <string.h>

//***************************************************************************
// Kin.cpp
//***************************************************************************

// Transform Matrix, joint 각도(degree, result), 해의 여부
// return값은 해의 갯수임.
int Inverse_Kin_M2(const double mtx[3][4],double jt_deg[MAX_SOL][6],int state[MAX_SOL]);

// a radian을 +pi에서 -pi사이의 값으로 변환함.
double range_rad(double a);
// a degree를 0에서 360사이의 값으로 변환함.
double range_deg(double a);

// user expression : (theta_z, theta_y, theta_x, x_pos, y_pos, z_pos)
// XYZ Fixed angles
void UtoiXyz(double abgxyz[6], double mtx[3][4]);	// degree, Transform matrix
void ItouXyz(double mtx[3][4], double abgxyz[6]);	// Transform Matrix, radian

// 절대값을 return.
double Double_Abs(double x);

// AP = ABT*BP --> rotate BP to AP(1)
// BP = (ABT)'*AP --> rotate AP to BP(0)
// ABT : describe the fradm{B} relative to the frame {A}
void RotateAB(double AP[3], double ABT[3][4], double BP[3], int inv);

// 051026 추가.
void Mul_TT(const double T1[3][4],const double T2[3][4],double T1T2[3][4]);
void Mul_TP(const double T01[3][4],const double P12[3],double P02[3]);
void INV_T(const double T01[3][4],double T10[3][4]);

// 060104 추가
// Degree(30 ~ 330) -> Position(0 ~ 1023)
int D2P(double Degree);

// Position(0 ~ 1023) -> Degree(30 ~ 330)
double P2D(int P);

// [입력]theta -> 좌표계상의 회전각
// [출력]position_ax -> AX-12의 위치
// [인자]LEG -> 좌, 우측 다리를 결정.
void LEG_D2P(double theta[6], int position_ax[6], int LEG);
void LEG_P2D(int position_ax[6], double theta[6], int LEG);

// void ARM_D2P(double waist, double theta[4], int position_ax[4], int ARM);
void ARM_D2P(double theta[4], int position_ax[4], int ARM);
void ARM_P2D(int position_ax[4], double theta[4], int ARM);

// [입력] position_ax -> AX-12의 이동 위치
// [전역] g_LEG_Positon -> AX-12의 현재 위치.
// [출력] speed_ax -> AX-12의 이동 속도
void Cal_LEG_Speed(int position_ax[2][6],int speed_ax[2][6], double SPEED_CONV);
void Cal_ARM_Speed(int position_ax[2][4],int speed_ax[2][4], double SPEED_CONV);

void Cal_LEG_BASE(double TCF[3][4], double T06[3][4], int LEG);

void Cal_Torque_LEG(const double theta[2][6], double Torque[3], double Pcog[3], const int BASE_FOOT);
void Cal_Torque_BODY(const double arm_theta[2][4], double Torque[3], double Pcog[3]);
void Cal_Torque_ALL(double arm_theta[2][4], const double theta[2][6],
			        double Torque[3], double Pcog[3], const int BASE_FOOT);

void Get_R_C_F(double mtx[3][4]);

// joint 각도(degree), Transform Matrix(result)
// void T02_for_M2(const double jt_deg[6], double mtx[3][4]);
void T03_for_M2(const double jt_deg[6], double mtx[3][4]);
void T04_for_M2(const double jt_deg[6], double mtx[3][4]);
// void T05_for_M2(const double jt_deg[6], double mtx[3][4]);
void T64_for_M2(const double jt_deg[6], double mtx[3][4]);
void T43_for_M2(const double jt_deg[6], double mtx[3][4]);
void T06_for_M2(const double jt_deg[6], double mtx[3][4]);
void TCF_for_M2(const double jt_deg[6], double mtx[3][4], int LEG);
void Cal_LEG_Position(double theta[2][6], double position[2][4][3]);
void BestSol_M(int state[4], double jt_resol[4][6], double jt_curr[6], double best_sol[6]);

// 상체의 정기구학 계산.
void T01_for_M2_BODY(const double jt_deg[4], double mtx[3][4]);
void T02_for_M2_BODY(const double jt_deg[4], double mtx[3][4]);
void T03_for_M2_BODY(const double jt_deg[4], double mtx[3][4]);
void T04_for_M2_BODY(const double jt_deg[4], double mtx[3][4]);
void T05_for_M2_BODY(const double jt_deg[4], double mtx[3][4]);
