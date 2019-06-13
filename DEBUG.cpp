#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>			// uint8_t....
#include <string.h>
#include <math.h>
#include <termios.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <netinet/in.h>
#include <sys/socket.h>

#include "define.h"
#include "debug.h"
#include "timer.h"
#include "serial.h"

// #include "MPU6050_6Axis_MotionApps20.h"
// 선언 순서에 주의!!!
#include "I2Cdev.h"
#include "helper_3dmath.h"

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];         // [x, y, z]          gyro sensor measurements

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// FD for Serial & LAN
extern int serial_fd;	
extern int socket_fd;
extern struct sockaddr_in servAddr;
extern bool serverReady;
extern bool	start_visualize;

PI_THREAD(sensor_t1)
{
    char sendBuffer[BUFSIZE], Buff_1[1024];
    
    unsigned int sendLen;
    unsigned int sendingErrorCount = 0;
    unsigned int count = 0;
    
	// if programming failed, don't try to do anything
	while(1)
	{
	    // get current FIFO count
	    fifoCount = mpu.getFIFOCount();
	    if (fifoCount == 1024) 
	    {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();
	
	    // otherwise, check for DMP data ready interrupt (this should happen frequently)
	    } else if (fifoCount >= 42) 
	    {
	        // read a packet from FIFO
	        mpu.getFIFOBytes(fifoBuffer, packetSize);	// 42 byte만큼 읽기
	        
	        
	        mpu.dmpGetQuaternion(&q, fifoBuffer);
	    	mpu.dmpGetAccel(&aa, fifoBuffer);
	        mpu.dmpGetGyro(gyro, fifoBuffer);
	        
	        #ifdef OUTPUT_READABLE_QUATERNION
	            // display quaternion values in easy matrix form: w x y z
	            // printf("quat %7.2f %7.2f %7.2f %7.2f    \n", q.w,q.x,q.y,q.z);
	        #endif
	
	        #ifdef OUTPUT_READABLE_EULER
	            // display Euler angles in degrees
	            mpu.dmpGetEuler(euler, &q);
	            // printf("euler %7.2f %7.2f %7.2f    \n", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
	        #endif
	
	        #ifdef OUTPUT_READABLE_YAWPITCHROLL
	            // display Euler angles in degrees
	            mpu.dmpGetGravity(&gravity, &q);
	            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	            // printf("ypr  %7.2f %7.2f %7.2f    \n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
			#endif
	
	        #ifdef OUTPUT_READABLE_REALACCEL
	            // display real acceleration, adjusted to remove gravity
	            mpu.dmpGetGravity(&gravity, &q);
	            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	            // printf("areal %6d %6d %6d    \n", aaReal.x, aaReal.y, aaReal.z);
	        #endif
	
	        #ifdef OUTPUT_READABLE_WORLDACCEL
	            // display initial world-frame acceleration, adjusted to remove gravity
	            // and rotated based on known orientation from quaternion	            
	            mpu.dmpGetGravity(&gravity, &q);
	            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	            // printf("aworld %6d %6d %6d    \n", aaWorld.x, aaWorld.y, aaWorld.z);
	        #endif
	        count++;
	        
	        // CH 1,2,3
	        // 로봇의 현재 기울기를 Roll(X), Pitch(Y), Yaw(Z)로 표현
	        // Roll(Z)  : 좌우 기울기
	        // Pitch(Y) : 앞뒤 기울기
	        // Yaw(Z)   : 회전
	        sprintf(Buff_1, "%7.2f %7.2f %7.2f ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
	       	strcat(sendBuffer,Buff_1);
	       	
	       	// CH 4,5,6
	       	// 로봇의 직선 움직임의 순간 가속도를 표현 (대략... - 30000 ~ 30000)
	       	// X : 앞뒤 움직임
	       	// Y : 좌우 움직임
	       	// Z : 상하 움직임
	       	// 단, Gravity를 제거했다고 하는데, 로봇의 기울기에 따라 편차가 보인다.
	       	// aaReal은 로봇 좌표계, aaWorld는 고정 좌표계
	       	// 고정좌표계의 X,Y,Z는 로봇의 시작위치인 것 같으며, 편차는 Z에서만 보인다.
	        // sprintf(Buff_1, "%6d %6d %6d ", aaWorld.x, aaWorld.y, aaWorld.z);
	        sprintf(Buff_1, "%6d %6d %6d ", aaReal.x, aaReal.y, aaReal.z);
	        strcat(sendBuffer,Buff_1);
	        
	        // CH 7,8,9
	        // 로봇의 X,Y,Z축에 대한 회전 각속도를 표현(대략... -1024 ~ 1024)
	        // X : 좌우 기울기
	        // Y : 앞뒤 기울기
	        // Z : 회전
	       	sprintf(Buff_1, "%6d %6d %6d ", gyro[0], gyro[1], gyro[2]);
	        strcat(sendBuffer,Buff_1);
	    }

	    if (start_visualize == false)
	    {
	    	memset(sendBuffer,0,sizeof(sendBuffer));
	    	sprintf(sendBuffer,"value ");
	    	count = 0;
	    }
	    else if (count >= 100)
	    {
	    	// sockfd 소켓을 통해 servAddr을 주소로 갖는 서버에게 데이터를 보냄
	    	strcat(sendBuffer,"\n");
    		sendLen =  write(socket_fd, sendBuffer, strlen(sendBuffer));
			if (sendLen != strlen(sendBuffer))
			{	printf("Sending Error (%d)\n",++sendingErrorCount);	}
			memset(sendBuffer,0,sizeof(sendBuffer));
			sprintf(sendBuffer,"value ");
			count = 0;
		}
	}
}

// DOS에서 사용된, 그러나 LINUX에서는 안되는 함수를 여기 구현.
int kbhit(void)
{
  struct timeval tv;
  fd_set read_fd;
  
  tv.tv_sec=0;
  tv.tv_usec=0;
  FD_ZERO(&read_fd);
  FD_SET(0,&read_fd);
  
  if(select(1, &read_fd, NULL, NULL, &tv) == -1) return 0;
  if(FD_ISSET(0,&read_fd)) return 1;

  return 0;
}

int getch(void)
{
  int ch;
  struct termios buf;
  struct termios save;

   tcgetattr(0, &save);
   buf = save;
   buf.c_lflag &= ~(ICANON|ECHO);
   buf.c_cc[VMIN] = 1;
   buf.c_cc[VTIME] = 0;
   tcsetattr(0, TCSAFLUSH, &buf);
   ch = getchar();
   tcsetattr(0, TCSAFLUSH, &save);
   return ch;
}

int max(int a,int b)
{
  if (a>b) return a;
  else return b;
}

int min(int a, int b)
{
  if (a<b) return a;
  else return b;
}
// 여기까지, Dos 함수 구현  


void DBG_Print_T(char* Title, const double mtx[3][4])
{
	int i;

	if (Title != NULL)	printf("Title = %s \r\n",Title);

	for (i=0;i<3;i++)
		printf("   %9.4f, %9.4f, %9.4f, %9.4f\r\n",mtx[i][0], mtx[i][1], mtx[i][2],mtx[i][3]);
	getch();
}

void DBG_Print_theta(char* Title, const double theta[6])
{
	if (Title != NULL)	printf("Title = %s \r\n",Title);

	printf("   %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\r\n",theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]);
	getch();
}

void DBG_Print_P(char* Title, const double P[3])
{
	if (Title != NULL)	printf("%s : ",Title);

	printf("Px = %6.1f, Py = %6.1f, Pz = %6.1f\r\n",P[X],P[Y],P[Z]);
}

void DBG_LF(void)
{
	printf("\r\n");
	getch();
}

void DBG_R_default(double mtx[3][4])
{
	mtx[0][3] = -200;
	mtx[1][3] = 0;
	mtx[2][3] = -32.7;
}

void DBG_L_default(double mtx[3][4])
{
	mtx[0][3] = -200;
	mtx[1][3] = 0;
	mtx[2][3] = 32.7;
}
