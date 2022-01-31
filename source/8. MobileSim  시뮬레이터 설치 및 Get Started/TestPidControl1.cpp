/*
 * TestPidControl1.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: a
 */

#include <pthread.h>  // 쓰레드 라이브러리 함수
#include <sys/wait.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>  // sleep 라이브러리 함수
#include <stdio.h>
#include <signal.h>   //  시그널 라이브러리 함수

#include "Aria.h"

#define MAX_V 500        // mm/s   1초에 50cm
#define MAX_W 0.872664   // rad/s  1초에 45도
#define ROBOT_L 168      // 로봇중심과 바퀴중심이 떨어진 거리 [mm]
#define PI  3.141592

ArRobot g_robot;                             // 로봇에게 명령을 주거나 데이터를 받는 클래스
pthread_t g_sigid[3];	// 쓰레드 ID를 저장한다.
ArTime g_start;                               /// 타이머 시작 변수
int g_button1, g_button2, g_button3;         // 조이스틱 버튼
double g_joy_v=0,g_joy_w=0;         //조이스틱-> v,w
double g_joy_lv=0, g_joy_rv=0;      //조이스틱 -> 각 바퀴 선속도
double g_robot_lv=0, g_robot_rv=0;     // 로봇 엔코더로 부터 읽어 들인 값으로 구한 값
bool b_exit = false;                // (조이스틱 버튼 8번)종료여부 저장
//뮤택스 생성 및 초기화
//fast mutex 뮤택스 잠근 뒤 또 잠그면 멈춤(데드락)
// 재귀적 뮤텍스는 잠근뒤 몇번이고 더 잠글수 있음
//걸었던 만큼 풀어줘야함 PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
	double  Ref;   			// Input: Reference input
	double  Fdb;   		// Input: Feedback input
	double  Err;			// Variable: Error
	double	ErrSum;		// Variable: Error Sum
	double  Kp;			// Parameter: Proportional gain
	double  Up;			// Variable: Proportional output
	double  Ui;			// Variable: Integral output
	double  Ud;			// Variable: Derivative output
	double  OutPreSat; 		// Variable: Pre-saturated output
	double  OutMax;		// Parameter: Maximum output
	double  OutMin;	    	// Parameter: Minimum output
	double  Out;   		// Output: PID output
	double  iMax;		// Parameter: Maximum Integration
	double  Ki;			// Parameter: Integral gain
	double  Kd; 		        // Parameter: Derivative gain
	double  ErrPrev;	   	// History: Previous error
	char* 	s;
			} PIDREG3;
typedef PIDREG3 *PIDREG3_handle;

// 100ms PID 제어주기
// 속도 제어 PID 게인
#define L_Kp 	25.0		//30.0		//30.0
#define L_Ki 	0.01		//5.0		//8.0
#define L_Kd 	3
#define R_Kp 	25.0		//30.0		//30.0
#define R_Ki 	0.01		//5.0		//8.0
#define R_Kd 	3
#define d_OutMax 	5900.0          // pid 최대값 지정
#define d_OutMin 	-5900.0
#define d_iMax 	5900.0            // i 최대값 지정


void pid_reg3_calc_v(PIDREG3_handle);

#define PIDREG3_V_LEFT     { 0.0, 0.0, 0.0, 0.0, L_Kp, 0.0, 0.0, 0.0, 0.0, \
                           d_OutMax, d_OutMin, 0.0, d_iMax, L_Ki, L_Kd, 0.0, "left"}
                           	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   //pid구조체 초기화할 값
#define PIDREG3_V_RIGHT     { 0.0, 0.0, 0.0, 0.0, R_Kp, 0.0, 0.0, 0.0, 0.0, \
                           d_OutMax, d_OutMin, 0.0, d_iMax, R_Ki, R_Kd, 0.0, "right"}


double joyToVel(int joy)
{
	double vel; // mm/s
	double joy1;
	joy1 = (double)(((double)joy / (double)100)*MAX_V);   // y축 조이스틱 값에    if (!laserDev.blockingConnect())
                                                                   // 따라 선형적으로
	vel = joy1;											  // 정규화하여 -500에서 500사이의 속력을 구함

	if(vel < 100 && vel > -100)                        // 조이스틱의 미묘한 움직임에는 로봇의 속력을 0으로 하기 위함
		vel = 0;

	return vel;
}

double joyToAngVel(int joy)
{
	double angVel;
	double joy1;
	joy1 = (double)(((double) joy / (double)100)*MAX_W);  // x축 조이스틱 값에 따라 선형적으로
	angVel = joy1;										  // 정규화하여 -0.872664에서 0.872664사이의 각속도를 구함

	if(angVel < 0.373 && angVel > -0.373)
		angVel = 0;

	return angVel;
}

void VelToWheelVel(double vel, double angVel){

	g_joy_lv = vel + angVel * ROBOT_L;              // 키네마틱 방정식 활용한 각 바퀴의 선속도 구하기
	g_joy_rv = vel - angVel * ROBOT_L;
}


void robotControl(double next_lv, double next_rv, int button1, int button2, int button3) // mm/s, rad/s  //##
{

	long int left;              //############ 엔코더
	long int right;				//############
	double leftVel;
	double rightVel;     // 오른쪽 바퀴 선속도
	double robotVel;   //   로봇 속도
	double robotAngVel;  //  로봇 각속도

	if(next_lv < 5 && next_lv > -5)       // 0과 가까우면 0으로 셋팅하여 멈추게함
		next_lv =0;
	if(next_rv < 5 && next_rv > -5)
			next_rv =0;

	if(button1)
	{
	  if(next_lv > 100)           // 선속도를 100mm/s 아니면 0
		  next_lv = 100;          // 로봇이 실제 1초 동안 10cm 이동했는지 확인해보기위해
	  else if(next_lv < -100)
		  next_lv = -100;
	  else
		  next_lv = 0;

	  if(next_rv > 100)
		  next_rv = 100;
	  else if (next_rv < -100)
		  next_rv = -100;
	  else
		  next_rv = 0;

	  printf("button1 next_lv:%lf  next_rv:%lf\n", next_lv, next_rv);

	  g_robot.lock();                      // 안전하게 로봇 데이터를 lock
	  g_robot.setVel2(next_lv, next_rv);       // 로봇이 각 바퀴의 선속도로 작동
	  g_robot.unlock();                    // lock을 풀음
	}
	else if (button2)
	{
	  robotVel = (next_lv + next_rv)/(double)2;
	  robotAngVel = (next_lv - next_rv)/((double)2*ROBOT_L);

	  if(robotAngVel > 0.373)       // 각속도를 PI/2 아니면 0
		  robotAngVel = PI / 2;     // 로봇의  1초에 90도 회전하는지 확인해보기위해
	  else if(robotAngVel < -0.373)
		  robotAngVel = -PI / 2;
	  else
		  robotAngVel = 0;

	  leftVel = robotVel + robotAngVel*ROBOT_L;
	  rightVel = robotVel - robotAngVel*ROBOT_L;
	  printf("robotVel:%6.3lf  robotAngVel:%6.3lf ", robotVel, robotAngVel);
	  printf("button2 leftVel:%lf  rightVel:%lf\n", leftVel, rightVel);
	  g_robot.lock();
	  g_robot.setVel2(leftVel, rightVel);
	  g_robot.unlock();
	}
	else if(button3)               //   1초동안 엔코더의 변화량을 살펴보기위해
	{
		g_robot.lock();
		g_robot.setVel2(100,100);  // 1초에 10cm
		g_robot.unlock();


	    g_start.setToNow();
	    while (1)
	    {
	      g_robot.lock();
	      if (g_start.mSecSince() > 5000)          //5초 동안 50cm
	      {
	        g_robot.unlock();
	        break;
	      }
	      g_robot.unlock();

	      if(g_start.mSecSince()>2000 && g_start.mSecSince() < 3000)   ///2초하고 3초사이 1초동안 엔코더 값의 변화 관찰
	    	  printf("222\n");
	      else if(g_start.mSecSince() >3000)
	    	  printf("333\n");
	      else
	    	  printf("***\n");

	      left = g_robot.getLeftEncoder();              //############
	      right = g_robot.getRightEncoder();			//############


	      printf("button3  ENCoder left:%ld right:%ld\n", left, right);		//############
	      ArUtil::sleep(50);
	    }
	    printf("aaaaaaaaaa\n");
	}
	else                   // 조이스틱 값에 따른 각 바퀴의 선속도
	{
	  printf("next_lv:%lf  next_rv:%lf\n", next_lv, next_rv);
	  g_robot.lock();
	  g_robot.setVel2(next_lv, next_rv);
	  g_robot.unlock();

	}
}
/*
 * 시그널 핸들러
 * 핸들러가 호출된 쓰레드의 ID와 시그널 번호를 출력한다.
 */

void sig_handler(int signo)            // 시그널 핸들러 함수  // sigaction에 이 함수가 등록되고 시그널을 받으면 실행됨
{
    printf("SIGNAL RECV TH ID %d : %d\n", pthread_self(),signo);
}

void *threadfunc2(void *arg);   // 작업 쓰레드2 선언
void *threadfunc(void *arg);    // 작업 쓰레드1 선언
void *s_signal(void *arg);		// 동기화 쓰레드3 선언



int main(int argc, char **argv)
{
	  Aria::init();                              // Aria 어느함수 보다 먼저 실행, 전역적 데이터 구조체를 초기화
	  ArArgumentParser parser(&argc, argv);
	  parser.loadDefaultArguments();             // 명령어 줄을 파서 및 저장

	  ArRobotConnector robotConnector(&parser, &g_robot);   //로봇과 통신
	  if(!robotConnector.connectRobot())                  //통신 실패시
	  {
	    ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot."); // 콘솔창에 문자열 출력
	    if(parser.checkHelpAndWarnUnparsed())
	    {
	        // -help not given
	        Aria::logOptions();
	        Aria::exit(1);
	    }
	  }

	  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	  {
	    Aria::logOptions();
	    Aria::exit(1);
	  }
	  ArLog::log(ArLog::Normal, "simpleConnect: Connected to robot.");
	//////////////////////////////////////////////////////////////
//// 레이저 - 시뮬레이터에서 사용
	ArLaserConnector laserConnector(&parser, &g_robot, &robotConnector);

	  // Parse command line arguments (there may be arguments specifying 
	  // what lasers to try to connect to)
	  if(!Aria::parseArgs())
	  {
		Aria::logOptions();
		Aria::exit(2);
	  }

	  // Try connecting to all lasers specified in the robot's parameter file
	  // and in command line arguments
	  if(!laserConnector.connectLasers())
	  {
		ArLog::log(ArLog::Terse, "Error, could not connect to lasers.\n");
		Aria::logOptions();
		Aria::exit(3);
	  }
	  
	  //////////////////////////////////////////////////////////////
	  g_robot.enableMotors();      // 로봇의 모터를 사용할수 있게함

	  // disable sonar, enable motors, disable amigobot sound
	  g_robot.comInt(ArCommands::SONAR, 0);        // sonar 센서를 사용안함
	  g_robot.comInt(ArCommands::ENABLE, 1);       // motor 를 사용함
	  g_robot.comInt(ArCommands::SOUNDTOG, 0);     // sound 를 사용안함

	  g_robot.requestEncoderPackets();                  //################

	  g_robot.runAsync(true);        // 순환하는 스레드를 생성, 통신이 끊기면 루프 종료



    pthread_create(&g_sigid[0], NULL, threadfunc, NULL);        // 각 쓰레드 생성
    pthread_create(&g_sigid[1], NULL, threadfunc2, NULL);      // 첫번째 매개변수에 쓰레드 아이디 저장
    pthread_create(&g_sigid[2], NULL, s_signal, NULL);      // 세번째 매개변수에 시그널 함수 주소 등록
//// 원하는 쓰레드로 시그널이 전달하는지 확인하기 위해서
    // 쓰레드 ID를 확인한다.
//    printf("thread1 id %d\n", sigid[0]);
//    printf("thread2 id %d\n", sigid[1]);
//    printf("thread3 id %d\n", sigid[2]);

    while(1){
    	usleep(100*1000);
    	if(b_exit == true){
    		 pthread_cancel(g_sigid[0]);          // 각 쓰레드에 취소 요청을 보냄
    		 pthread_cancel(g_sigid[1]);
    		 pthread_cancel(g_sigid[2]);
    		break;
    	}
    }

    pthread_join(g_sigid[0], NULL);         // 각 쓰레드 끝날때까지 기다림
    pthread_join(g_sigid[1], NULL);
    pthread_join(g_sigid[2], NULL);

    pthread_mutex_destroy(&g_mutex);      //뮤택스 없앰

    ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
     g_robot.stopRunning();         // 쓰레드 종료

     g_robot.waitForRunExit();  // 스레드가 종료할 때까지 일시 중단

     // exit
     ArLog::log(ArLog::Normal, "simpleConnect: Exiting.");
     Aria::exit(0);        // Aria 모든 프로세스/스레드 종료
     return 0;
}


void pid_reg3_calc_v(PIDREG3 *v)
{
	float outpresave;

	// Compute the error
	v->Err = v->Ref - v->Fdb;

	// Compute the error sum
	v->ErrSum = v->ErrSum + v->Err;

	// Compute the proportional output
	v->Up = v->Kp * v->Err;

	// Compute the integral output
	v->Ui = v->Ki*v->ErrSum;
	if(v->Ui > v->iMax) 			v->Ui = v->iMax;
	else if(v->Ui < (-1.0 * v->iMax) ) 	v->Ui = -1.0 * v->iMax;

	// Compute the derivative output
	v->Ud = v->Kd * (v->Err - v->ErrPrev);

	// Compute the pre-saturated output
	v->OutPreSat = v->Up + v->Ui + v->Ud;
	outpresave = v->OutPreSat;

	outpresave *= 0.4;        /////////// pid값에 ratio=0.4(임의로 정한값)을 곱함
	                             // 이 값은 현재 피드백값에 더해주어 레퍼런스값에 점차 도달해감
	 v->OutPreSat *= 0.4;

/////////////////////////////////////////////////////////////////////////////////// 이 부분은 PWM 제어시 필요
	// PWM 값이 20KHz에서 0 ~ 4000 까지의 값을 가질 수 있지만
	// 최소 어느 정도의 값을 줘야 실제 로봇(O2)의 바퀴가
	// 천천히 돌기 시작하기 때문에 다음과 같이 한다.
//	if(v->OutPreSat < d_OutMax && v->OutPreSat > 0.0 && v->Ref!=0) 		v->OutPreSat = d_OutMax;
//	else if(v->OutPreSat > d_OutMin && v->OutPreSat < 0.0 && v->Ref!=0)	v->OutPreSat = d_OutMin;
/////////////////////////////////////////////////////////////////////////////////////


	// Saturate the output
	if(v->OutPreSat > v->OutMax)	v->Out =  v->OutMax;
	else if(v->OutPreSat < v->OutMin)	v->Out =  v->OutMin;
	else 						v->Out = v->OutPreSat;
	v->ErrPrev = v->Err;

//	printf("Vcalc: ");
//	printf("Kp: %d   Ki: %d   Kd: %d ", (int)v->Kp, (int)v->Ki, (int)v->Kd );
	if(!strcmp(v->s, "left")){
		printf("left@@ Ref: %d   Fdb: %d   Err: %d   ErrSum: %d   Outpre: %d(%d+%d+%d)  \
	Out: %d  ", (int)(v->Ref), (int)(v->Fdb), (int)(v->Err), (int)(v->ErrSum), (int)(outpresave), \
	(int)(v->Up), (int)(v->Ui), (int)(v->Ud), (int)(v->Out) );
	}
//	else if(!strcmp(v->s, "right"))
//			printf("right@@ Ref: %d   Fdb: %d   Err: %d   ErrSum: %d   Outpre: %d(%d+%d+%d) \
	Out: %d  \n", (int)(v->Ref), (int)(v->Fdb), (int)(v->Err), (int)(v->ErrSum), (int)(outpresave), \
	(int)(v->Up), (int)(v->Ui), (int)(v->Ud), (int)(v->Out) );
}

void *threadfunc(void *arg)
{
	 ArJoyHandler joy;
	  joy.init();    // 조이스틱 초기화

	  double x, y, z;
	  int xi, yi, zi;

	  int button8=0;  //##########

    int i=0;
    struct sigaction act;     // 시그널 구조체
    sigset_t newmask;         // 시느러 마스크
    int lsigno;				  // 시그널 저장


    if (!joy.haveJoystick())  // 초기화가 제대로 안됐으면 0 리턴
      {
        printf("Did not definitely detect a joystick, it may not work.\n");
      }
    // 결과의 확인을 위해서 쓰레드 ID를 출력한다.
//    printf("SIGINT Thread Start %d\n", pthread_self());
    sigemptyset(&newmask);          // 시그널 마스크 비움
    sigaddset(&newmask, SIGINT);	// 시그널 마스크에 SIGINT 시그널 추가

    act.sa_handler = sig_handler;   // 시그널 핸들러 함수 등록
    sigaction(SIGINT, &act, NULL);  // 시그널 함수 등록 //SIGINT 시그널이 나올 때마다 시그널 핸들러 함수 실행

    //취소요청을 받아들일 것인지 아닌지를 결정
    // 파라미터 2개 필요함.// 하나는 새로운 취소상태가 설정// 다른하나는 이전 취소상태가 담길 변수
    // 두번째 파라미터가 NULL이면 이전 취소 상태에 대해 알 수 없음
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	// 취소요청에 대한 반응을 결정 // 이때 이 쓰레드는 취소될 수 있다고 가정
	// 취소요청을 즉시(비동기적으로) 처리하는 것과 취소위치에 도착하기전까지 취소를 미루는 것
	// 취소를 미루려면 PTHREAD_CANCEL_DEFERRED
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while(1)
    {
    	i++;
    	sigwait(&newmask, &lsigno);         // 시그널을 기다렸다가 시그널이 발생하면 다음줄 진행
                                           // lsigno에  받은 시그널 저장
        //////////////////////////////////////////////////////////////////////////////////
//        printf("\rButton ");
//         for (i = 1; i <= joy.getNumButtons(); i++)  // 조이스틱 버튼 수 리턴
//           printf(" %d:%d", i, joy.getButton(i));    // 버튼이 눌리면 1 아니면 0
         g_button1 = joy.getButton(1);
         g_button2 = joy.getButton(2);
    	 g_button3 = joy.getButton(3);		//##########
         button8 = joy.getButton(8);

    	 if(button8)                       // 버튼8이 눌리면 반복문 빠져나옴
    		 b_exit=true;

         joy.getDoubles(&x, &y, &z);      // -1에서 1사이의 값으로 조정된 조이스틱 값을 얻음
         joy.getAdjusted(&xi, &yi, &zi);  // 설정된 속력에 기반한 정수형태로서 값을 얻음
//         printf(" Axis x:%6.3f y:%6.3f (adj x:%d y:%d) ", x, y, xi, yi);

         joy.setSpeeds(100,100,100);    //  각 축에 대한 최대 값을 설정
/////////////////////////////////////////////////////////////////////////////////////////
         pthread_mutex_lock(&g_mutex);        // 뮤택스 잠금
       	 g_joy_v = joyToVel(yi);              // yi 값을 기반으로 속력을 구함
    	 g_joy_w = joyToAngVel(xi);           // xi 값을 기반으로 각속도를 구함
    	 VelToWheelVel(g_joy_v, g_joy_w);
//    	 printf("g_joy_v:%6.3lf  g_joy_w:%6.3lf g_joy_lv:%6.3lf  g_joy_rv:%6.3lf \n", \
    	 g_joy_v, g_joy_w, g_joy_lv, g_joy_rv);
    	 pthread_mutex_unlock(&g_mutex);    // 뮤택스 풀기
//////////////////////////////////////////////////////////////////////////////////////////

    	  if(b_exit==true)
			{
				  printf("THREAD cancel 1\n");
				  pthread_testcancel();        // 취소 요청을 받으면 즉시 쓰레드 종료
				  break;
			}
    }
}

void *threadfunc2(void *arg)
{
    struct sigaction act;
    int lsigno;
    long int E_left;    // 좌측 엔코더 정보 담을 변수
    long int E_right;    // 우측 엔코더 정보 담을 변수
    long int temp_E_left;   // 좌측 엔코더의 전 주기에 받았던 정보 담을 변수
    long int temp_E_right;
    long int gap_E_left;    //  한 주기동안 좌측 엔코더 변화량을 담을 변수
    long int gap_E_right;
    double V_left;         // 좌측 바퀴의 선속도를 aria api 함수로 읽어서 저장할 변수
    double V_right;
    int i=0;
    struct timeval start_time;       // 주기를 측정하기 위한 타이머 구조체 생성
    struct timeval end_time;
    unsigned int cycle_time;     // 측정한 한 주기를 담을 변수

    sigset_t newmask;
    sigemptyset(&newmask);
    sigaddset(&newmask, SIGINT);

    act.sa_handler = sig_handler;
    sigaction(SIGINT, &act, NULL);


	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);


//   	g_robot.lock();
//   	g_robot.setVel2(500,500);  // 1초에 10cm
//   	g_robot.unlock();

	gettimeofday(&start_time, NULL);       // 현재 시간을 가져옴
	gettimeofday(&end_time, NULL);


	PIDREG3 pidVR = PIDREG3_V_RIGHT;       // pid 구조체를 초기화
	PIDREG3 pidVL = PIDREG3_V_LEFT;
    double leftS;               // 왼쪽 바퀴 pid 결고물을 저장
    double rightS;

    //  (다음 목표속도) = (현재 목표속도) + (가속도)*(타이머 주기)
    double now_lv =0;          // 현재 왼쪽 바퀴의 읽어들인 속도
    double next_lv=0;          //  다음 왼쪽 바퀴의 목표 속도
    double d_add_lv=0;         //  왼쪽 바퀴의 현재속도에 더해주어야할 (가속도)*(타이머주기) 변수 \
    (가속도는 pid 결과값으로 입력)
    double now_rv =0;
	double next_rv=0;
	double d_add_rv=0;

    while(1)
    {
    	gettimeofday(&start_time, NULL);
    	sigwait(&newmask, &lsigno);

    	gettimeofday(&end_time, NULL);

    	if(i==0){
    		i=1;
    		  if(g_robot.getLeftEncoder()>=0)
				  E_left = g_robot.getLeftEncoder();
			  else
				  E_left = g_robot.getLeftEncoder() + 65536;
    		  //getLeftEncoder()는 16비트 처리 범위  -32768 ~ 32767 \
    		      		  안에서  엔코더값 읽어드림 ex) 0->32767->-32768->0 반복(정방향)
    	// 이를 0 ~ 65535 값으로 변환

    		  if(g_robot.getRightEncoder()>=0)
				  E_right = g_robot.getRightEncoder();
			  else
				  E_right = g_robot.getRightEncoder() + 65536;

    		temp_E_left = E_left;      // 처음에 이전 엔코더값과 현재 엔코더값을 값게 설정
    		temp_E_right = E_right;
    	}else{
    		  if(g_robot.getLeftEncoder()>=0)
				  E_left = g_robot.getLeftEncoder();
			  else
				  E_left = g_robot.getLeftEncoder() + 65536;

			  if(g_robot.getRightEncoder()>=0)
				  E_right = g_robot.getRightEncoder();
			  else
				  E_right = g_robot.getRightEncoder() + 65536;
    	}


		  if(end_time.tv_usec > start_time.tv_usec)                       // 주기를 구함 => 약 0.1초
			  cycle_time = end_time.tv_usec  - start_time.tv_usec;
		  else
			  cycle_time = end_time.tv_usec+1000000  - start_time.tv_usec;

		  if(E_left - temp_E_left > 50000)                  //65536에서 1추가 되면 1부터 다시
			      gap_E_left = E_left - temp_E_left - 65536; // 시작하므로 올바른 카운트 변화값 측정을 위해 이와같이 처리
		  else if(E_left - temp_E_left < -50000)
				  gap_E_left = E_left - temp_E_left + 65536;
		  else
			  	  gap_E_left = E_left - temp_E_left;

		  if(E_right - temp_E_right > 50000)
				  gap_E_right = E_right - temp_E_right - 65536;
		  else if(E_right - temp_E_right < -50000)
				  gap_E_right = E_right - temp_E_right + 65536;
		  else
			      gap_E_right = E_right - temp_E_right;

//		  g_robot_lv = (double)(gap_E_left) / (double)128  /((double)cycle_time/(double)1000000);  \
		  //// 엔코더 128 counts/mm      => mm
//		  g_robot_rv = (double)(gap_E_right) /(double)128 /((double)cycle_time/(double)1000000);  \
		  //  0.1초는 주기   			=> mm/s
		  g_robot_lv = (double)(gap_E_left) / (double)128 /0.1;     // 엔코더 128 counts/mm      => mm
		  g_robot_rv = (double)(gap_E_right) /(double)128 /0.1;     //  0.1초는 주기   			=> mm/s
		  // 엔코더 읽어들이는 api함수가 0.1초 주기로 정확한 수치를 출력함 따라서 구한 주기값 말고 0.1초로 나눠줌


		  V_left = g_robot.getLeftVel();        //왼쪽 바퀴의 선속도를 읽음
		  V_right = g_robot.getRightVel();


		  pidVL.Ref = double(g_joy_lv);      // 조이스틱으로 원하는 속도를 주입
		  pidVR.Ref = double(g_joy_rv);
		  pidVL.Fdb = g_robot_lv;            // 현재 로봇의 속도를 입력
		  pidVR.Fdb = g_robot_rv;
		  pid_reg3_calc_v(&pidVL);           // pid 값을 계산
		  pid_reg3_calc_v(&pidVR);
	      leftS = pidVL.Out;                 // pid 값을 저장
	      rightS = pidVR.Out;


	      d_add_lv = leftS*0.1;            // (가속도)*(타이머주기) 구함
	      now_lv = g_robot_lv;             // 현재 왼쪽 바퀴의 선속도 저장
	      next_lv = now_lv + d_add_lv;    //  다음 목표의 왼쪽 바퀴 선속도 저장

	      d_add_rv = rightS*0.1;
		  now_rv = g_robot_rv;
		  next_rv = now_rv + d_add_rv;


	      robotControl(next_lv, next_rv, g_button1, g_button2, g_button3);   //  로봇을 각 바퀴의 선속도로 컨트롤함

		  temp_E_left = E_left;      // 지금의 엔코더 값을 임시 저장
		  temp_E_right = E_right;

		  printf("  ENCoder left:%ld right:%ld,  V_left:%lf, g_robot_lv:%lf V_right:%lf, g_robot_rv:%lf  cycle:%lf\n", \
				  E_left, E_right, V_left, g_robot_lv, V_right, g_robot_rv, ((double)cycle_time/(double)1000000));



		  if(b_exit)
			{
				  printf("THREAD cancel 2\n");
				  pthread_testcancel();
				  break;
		    }
    }
}


/*
 * SIGINT를 두개의 쓰레드로 서로다른 시간간격으로
 * 전달한다.
 */

void *s_signal(void *arg)
{
    int i = 0;


	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);


    while(1)
    {
        usleep(1*1000);  //1ms 지연
        i++;

   //     printf("Send1 i=%d, SIGINT %d\n", i, sigid[0]);
        pthread_kill(g_sigid[0], SIGINT);   // SIGINT 시그널을 작업 쓰레드1에 전달

        if((i % 92) == 0)
        {
     //       printf("Send2 i=%d, SIGINT %d\n", i, sigid[1]);
            pthread_kill(g_sigid[1], SIGINT);     // SIGINT 시그널을 작업 쓰레드2에 전달
            i=0;
        }
        if(b_exit==true)
        {
//        	pthread_kill(sigid[0], SIGINT);
        	pthread_kill(g_sigid[1], SIGINT);
        	  printf("THREAD cancel 3\n");
        	  pthread_testcancel();          // 취소 요청 받으면 즉시 쓰레드 종료
        	  break;
        }
    }

}
