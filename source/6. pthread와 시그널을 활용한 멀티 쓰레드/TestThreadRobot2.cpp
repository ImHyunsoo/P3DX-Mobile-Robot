/*
 * TestThreadRobot2.cpp
 *
 *  Created on: Jan 7, 2017
 *      Author: a
 */

#include <pthread.h>
#include <sys/wait.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

#include "Aria.h"

ArRobot g_robot;                             // 로봇에게 명령을 주거나 데이터를 받는 클래스
pthread_t g_sigid[3];	// 쓰레드 ID를 저장한다.
bool b_exit = false;     // 종료여부
//뮤택스 생성 및 초기화
//fast mutex 뮤택스 잠근 뒤 또 잠그면 멈춤(데드락)
// 재귀적 뮤텍스는 잠근뒤 몇번이고 더 잠글수 있음
//걸었던 만큼 풀어줘야함 PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
int g_count;

void *threadfunc2(void *arg);   //작업 쓰레드2
void *threadfunc(void *arg);    //작업 쓰레드1
void *s_signal(void *arg);     // 동기화 쓰레드3

void sig_handler(int signo)        //시그널 핸들러가 등록되면 시그널을 받을 때마다 실행
{
    printf("SIGNAL Handler TH ID %d : %d\n", pthread_self(),signo);
}

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

	  g_robot.enableMotors();      // 로봇의 모터를 사용할수 있게함

	  // disable sonar, enable motors, disable amigobot sound
	  g_robot.comInt(ArCommands::SONAR, 0);        // sonar 센서를 사용안함
	  g_robot.comInt(ArCommands::ENABLE, 1);       // motor 를 사용함
	  g_robot.comInt(ArCommands::SOUNDTOG, 0);     // sound 를 사용안함

	  g_robot.runAsync(true);        // 순환하는 스레드를 생성, 통신이 끊기면 루프 종료


    // 원하는 쓰레드로 시그널이 전달하는지 확인하기 위해서
    // 쓰레드 ID를 확인한다.
    pthread_create(&g_sigid[0], NULL, threadfunc, NULL);
    pthread_create(&g_sigid[1], NULL, threadfunc2, NULL);
    pthread_create(&g_sigid[2], NULL, s_signal, NULL);

    printf("thread1 생성 id %d\n", g_sigid[0]);
    printf("thread2 생성 id %d\n", g_sigid[1]);
    printf("thread3 생성 id %d\n", g_sigid[2]);

    while(1){
    	usleep(100*1000);
    	if(b_exit){
    		 pthread_cancel(g_sigid[0]);          // 각 쓰레드에 취소 요청을 보냄
    		 pthread_cancel(g_sigid[1]);
    		 pthread_cancel(g_sigid[2]);
    		break;
    	}
    }

    pthread_join(g_sigid[0], NULL);         // 각 쓰레드 끝날때까지 기다림료
    pthread_join(g_sigid[1], NULL);
    pthread_join(g_sigid[2], NULL);

    printf("thread1 종료 id %d\n", g_sigid[0]);
    printf("thread1 종료 id %d\n", g_sigid[1]);
    printf("thread1 종료 id %d\n", g_sigid[2]);

    pthread_mutex_destroy(&g_mutex);      //뮤텍스 없애기

    ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
     g_robot.stopRunning();         // 쓰레드 종료

     g_robot.waitForRunExit();  // 스레드가 종료할 때까지 일시 중단

     // exit
     ArLog::log(ArLog::Normal, "simpleConnect: Exiting.");
     Aria::exit(0);        // Aria 모든 프로세스/스레드 종료
     return 0;
}

void *threadfunc(void *arg)
{
	 ArJoyHandler joy;
	  joy.init();    // 조이스틱 초기화

	  int button8=0;  //##########

    struct sigaction act;   // 시그널 구조체
    sigset_t newmask;       // 시그널 마스크
    int lsigno;             // 시그널 저장

    if (!joy.haveJoystick())  // 초기화가 제대로 안됐으면 0 리턴
      {
        printf("Did not definitely detect a joystick, it may not work.\n");
      }

    sigemptyset(&newmask);         //시그널 마스크 비움
    sigaddset(&newmask, SIGINT);   // 시그널 마스크에 SIGINT 시그널 추가

    act.sa_handler = sig_handler;    //시그널 핸들러 함수 등록
    sigaction(SIGINT, &act, NULL);    // 시그널 함수 등록 //SIGINT 시그널이 나올 때마다 시그널 핸들러 함수 실행

    //취소요청을 받아 들일 것인지 아닌지를 결정
	//파라미터2개 필요함. 하나는 새로운 취소상태가 설정. 다른 하나는 이전 취소상태가 담길 변수
	// 두번째 파라미터가 NULL이면 이전 취소 상태에 대해 알 수 없음
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	//취소요청에 대한 반응을 결정. 이때 이 쓰레드는 취소될 수 있다고 가정
    	//취소요청을 즉시(비동기적으로) 처리하는것과 취소위치에 도착하기전까지 취소를 미루는 것
    	//취소를 미루려면 PTHREAD_CANCEL_DEFERRED
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	printf("버튼8을 누르면 종료\n");

	while(1)
    {
    	sigwait(&newmask, &lsigno);      // 시그널을 기다렸다가 시그널이 발생하면  다음줄 진행
    	                                 // lsigno에 받은 시그널 저장

         button8 = joy.getButton(8);
    	 if(button8)                       // 버튼8이 눌리면 종료
    		 b_exit=true;

/////////////////////////////////////////////////////////////////////////////////////////
         pthread_mutex_lock(&g_mutex);
////////////코딩시작

    	 pthread_mutex_unlock(&g_mutex);
//////////////////////////////////////////////////////////////////////////////////////////
    	  if(b_exit==true)
			{
				  printf("THREAD cancel 1\n");
				  pthread_testcancel();          // 취소 요청 받으면 즉시 쓰레드 종료
				  break;
			}
    }
}

void *threadfunc2(void *arg)
{
    struct sigaction act;
    int lsigno;

    sigset_t newmask;
    sigemptyset(&newmask);
    sigaddset(&newmask, SIGINT);

    act.sa_handler = sig_handler;
    sigaction(SIGINT, &act, NULL);


	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);


    while(1)
    {
    	sigwait(&newmask, &lsigno);
//////////////////////////////////////////////////////////////////
    	pthread_mutex_lock(&g_mutex);           //뮤택스 잠금
    	/// 코딩시작
    	printf("g_count:%d\n", g_count++);
    	pthread_mutex_unlock(&g_mutex);         // 뮤택스 풀기
    	g_robot.lock();
    	g_robot.setVel2(100,100);   // 100mm/s으로 로봇 구동
    	g_robot.unlock();
///////////////////////////////////////////////////////////////////
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
        usleep(1*1000);         //1ms 지연
        i++;

   //     printf("Send1 i=%d, SIGINT %d\n", i, sigid[0]);
        pthread_kill(g_sigid[0], SIGINT);            // SIGINT 시그널을 작업 쓰레드1에 전달

        if((i % 92) == 0)
        {
     //       printf("Send2 i=%d, SIGINT %d\n", i, sigid[1]);
            pthread_kill(g_sigid[1], SIGINT);          // SIGINT 시그널을 작업 쓰레드2에 전달
            i=0;
        }
        if(b_exit==true)
        {
        	pthread_kill(g_sigid[0], SIGINT);
        	pthread_kill(g_sigid[1], SIGINT);
        	  printf("THREAD cancel 3\n");
        	  pthread_testcancel();          // 취소 요청 받으면 즉시 쓰레드 종료
        	  break;
        }
    }

}
