#include "Aria.h"

#define MAX_V 500        // mm/s   1초에 50cm
#define MAX_W 0.872664   // rad/s  1초에 45도
#define ROBOT_L 168      // 로봇중심과 바퀴중심이 떨어진 거리 [mm]
#define PI  3.141592

ArRobot robot;                             // 로봇에게 명령을 주거나 데이터를 받는 클래스

double joyToVel(int joy)
{
	double vel; // mm/s
	double joy1;
	joy1 = (double)(((double)joy / (double)100)*MAX_V);   // y축 조이스틱 값에 따라 선형적으로
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

void robotControl(double vel, double angVel, int button1, int button2) // mm/s, rad/s
{
	double wh_vL;    // 완쪽 바퀴 선속도
	double wh_vR;
	double v = vel;
	double w = angVel;

	wh_vL = vel + angVel * ROBOT_L;              // 키네마틱 방정식 활용한 각 바퀴의 선속도 구하기
	wh_vR = vel - angVel * ROBOT_L;

	if(button1)
	{
	  if(wh_vL > 100)           // 선속도를 100mm/s 아니면 0
		  wh_vL = 100;
	  else if(wh_vL < -100)
		  wh_vL = -100;
	  else
		  wh_vL = 0;

	  if(wh_vR > 100)
		  wh_vR = 100;
	  else if (wh_vR < -100)
		  wh_vR = -100;
	  else
		  wh_vR = 0;
	  printf("wh_vL:%lf  wh_vR:%lf", wh_vL, wh_vR);

	  robot.lock();                      // 안전하게 로봇 데이터를 lock
	  robot.setVel2(wh_vL, wh_vR);       // 로봇이 각 바퀴의 선속도로 작동
	  robot.unlock();                    // lock을 풀음
	}

	else if (button2)
	{
	  if(w > 0.373)       // 각속도를 PI/2 아니면 0
		  w = PI / 2;
	  else if(w < -0.373)
		  w = -PI / 2;
	  else
		  w = 0;

	  wh_vL = v + w*ROBOT_L;
	  wh_vR = v - w*ROBOT_L;
	  printf("v:%6.3lf  w:%6.3lf ", v, w);
	  printf("wh_vL:%lf  wh_vR:%lf", wh_vL, wh_vR);
	  robot.lock();
	  robot.setVel2(wh_vL, wh_vR);
	  robot.unlock();
	}
	else                   // 선속도나 각속도에 제한없는 각 바퀴의 선속도
	{
	  printf("wh_vL:%lf  wh_vR:%lf", wh_vL, wh_vR);
	  robot.lock();
	  robot.setVel2(wh_vL, wh_vR);
	  robot.unlock();
	}
}

int main(int argc, char **argv)
{
  Aria::init();                              // Aria 어느함수 보다 먼저 실행, 전역적 데이터 구조체를 초기화
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();             // 명령어 줄을 파서 및 저장
  ArJoyHandler joy;
  joy.init();    // 조이스틱 초기화
  unsigned int i;
  double x, y, z;
  int xi, yi, zi;
  double v,w;
  int button1=0,button2=0, button8=0;

  ArRobotConnector robotConnector(&parser, &robot);   //로봇과 통신
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

  if (!joy.haveJoystick())  // 초기화가 제대로 안됐으면 0 리턴
   {
     printf("Did not definitely detect a joystick, it may not work.\n");
   }

  robot.enableMotors();      // 로봇의 모터를 사용할수 있게함

  // disable sonar, enable motors, disable amigobot sound
  robot.comInt(ArCommands::SONAR, 0);        // sonar 센서를 사용안함
  robot.comInt(ArCommands::ENABLE, 1);       // motor 를 사용함
  robot.comInt(ArCommands::SOUNDTOG, 0);     // sound 를 사용안함

  robot.runAsync(true);        // 순환하는 스레드를 생성, 통신이 끊기면 루프 종료

  while (Aria::getRunning()) //여전히 Aria 메인 스레드가 작동중이면 1
   {
     printf("\rButton ");
     for (i = 1; i <= joy.getNumButtons(); i++)  // 조이스틱 버튼 수 리턴
       printf(" %d:%d", i, joy.getButton(i));    // 버튼이 눌리면 1 아니면 0
     button1 = joy.getButton(1);
     button2 = joy.getButton(2);
	 button8 = joy.getButton(8);
	 if(button8)                       // 버튼8이 눌리면 반복문 빠져나옴
	     break;

     joy.getDoubles(&x, &y, &z);      // -1에서 1사이의 값으로 조정된 조이스틱 값을 얻음
     joy.getAdjusted(&xi, &yi, &zi);  // 설정된 속력에 기반한 정수형태로서 값을 얻음
     printf(" Axis x:%6.3f y:%6.3f (adj x:%d y:%d) ", x, y, xi, yi);

     joy.setSpeeds(100,100,100);    //  각 축에 대한 최대 값을 설정

   	 v = joyToVel(yi);              // yi 값을 기반으로 속력을 구함
	 w = joyToAngVel(xi);           // xi 값을 기반으로 각속도를 구함
	 printf("v:%6.3lf  w:%6.3lf ", v, w);

	 robotControl(v, w, button1, button2);   //  v,w로부터 각 바퀴의 선속도를 구함(키네마틱 방정식 활용)
	                                         	 	 // 버튼에 따른 각속도나 각 바퀴의 선속도를 제한

     ArUtil::sleep(1);  // 1ms 동안 기다림
   }

  ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning();         // 쓰레드 종료

  robot.waitForRunExit();  // 스레드가 종료할 때까지 일시 중단

  // exit
  ArLog::log(ArLog::Normal, "simpleConnect: Exiting.");
  Aria::exit(0);        // Aria 모든 프로세스/스레드 종료
  return 0;
}


