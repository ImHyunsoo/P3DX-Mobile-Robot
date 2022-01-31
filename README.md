# P3DX-Mobile-Robot
이동 로봇 제어 실습 교육용 목적으로 만들어 만들어진 페이지입니다.
## 이동로봇 플랫폼: Pioneer 3-DX
 Pioneer 3-DX는 2개의 능동휠과 하나의 캐스터 휠로 구성된 차분 구동형(Differential Drive Type) 이동로봇 플랫폼이다. 본 이동로봇은 ActivMedia, Inc.에서 개발된 연구, 개발용 플랫폼으로 실제 환경에서 인식과 주행을 위한 기본적인 구성요소들을 탑재하고 있다.
 
- Pioneer의 특징은 다음과 같다.
  * Differential Drive Movement
  * Turn Radius: 0 cm
  * Max. Forward/Backward Speed: 1.2 m/s
  * Rotation Speed: 300°/s
  * Sonar Ring (8 front, 8 rear(optional))
  * Wheel encoders
  * Microcontroller with ARCOS(ActivMedia Robotics Operating System) firmware
  * RS-232 compatible serial port or Ethernet
  
<center><img src="https://user-images.githubusercontent.com/20950569/151878132-d7014a33-6863-4026-809d-5f79495f5a42.png" width="500", height="400"> </center>

## 목차
**1. 운영체제 (Operating System, OS) 설치**

     1.1 우분투 (Ubuntu)란?
     1.2 우분투 설치
**2. 컴파일 (Compile)**

     2.1 GCC (GNU C Compile) 컴파일
     2.2 분할 컴파일
     2.3 make 빌드
     2.4 cmake 빌드
**3. 통합개발환경 (Integrated Development Environment, IDE) 구축**

     3.1 QtCreator 설치
     3.2 QtCreator Hello World 작성하기 
**4. Aria (Advanced Robot Interface for Applicationd)**

     4.1 Aria 라이브러리 설치
     4.2 Aria 예제 get started
**5. 오도메트리 (Odometry)에 기반한 이동로봇의 위치인식**

     5.1 조이스틱 이용하여 로봇 제어
     5.2 kinematic equations에 기반한 기구학 모델 적용
**6. 멀티쓰레드 (Multi Thread)**
     
     6.1 쓰레드
     6.2 시그날
     6.3 쓰레드 동기화
**7. PID (Proportinal Integral Differential) 제어** 
     
     7.1 엔코더 (Encoder) 이해 및 현재 속도 구하기 
     7.2 타켓 속도 및 현재 속도에 기반한 PID 제어
**8. 시뮬레이터 (Simulator) 활용**

     8.1 MobileSim 시뮬레이터 설치
     8.2 프로그램된 코드 시뮬레이팅 
