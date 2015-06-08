#include "simmower.h"

#define OBSTACLE_RADIUS 8

SimLED LED;
SimMotor Motor;
SimMotorMow MotorMow;
SimSettings Settings;
SimPerimeter Perimeter;
SimBattery Battery;
SimTimer Timer;
SimRobot Robot;


// ------------------------------------------

void SimSettings::setup(){
  randomSeed( time(NULL) );
  // --- gear motors ----
  Motor.motorSpeedMaxPwm = 255;
  Motor.motorLeftSwapDir = false;
  Motor.motorRightSwapDir = false;
  Motor.motorSpeedMaxRpm = 25;
  Motor.enableStallDetection = true;
  Motor.enableErrorDetection = false;
  Motor.odometryTicksPerRevolution = 1060;   // encoder ticks per one full resolution
  Motor.odometryTicksPerCm = 13.49;    // encoder ticks per cm
  Motor.odometryWheelBaseCm = 36;    // wheel-to-wheel distance (cm)
  Motor.motorLeftPID.Kp       = 0.4;    // PID speed controller
  Motor.motorLeftPID.Ki       = 0.0;
  Motor.motorLeftPID.Kd       = 0.0;
  // --- mower motors ----
  MotorMow.motorMowSpeedMaxPwm = 255;
  // --- perimeter motors ----
  Perimeter.enable = true;
  Robot.perimeterPID.Kp    = 160;  // perimeter PID controller
  Robot.perimeterPID.Ki    = 4;
  Robot.perimeterPID.Kd    = 50;
  // --- battery ----
  Battery.enableMonitor = true;
  Battery.batFull = 29.4;
  //Battery.batGoHomeIfBelow = 29.39;
  Battery.batGoHomeIfBelow = 29.2;
  Battery.batVoltage = Battery.batFull;
}

// ------------------------------------------

void SimMotor::setDriverPWM(int leftMotorPWM, int rightMotorPWM){
}

void SimMotor::readOdometry(){
}

void SimMotor::readCurrent(){
  if (enableStallDetection) {
    if (!motorLeftStalled){
      if ( Perimeter.hitObstacle(Robot.simX, Robot.simY, Motor.odometryWheelBaseCm/2+OBSTACLE_RADIUS, Robot.simOrientation )){
        if ( (Motor.motorLeftSpeedRpmSet > 2) && (Motor.motorLeftSpeedRpmSet > 2) )  {
          motorLeftStalled = true;
          Console.print(F("  LEFT STALL"));
          Console.println();
          motorLeftStalled = true;
          stopImmediately();
        }
      }
    }
  }
}
// ------------------------------------------

void SimMotorMow::setDriverPWM(int pwm){
}

// ------------------------------------------

SimPerimeter::SimPerimeter(){
  drawMowedLawn = true;
  memset(lawnMowStatus, 0, sizeof lawnMowStatus);
  //printf("%d\n", sizeof bfield);
  memset(bfield, 0, sizeof bfield);
  imgBfield = cv::Mat(WORLD_SIZE_Y, WORLD_SIZE_X, CV_8UC3, cv::Scalar(0,0,0));
  imgWorld = cv::Mat(WORLD_SIZE_Y, WORLD_SIZE_X, CV_8UC3, cv::Scalar(0,0,0));

  // obstacles (cm)
  obstacles.push_back( (point_t) {120, 155 } );
  obstacles.push_back( (point_t) {70, 165 } );
  obstacles.push_back( (point_t) {300, 190 } );
  obstacles.push_back( (point_t) {210, 350 } );
  obstacles.push_back( (point_t) {120, 190 } );
  obstacles.push_back( (point_t) {250, 160 } );
  obstacles.push_back( (point_t) {220, 110 } );
  obstacles.push_back( (point_t) {110, 200 } );
  obstacles.push_back( (point_t) {90, 390 } );
  obstacles.push_back( (point_t) {90, 390 } );
  obstacles.push_back( (point_t) {90, 330 } );

  // perimeter lines coordinates (cm)
  std::vector<point_t> list;
  list.push_back( (point_t) {30, 55 } );
  list.push_back( (point_t) {50, 45 } );
  list.push_back( (point_t) {610, 40 } );
  list.push_back( (point_t) {620, 50 } );
  list.push_back( (point_t) {630, 180 } );
  list.push_back( (point_t) {450, 200 } );
  list.push_back( (point_t) {420, 290 } );
  list.push_back( (point_t) {410, 450 } );
  list.push_back( (point_t) {60, 450 } );
  list.push_back( (point_t) {40, 450 } );
  list.push_back( (point_t) {30, 430 } );

  chgStationX = 30;
  chgStationY = 150;

  // compute magnetic field (compute distance to perimeter lines)
  int x1 = list[list.size()-1].x;
  int y1 = list[list.size()-1].y;
  // for each perimeter line
  for (int i=0; i < list.size(); i++){
    int x2 = list[i].x;
    int y2 = list[i].y;
    int dx = (x2-x1);
    int dy = (y2-y1);
    int len=(sqrt( dx*dx + dy*dy )); // line length
    float phi = atan2(dy,dx); // line angle
    // compute magnetic field for points (x,y) around perimeter line
    for (int y=-200; y < 200; y++){
      for (int x=-100; x < len*2+100-1; x++){
        int px= x1 + cos(phi)*x/2 - sin(phi)*y;
        int py= y1 + sin(phi)*x/2 + cos(phi)*y;
        int xend = max(0, min(len, x/2)); // restrict to line ends
        int cx = x1 + cos(phi)*xend;  // cx on line
        int cy = y1 + sin(phi)*xend;  // cy on line
        if ((py >= 0) && (py < WORLD_SIZE_Y)
           && (px >=0) && (px < WORLD_SIZE_X)) {
          float r = max(0.000001, sqrt( (cx-px)*(cx-px) + (cy-py)*(cy-py) ) ) / 10; // distance to line (meter)
          float b=100.0/(2.0*M_PI*r); // field strength
          int c = pnpoly(list, px, py);
          //if ((y<=0) || (bfield[py][px] < 0)){
          if (c != 0){
            b=b*-1.0;
            bfield[py][px] =  min(bfield[py][px], b);
          } else bfield[py][px] = max(bfield[py][px], b);
        }
      }
    }
    x1=x2;
    y1=y2;
  }

  // draw magnetic field onto image
  for (int y=0; y < WORLD_SIZE_Y; y++){
    for (int x=0; x < WORLD_SIZE_X; x++) {
      float b=30 + 30*sqrt( abs(getBfield(x,y)) );
      //b:=10 + bfield[y][x];
      int v = min(255, max(0, (int)b));
      cv::Vec3b intensity;
      if (bfield[y][x] <= 0){
        intensity.val[0]=255-v;
        intensity.val[1]=255-v;
        intensity.val[2]=255;
      } else {
        intensity.val[0]=255;
        intensity.val[1]=255-v;
        intensity.val[2]=255-v;
      }
      imgBfield.at<cv::Vec3b>(y, x) = intensity;
    }
  }
}

// x,y: cm
float SimPerimeter::getBfield(int x, int y, int resolution){
  float res = 0;
  if ((x >= 0) && (x < WORLD_SIZE_X) && (y >= 0) && (y < WORLD_SIZE_Y)){
    int xd = ((int)((x+resolution/2)/resolution))*resolution;
    int yd = ((int)((y+resolution/2)/resolution))*resolution;
    res = bfield[yd][xd];
  }
  //float measurement_noise = 0.5;
  //res += gauss(0.0, measurement_noise);
  return res;
}

void SimPerimeter::run(){
}

// return world size (cm)
int SimPerimeter::sizeX(){
  return WORLD_SIZE_X;
}

int SimPerimeter::sizeY(){
  return WORLD_SIZE_Y;
}


void SimPerimeter::draw(){
  char buf[64];
  sprintf(buf, " (%dcm x %dcm)", WORLD_SIZE_X, WORLD_SIZE_Y);
  imshow("world " + std::string(buf), imgWorld);
  imgBfield.copyTo(imgWorld);
  // draw lawn mowed status
  if (drawMowedLawn){
    cv::Vec3b intensity;
    intensity.val[0]=0;
    intensity.val[1]=255;
    intensity.val[2]=0;
    for (int y=0; y < WORLD_SIZE_Y; y++){
      for (int x=0; x < WORLD_SIZE_X; x++){
        if (lawnMowStatus[y][x] > 0) imgWorld.at<cv::Vec3b>(y, x) = intensity;
      }
    }
  }
  // draw charging station
  circle( imgWorld, cv::Point( chgStationX, chgStationY), 10, cv::Scalar( 0, 0, 0 ), -1, 8 );
  // draw obstacles
  for (int i=0; i < obstacles.size(); i++){
    int x = obstacles[i].x;
    int y = obstacles[i].y;
    circle( imgWorld, cv::Point( x, y), 10, cv::Scalar( 0, 255, 255 ), -1, OBSTACLE_RADIUS );
  }
  // draw information
  sprintf(buf, "time %.0fmin bat %.1fv dist %.0fm  orient %d",
          Timer.simTimeTotal/60.0,
          Battery.batVoltage,
          Motor.totalDistanceTraveled,
          ((int)(Robot.simOrientation / PI * 180.0))  );
  putText(imgWorld, std::string(buf), cv::Point(10,WORLD_SIZE_Y-15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0) );

}

// approximate circle pattern
// 010
// 111
// 010
// x,y: cm
void SimPerimeter::setLawnMowed(int x, int y){
  int r = 8;
  if (  (x <= 2*r) || (x >= WORLD_SIZE_X-2*r ) || (y <= 2*r) || (y >= WORLD_SIZE_Y-2*r )  )return;
  for (int i=-r; i <= r; i++){
    for (int j=-r; j <= r; j++){
      lawnMowStatus[y+i][x+j] = 1.0;
    }
  }
}


// checks if point is inside polygon
// The algorithm is ray-casting to the right. Each iteration of the loop, the test point is checked against
// one of the polygon's edges. The first line of the if-test succeeds if the point's y-coord is within the
// edge's scope. The second line checks whether the test point is to the left of the line
// If that is true the line drawn rightwards from the test point crosses that edge.
// By repeatedly inverting the value of c, the algorithm counts how many times the rightward line crosses the
// polygon. If it crosses an odd number of times, then the point is inside; if an even number, the point is outside.

int SimPerimeter::pnpoly(std::vector<point_t> &vertices, float testx, float testy)
{
  int i, j, c = 0;
  int nvert = vertices.size();
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((vertices[i].y>testy) != (vertices[j].y>testy)) &&
     (testx < (vertices[j].x-vertices[i].x) * (testy-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) )
       c = !c;
  }
  return c;
}

bool SimPerimeter::hitObstacle(int x, int y, int distance, float orientation){
  for (int i=0; i < obstacles.size(); i++){
    int ox = obstacles[i].x;
    int oy = obstacles[i].y;
    float odistance = sqrt( sq(abs(ox-x)) + sq(abs(oy-y)) );
    if (odistance <= distance) {
      //float obstacleCourse = atan2(oy-y, ox-x);
      //if (abs(distancePI(obstacleCourse, orientation)) < PI/2) return true;
      return true;
    }
  }
  return false;
}

bool SimPerimeter::isInside(char coilIdx){
  float r = Motor.odometryWheelBaseCm/2;
  int receiverX = Robot.simX + r * cos(Robot.simOrientation);
  int receiverY = Robot.simY + r * sin(Robot.simOrientation);
  float bfield = getBfield(receiverX, receiverY, 1);
  //printf("bfield=%3.2f\n", bfield);
  return (bfield < 0);
}

int SimPerimeter::getMagnitude(char coilIdx){
  float r = Motor.odometryWheelBaseCm/2;
  int receiverX = Robot.simX + r * cos(Robot.simOrientation);
  int receiverY = Robot.simY + r * sin(Robot.simOrientation);
  float bfield = getBfield(receiverX, receiverY, 1);
  return min(4095, max(-4095, bfield * 10));
}

// ------------------------------------------

SimTimer::SimTimer(){
  simTimeStep = 0.01; // one simulation step (seconds)
  simTimeTotal = 0; // simulation time
  simStopped = false;
  simFast = false;
}

unsigned long SimTimer::millis(){
  //printf("%.1f\n", timeTotal);
  return simTimeTotal * 1000.0;
}

void SimTimer::run(){
  TimerControl::run();
  simTimeTotal += simTimeStep;
}

// ------------------------------------------

void SimBattery::read(){
  batVoltage -= 0.001;
}


bool SimBattery::chargerConnected(){
  float r = Motor.odometryWheelBaseCm/2;
  int chargePinX = Robot.simX + r * cos(Robot.simOrientation);
  int chargePinY = Robot.simY + r * sin(Robot.simOrientation);
  float stationDistance = sqrt( sq(abs(Perimeter.chgStationX-chargePinX)) + sq(abs(Perimeter.chgStationY-chargePinY)) );
  float stationOrient = abs(distancePI(Robot.simOrientation, -PI/2));
  /*printf("stationDistance=%.0f orient=%.0f\n",
         stationDistance,
         stationOrient / PI * 180.0);*/
  if ( (stationDistance < 10) && (stationOrient < PI/8)  ) return true;
  return false;
}

// ------------------------------------------

// initializes robot
SimRobot::SimRobot(){
  distanceToChgStation = 0;

  simX = 340;
  simY = 100;
  simOrientation = 0;
  //leftMotorSpeed = 30;
  //rightMotorSpeed = 5;

  motor_noise = 90;
  slope_noise = 5;
}

void SimRobot::move(){
  float cmPerRound = Motor.odometryTicksPerRevolution / Motor.odometryTicksPerCm;

  // motor pwm-to-rpm
  float leftnoise = 0;
  float rightnoise = 0;
  if (abs(Motor.motorLeftPWMCurr) > 2) leftnoise = motor_noise;
  if (abs(Motor.motorRightPWMCurr) > 2) rightnoise = motor_noise;
  Motor.motorLeftRpmCurr  = min(33, max(-33, 0.9 * Motor.motorLeftRpmCurr  + 0.1 * gauss(Motor.motorLeftPWMCurr,  leftnoise)));
  Motor.motorRightRpmCurr = min(33, max(-33, 0.9 * Motor.motorRightRpmCurr + 0.1 * gauss(Motor.motorRightPWMCurr, rightnoise)));

  // slope
  float leftrpm;
  float rightrpm;
  leftnoise = 0;
  rightnoise = 0;
  if (abs(Motor.motorLeftPWMCurr) > 2) leftnoise = slope_noise;
  if (abs(Motor.motorRightPWMCurr) > 2) rightnoise = slope_noise;
  leftrpm  = gauss(Motor.motorLeftRpmCurr, leftnoise);
  rightrpm = gauss(Motor.motorRightRpmCurr, rightnoise);

  float left_cm  = leftrpm  * cmPerRound/60.0 * Timer.simTimeStep;
  float right_cm = rightrpm * cmPerRound/60.0 * Timer.simTimeStep;

  double avg_cm  = (left_cm + right_cm) / 2.0;
  double wheel_theta = (left_cm - right_cm) / Motor.odometryWheelBaseCm;
  simOrientation = scalePI( simOrientation + wheel_theta );
  simX = simX + (avg_cm * cos(simOrientation)) ;
  simY = simY + (avg_cm * sin(simOrientation)) ;

  Motor.totalDistanceTraveled += fabs(avg_cm/100.0);

  Motor.odometryDistanceCmCurr += avg_cm;
  Motor.odometryThetaRadCurr = scalePI( Motor.odometryThetaRadCurr + wheel_theta );
}

void SimRobot::run(){
  if (!Timer.simStopped){
    RobotControl::run();
    move();
    if (!MotorMow.hasStopped()) Perimeter.setLawnMowed(Robot.simX, Robot.simY);
    Perimeter.draw();
    Robot.draw(Perimeter.imgWorld);
  }
  if (  (!Timer.simFast) || ((Timer.simFast) && (loopCounter % 100 == 0))  ){
    char key = cvWaitKey( 10 );
    switch (key){
      case 'f':
        Timer.simFast = !Timer.simFast;
        break;
      case 27:
        exit(0);
        break;
      case ' ':
        Timer.simStopped=!Timer.simStopped;
        break;
    }
  }
}

// draw robot on surface
void SimRobot::draw(cv::Mat &img){
  float r = Motor.odometryWheelBaseCm/2;
  circle( img, cv::Point( simX, simY), r, cv::Scalar( 0, 0, 0 ), 2, 8 );
  line( img, cv::Point(simX, simY),
       cv::Point(simX + r * cos(simOrientation), simY + r * sin(simOrientation)),
       cv::Scalar(0,0,0), 2, 8);
}


// ------------------------------------------




