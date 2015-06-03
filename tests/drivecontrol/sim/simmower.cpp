#include "simmower.h"


SimLED LED;
SimMotor Motor;
SimSettings Settings;
SimPerimeter Perimeter;
SimRobot Robot;


// ------------------------------------------

void SimSettings::setup(){
  Motor.motorLeftSwapDir = false;
  Motor.motorRightSwapDir = false;
  Motor.motorSpeedMaxRpm = 10;
  Motor.enableStallDetection = false;
  Motor.enableErrorDetection = false;
  Motor.odometryTicksPerRevolution = 1060;   // encoder ticks per one full resolution
  Motor.odometryTicksPerCm = 13.49;    // encoder ticks per cm
  Motor.odometryWheelBaseCm = 36;    // wheel-to-wheel distance (cm)
  Motor.motorLeftPID.Kp       = 0.1;    // PID speed controller
  Motor.motorLeftPID.Ki       = 0.01;
  Motor.motorLeftPID.Kd       = 0.01;
  Perimeter.enable = false;
}

// ------------------------------------------

void SimMotor::setDriverPWM(int leftMotorPWM, int rightMotorPWM){
}

// ------------------------------------------

SimPerimeter::SimPerimeter(){
  drawMowedLawn = true;
  memset(lawnMowStatus, 0, sizeof lawnMowStatus);
  //printf("%d\n", sizeof bfield);
  memset(bfield, 0, sizeof bfield);
  imgBfield = cv::Mat(WORLD_SIZE_Y, WORLD_SIZE_X, CV_8UC3, cv::Scalar(0,0,0));
  imgWorld = cv::Mat(WORLD_SIZE_Y, WORLD_SIZE_X, CV_8UC3, cv::Scalar(0,0,0));

  // perimeter lines coordinates (cm)
  std::vector<point_t> list;
  list.push_back( (point_t) {30, 35 } );
  list.push_back( (point_t) {50, 15 } );
  list.push_back( (point_t) {400, 40 } );
  list.push_back( (point_t) {410, 50 } );
  list.push_back( (point_t) {420, 90 } );
  list.push_back( (point_t) {350, 160 } );
  list.push_back( (point_t) {320, 190 } );
  list.push_back( (point_t) {210, 250 } );
  list.push_back( (point_t) {40, 300 } );
  list.push_back( (point_t) {20, 290 } );
  list.push_back( (point_t) {30, 230 } );

  chgStationX = 35;
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
          if (c == 0){
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
      if (bfield[y][x] > 0){
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
  circle( imgWorld, cv::Point( chgStationX, chgStationY), 10, cv::Scalar( 0, 255, 255 ), -1, 8 );
}

// approximate circle pattern
// 010
// 111
// 010
// x,y: cm
void SimPerimeter::setLawnMowed(int x, int y){
  if (  (x <= 5) || (x >= WORLD_SIZE_X-5 ) || (y <= 5) || (y >= WORLD_SIZE_Y-5 )  )return;
  for (int i=-2; i <= 2; i++){
    for (int j=-2; j <= 2; j++){
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


// ------------------------------------------

// initializes robot
SimRobot::SimRobot(){
  distanceToChgStation = 0;
  totalDistance = 0;

  x = 50;
  y = 50;
  orientation = 0;
  //leftMotorSpeed = 30;
  //rightMotorSpeed = 5;

  steering_noise    = 0.0;
  distance_noise    = 0.0;
  measurement_noise = 0.0;
  motor_noise = 10;

  timeStep = 0.1; // one simulation step (seconds)
}

void SimRobot::move(){
  float cmPerRound = Motor.odometryTicksPerRevolution / Motor.odometryTicksPerCm;

  // apply noise
  // gauss(mean, std)
  float leftSpeedNoise  = gauss(Motor.motorLeftSpeedRpmSet, motor_noise);
  float rightSpeedNoise = gauss(Motor.motorRightSpeedRpmSet, motor_noise);

  float left_cm = leftSpeedNoise * cmPerRound/60.0 * timeStep;
  float right_cm = rightSpeedNoise * cmPerRound/60.0 * timeStep;

  double avg_cm  = (left_cm + right_cm) / 2.0;
  double wheel_theta = (left_cm - right_cm) / Motor.odometryWheelBaseCm;
  orientation = scalePI( orientation + wheel_theta );
  x = x + (avg_cm * cos(orientation)) ;
  y = y + (avg_cm * sin(orientation)) ;

  totalDistance += fabs(avg_cm/100.0);
}

void SimRobot::run(){
  RobotControl::run();
  move();
  Perimeter.draw();
  Robot.draw(Perimeter.imgWorld);
  char key = cvWaitKey( 10 );
  if (key == 27) exit(0);
}

// draw robot on surface
void SimRobot::draw(cv::Mat &img){
  float r = Motor.odometryWheelBaseCm/2;
  circle( img, cv::Point( x, y), r, cv::Scalar( 0, 0, 0 ), 2, 8 );
  line( img, cv::Point(x, y), cv::Point(x + r * cos(orientation), y + r * sin(orientation)), cv::Scalar(0,0,0), 2, 8);
}


// ------------------------------------------




