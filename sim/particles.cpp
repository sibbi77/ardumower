#include "particles.h"
#include "simrobot.h"
#include "sim.h"
#include <vector>
#include <algorithm>

using namespace std;

Particles::Particles(){
}

void Particles::init(float x, float y, float theta,
              float steering_noise, float distance_noise, float measurement_noise, int N)
{
    //	initializes particle set with given initial position
    this->N = N;
    this->steering_noise = steering_noise;
    this->distance_noise = distance_noise;
    this->measurement_noise = measurement_noise;

    for (int i=0; i < N; i++){
       SimRobot r;
       r.set(x, y, theta);
       r.set_noise(steering_noise, distance_noise, measurement_noise);
       data.push_back(r);
    }
}

void Particles::reset(){
  for (int i=0; i < N; i++){
    data[i].x = random() * WORLD_SIZE_X;
    data[i].y = random() * WORLD_SIZE_Y;
    //printf("%3f, %3f\n", data[i].x, data[i].y);
  }
  overall_measurement_prob = 1000;
}

// extract position from a particle set
void Particles::get_position(float &x, float &y, float &orientation){
  x=0;
  y=0;
  orientation=0;
  for (int i=0; i < N; i++){
    x += data[i].x;
    y += data[i].y;
    // orientation is tricky because it is cyclic. By normalizing
    // around the first particle we are somewhat more robust to
    // the 0=2pi problem
    orientation += ( fmod((data[i].orientation
                    - data[0].orientation + M_PI) , (2.0 * M_PI))
                    + data[0].orientation - M_PI);
  }
  x/=N;
  y/=N;
  orientation/=N;
}


// motion of the particles
void Particles::move(Sim &sim, float course, float speed){
  //vector <SimRobot>newdata;
  for (int i=0; i < N; i++){
    //SimRobot r = data[i].move(world, steer, speed);
    data[i].move(sim, course, speed);
    //newdata.push_back(r);
  }
  //data = newdata;
}


// sensing and resampling
void Particles::sense(Sim &sim, float measurement){
  vector<float>w;
  float new_overall_measurement_prob=0;
  for (int i=0; i < N; i++){
    float measurement_prob = data[i].measurement_prob(sim, measurement);
    //new_overall_measurement_prob = max(new_overall_measurement_prob, measurement_prob);
    new_overall_measurement_prob += measurement_prob;
    w.push_back(measurement_prob);
  }
  overall_measurement_prob = 0.95 * overall_measurement_prob + 0.05 * new_overall_measurement_prob;
  // resampling (careful, this is using shallow copy)
  vector<SimRobot>p3;
  int index = floor(random() * N);
  float beta = 0.0;
  float mw = *max_element(w.begin(), w.end());
  for (int i=0; i < N; i++){
    //beta += random() * 2.0 * mw;
    beta += 1.0 * mw;
    while (beta > w[index]){
      beta -= w[index];
      index = (index + 1) % N;
    }
    p3.push_back(data[index]);
  }
  data = p3;
}

// draw particle set
void Particles::draw(Mat &img){
  for (int i =0; i < N; i++){
    data[i].draw(img, true);
  }
}

// draw particle set center
void Particles::drawCenter(Mat &img, float x, float y, float theta){
  float length = 7;
  circle( img, Point( x, y), length, Scalar( 255, 0, 0), 2, 8 );
  line( img, Point(x, y), Point(x + 15 * cos(theta), y + length * sin(theta)), Scalar(255,0,0), 2, 8);
}
