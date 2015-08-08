/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau

  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)
*/

#include "sim.h"

//#include "simrobot.h"
//#include "world.h"



// simulation initialization
Sim::Sim(){
  stepCounter = 0;
  plotIdx = 0;
  imgBfieldRobot = Mat(140, 500, CV_8UC3, Scalar(0,0,0));
  timeStep = 0.2; // one simulation step (seconds)
  simTime = 0;
  // start random generator
  time_t t;
  time(&t);
  srand((unsigned int)t);
  // place robot onto world
  robot.isParticle = false;
  robot.orientation = -1.5*M_PI;
  robot.x = world.chgStationX; //+ 10;
  robot.y = world.chgStationY; // + 10;
  //robot.x = 100;
  //robot.y = 100;
  float steering_noise    = 0.05;  // robot steering noise sigma (rad units)
  float distance_noise    = 0.2; // distance sensor measurement noise sigma (10cm units)
  float measurement_noise = 0.5; // bfield sensor measurement noise sigma
  robot.set_noise(steering_noise, distance_noise, measurement_noise);
  filter.init(robot.x, robot.y, robot.orientation,
              steering_noise, distance_noise, measurement_noise, 2000);
  filter.reset();

  // plots
  SimPlot plot1;
  plot1.name = "bfield";
  plot1.color = cv::Scalar(255, 0, 0);
  simPlots.push_back(plot1);

  SimPlot plot2;
  plot2.name = "prob";
  plot2.color = cv::Scalar(0, 200, 0);
  simPlots.push_back(plot2);

  SimPlot plot3;
  plot3.name = "error";
  plot3.color = cv::Scalar(0, 0, 255);
  simPlots.push_back(plot3);

  imgPlots = cv::Mat::zeros( 60 * simPlots.size() + 3, 500, CV_8UC3 );
}


// plotting
void Sim::addPlot(int plotIdx, float value) {
  if (simPlots[plotIdx].values.size() > 500)
    simPlots[plotIdx].values.erase(simPlots[plotIdx].values.begin());
  simPlots[plotIdx].values.push_back(value);
}

void Sim::plot()
{
    int rows = 50;
    int height  = rows + 10;
    imgPlots.setTo(255);
    int ofs = 0;

    for (int idx=0; idx < simPlots.size(); idx++){
      std::vector<float> &vals = simPlots[idx].values;
      cv::line(imgPlots,
                 cv::Point(0, ofs),
                 cv::Point(499, ofs),
                 cv::Scalar(0, 0, 0), 1);
      if (vals.size() != 0) {
        float newmax = *( std::max_element(vals.begin(), vals.end()) );
        float newmin = *( std::min_element(vals.begin(), vals.end()) );
        float newminmax = max( abs(newmax), abs(newmin) );
        simPlots[idx].vmin = min(simPlots[idx].vmin, -newminmax);
        simPlots[idx].vmax = max(simPlots[idx].vmax, newminmax);
        float vmax = newmax; // simPlots[idx].vmax;
        float vmin = newmin; //simPlots[idx].vmin;
        //float scale = 1./ceil(vmax - vmin);
        float scale = 1./(vmax - vmin);
        float bias = vmin;

        cv::line(imgPlots,
                 cv::Point(0, rows +1 - (0 - bias)*scale*rows + ofs),
                 cv::Point(499, rows +1 - (0 - bias)*scale*rows + ofs),
                 cv::Scalar(227, 227, 227), 1);

        for (int i = 0; i < min(500, (int)vals.size()) -1; i++){
          cv::line(imgPlots,
                 cv::Point(i, rows +1 - (vals[i] - bias)*scale*rows + ofs),
                 cv::Point(i+1, rows +1 - (vals[i+1] - bias)*scale*rows + ofs),
                 simPlots[idx].color, 2);
        }
        char buf[64];
        sprintf(buf, "%.3f", vmin);
        putText(imgPlots, std::string(buf), cv::Point(10,ofs+height-5), cv::FONT_HERSHEY_PLAIN, 1, simPlots[idx].color );
        sprintf(buf, "%.3f", vmax);
        putText(imgPlots, std::string(buf), cv::Point(10,ofs+14), cv::FONT_HERSHEY_PLAIN, 1, simPlots[idx].color );
      }
      cv::Mat roi(imgPlots(cv::Rect(0,ofs+height/2-10,140,16)));
      roi.setTo(240);
      putText(imgPlots, simPlots[idx].name, cv::Point(0,ofs+height/2+2), cv::FONT_HERSHEY_PLAIN, 1, simPlots[idx].color );
      ofs += height;
    }
    imshow("plots", imgPlots);
}

// simulation step
void Sim::step(){
  //printf("stateTime=%1.4f\n", stateTime);

  // simulate robot movement
  robot.move(*this, robot.orientation, robot.speed*10 * timeStep);
  filter.move(*this, robot.orientation, robot.speed*10 * timeStep);
  world.setLawnMowed(robot.x, robot.y);

  robot.sense(*this);
  filter.sense(*this, robot.bfieldStrength);

  // run robot controller
  robot.control(*this, timeStep);

  // simulation time
  simTime += timeStep;

  if ((stepCounter % 100) == 0){
    printf("time=%5.1fs  orient=%3.1f  laneHeading=%3.1f  laneCounter=%d  distChg=%3.1fm  totalDist=%3.1fm  mapsz=%d\n",
           simTime,
           robot.orientation/M_PI*180.0,
           robot.laneHeading/M_PI*180.0,
           robot.laneCounter,
           robot.distanceToChgStation/10,
           robot.totalDistance/10,
           robot.perimeterOutline.size() );
           //filter.overall_measurement_prob);
  }
  if (filter.overall_measurement_prob < 400.4) filter.reset();
  stepCounter++;
}


// draw world, robot, particles etc.
void Sim::draw(){
  world.draw();
  filter.draw(world.imgWorld);

  float x,y,theta;
  filter.get_position(x,y,theta);
  filter.drawCenter(world.imgWorld, x,y,theta);

  robot.draw(world.imgWorld);
  robot.drawMap(world);

  // plot robot bfield sensor
  float bfieldStrength = max(-1.0f, min(24.0f, robot.bfieldStrength));
  float error = distance( x,y, robot.x, robot.y );
  addPlot(0, bfieldStrength);
  addPlot(1, filter.overall_measurement_prob);
  addPlot(2, error/10);
  plot();
}


