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


#ifndef SIM_H
#define SIM_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "world.h"
#include "simrobot.h"
#include "particles.h"

using namespace std;
using namespace cv;

class SimRobot;
class World;


class SimPlot
{
  public:
    float vmin;
    float vmax;
    std::string name;
    cv::Scalar color;
    std::vector<float> values;
};


// simulation
class Sim
{
  public:
    int plotIdx;
    std::vector<SimPlot> simPlots;
    Mat imgBfieldRobot;
    cv::Mat imgPlots;
    float simTime; // seconds
    float timeStep; // seconds
    int stepCounter;
    World world;
	SimRobot robot;
	Particles filter;
    Sim();
    void step();
    void draw();
    void addPlot(int plotIdx, float value);
    void plot();
};



#endif
