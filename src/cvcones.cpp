/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include "cvcones.hpp"


CVCones::CVCones() :
m_coneMutex(),
m_stampMutex()
{

}

void CVCones::setCvCones(Eigen::MatrixXd cones){

  std::lock_guard<std::mutex> lockCone(m_coneMutex);
  m_cones.clear();
  for(uint32_t i = 0; i < cones.cols(); i++){
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i), cones(1,i),cones(2,i));
    Cone cone = Cone(localCone(0),localCone(1),(int)cones(3,i),1000);
    m_cones.push_back(cone);
  }
}
std::vector<Cone> CVCones::getCvCones(){

  std::lock_guard<std::mutex> lockCone(m_coneMutex);
  return m_cones;
}

void CVCones::setTimeStamp(cluon::data::TimeStamp currentFrameTime){

  std::lock_guard<std::mutex> lockStamp(m_stampMutex);
  m_lastTimeStamp = currentFrameTime;
}
cluon::data::TimeStamp CVCones::getTimeStamp(){

  std::lock_guard<std::mutex> lockStamp(m_stampMutex);
  return m_lastTimeStamp;
}

Eigen::Vector3d CVCones::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}
Eigen::Vector2d CVCones::transformConeToCoG(double angle, double distance){
  const double lidarDistToCoG = 1.5;
  double sign = angle/std::fabs(angle);
  angle = PI - std::fabs(angle*DEG2RAD); 
  double distanceNew = std::sqrt(lidarDistToCoG*lidarDistToCoG + distance*distance - 2*lidarDistToCoG*distance*std::cos(angle));
  double angleNew = std::asin((std::sin(angle)*distance)/distanceNew )*RAD2DEG; 
  Eigen::Vector2d transformed;
  transformed << angleNew*sign,distanceNew;

  return transformed;
}