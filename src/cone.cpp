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

#include "cone.hpp"

Cone::Cone(double x, double y,int type,int id):
  m_x()
, m_y()
, m_type()
, m_id()
{
  m_meanX = x;
  m_meanY = y;
  m_mapX = x;
  m_mapY = y;

  m_x = x;
  m_y = y;
  m_type = type;
  m_id = id;
}

opendlv::logic::perception::ObjectDirection Cone::getDirection(Eigen::Vector3d pose){
  double x = m_x-pose(0);
  double y = m_y-pose(1);
  double newX = x*cos(-pose(2))-y*sin(-pose(2));
  double newY = x*sin(-pose(2))+y*cos(-pose(2));
  //double heading = pose(2)*static_cast<double>(1/RAD2DEG);
  double azimuthAngle = atan2(newY,newX)*static_cast<double>(RAD2DEG);
  //azimuthAngle = azimuthAngle-heading;
  opendlv::logic::perception::ObjectDirection direction;
  direction.zenithAngle(0);
  direction.azimuthAngle(static_cast<float>(azimuthAngle));
  return direction;
}

opendlv::logic::perception::ObjectDistance Cone::getDistance(Eigen::Vector3d pose){
  double x = m_x-pose(0);
  double y = m_y-pose(1);
  double distance = sqrt(x*x+y*y);
  opendlv::logic::perception::ObjectDistance msgDistance;
  msgDistance.distance(static_cast<float>(distance));
  return msgDistance;
}
double Cone::getOptX(){

  return m_optX;
}
double Cone::getOptY(){

  return m_optY;
}
void Cone::setOptX(double x){
  m_optX = x;
  m_x = x;
}
void Cone::setOptY(double y){
  m_optY = y;
  m_y = y;
}
double Cone::getMeanX(){
  return m_meanX;
}

double Cone::getMeanY(){
  return m_meanY;
}
double Cone::getX(){
  return m_x;
}

double Cone::getY(){
  return m_y;
}
int Cone::getType(){
  return m_type;
}

int Cone::getId(){
  return m_id;
}

void Cone::setType(int type){
  m_type = type;
}

void Cone::setId(int id){
  m_id = id;
}

double Cone::getMapX(){
  return m_mapX;
}

double Cone::getMapY(){
  return m_mapY;
}

bool Cone::isSeen(){
  return m_seen;
}

void Cone::addObservation(Eigen::Vector3d localObservation,Eigen::Vector3d globalObservation,int i){
  if(!m_seen){
    m_seen = true;
  }
  Eigen::Vector2d newLocalObservation;
  newLocalObservation << localObservation(0),localObservation(1);
  m_localObserved.push_back(newLocalObservation);

  Eigen::Vector2d newGlobalObservation;
  newGlobalObservation << globalObservation(0),globalObservation(1);
  m_observed.push_back(newGlobalObservation);

  m_connectedPoses.push_back(i);
  calculateMean();
}

uint32_t Cone::getObservations(){
  return m_observed.size();
}

Eigen::Vector2d Cone::getLocalConeObservation(int i){

  return m_localObserved[i];
}
Eigen::Vector2d Cone::getGlobalConeObservation(int i){

  return m_observed[i];
}

void Cone::calculateMean(){
  uint32_t observations = m_observed.size();
  double x = 0;
  double y = 0;
  if(observations > 1){
    for(uint32_t i = 0; i < observations; i++){
      x += m_observed[i](0);
      y += m_observed[i](1);
    }
    m_meanX = x/observations;
    m_meanY = y/observations;
    if(!m_optimizedState){
      m_x = m_meanX;
      m_y = m_meanY;
    }
  }
}

Eigen::Vector2d Cone::getCovariance(){

  uint32_t observations = m_observed.size();
  if(observations > 1){
    double varX = 0;
    double varY = 0;

    for(uint32_t i = 0; i < observations; i++){
      varX += (m_observed[i](0) - m_meanX)*(m_observed[i](0) - m_meanX);
      varY += (m_observed[i](1) - m_meanY)*(m_observed[i](1) - m_meanY);
    }  
    varX = varX/observations;
    varY = varY/observations;
    Eigen::Vector2d covVec;
    covVec << varX,varY;
    return covVec; 
  }else{
    Eigen::Vector2d covVec;
    covVec << 0.5,0.5;
    return covVec;
  }
}

std::vector<int> Cone::getConnectedPoses(){
  return m_connectedPoses;
}

void Cone::setOptimized(){
  m_optimizedState = true;
}

bool Cone::isOptimized(){
  return m_optimizedState;
}

void Cone::setValidState(bool state){
  m_validState = state;
}

bool Cone::isValid(){
  return m_validState;
}