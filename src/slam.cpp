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

#include <iostream>
#include "slam.hpp"
#include "WGS84toCartesian.hpp"

Slam::Slam(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4) :
  od4(a_od4)
, m_optimizer()
, m_lastTimeStamp()
, m_coneCollector()
, m_lastObjectId()
, m_coneMutex()
, m_sensorMutex()
, m_mapMutex()
, m_optimizerMutex()
, m_yawMutex()
, m_groundSpeedMutex()
, m_stateMachineMutex()
, m_odometryData()
, m_gpsReference()
, m_map()
, m_keyframeTimeStamp(cluon::time::now())
, m_newFrame()
, m_sendPose()
, m_sendMutex()
, m_localization()
{
  setupOptimizer();
  setUp(commandlineArguments);
  m_coneCollector = Eigen::MatrixXd::Zero(4,100);
  m_lastObjectId = 0;
  m_odometryData << 0,0,0;
  m_sendPose << 0,0,0;
  m_newFrame = true;
}

void Slam::setupOptimizer(){

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  m_optimizer.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //m_optimizer.setVerbose(true);
}

void Slam::nextSplitPose(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  if(data.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID()){
    auto position = cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(std::move(data));

    double longitude = position.longitude();
    double latitude = position.latitude();

    //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

    std::array<double,2> WGS84ReadingTemp;

    WGS84ReadingTemp[0] = latitude;
    WGS84ReadingTemp[1] = longitude;

    std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
    //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
    //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

    m_odometryData(0) =  WGS84Reading[0];
    m_odometryData(1) =  WGS84Reading[1];
  }
  else if(data.dataType() == opendlv::proxy::GeodeticHeadingReading::ID()){
    auto message = cluon::extractMessage<opendlv::proxy::GeodeticHeadingReading>(std::move(data));
    double heading = message.northHeading();
    heading = heading-PI;
    heading = (heading > PI)?(heading-2*PI):(heading);
    heading = (heading < -PI)?(heading+2*PI):(heading);
    m_odometryData(2) = heading;
    //std::cout << "head: " << heading << std::endl;
  }
}

void Slam::nextPose(cluon::data::Envelope data){
    //#########################Recieve Odometry##################################
  
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  m_geolocationReceivedTime = data.sampleTimeStamp();
  auto odometry = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(data));

  double newX = odometry.longitude()*cos(m_headingOffset)-odometry.latitude()*sin(m_headingOffset);
  double newY = odometry.longitude()*sin(m_headingOffset)+odometry.latitude()*cos(m_headingOffset);
  double x = newX+m_xOffset+m_xError/m_errorCounter;
  double y = newY+m_yOffset+m_yError/m_errorCounter;

  //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

  //std::array<double,2> WGS84ReadingTemp;

  //WGS84ReadingTemp[0] = latitude;
  //WGS84ReadingTemp[1] = longitude;

  //std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
  double heading = odometry.heading()+m_headingOffset;
  heading = (heading > PI)?(heading-2*PI):(heading);
  heading = (heading < -PI)?(heading+2*PI):(heading);

  m_odometryData << x,
                    y,
                    heading;
  //std::cout << "head: " << odometry.heading() << std::endl;                   
}

void Slam::nextYawRate(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(data));
  m_yawRate = yawRate.angularVelocityZ();
   m_yawReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}
void Slam::nextGroundSpeed(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
  auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(data));
  m_groundSpeed = groundSpeed.groundSpeed();
   m_groundSpeedReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}

void Slam::recieveCombinedMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame){
  m_lastTimeStamp = currentFrameTime;
  if(isKeyframe() || true){
    Eigen::MatrixXd cones = Eigen::MatrixXd::Zero(4,currentFrame.size());
    std::map<int,ConePackage>::iterator it;
    int coneIndex = 0;
    it =currentFrame.begin();
    while(it != currentFrame.end()){
      auto direction = std::get<0>(it->second);
      auto distance = std::get<1>(it->second);
      auto type = std::get<2>(it->second);
      cones(0,coneIndex) = direction.azimuthAngle();
      cones(1,coneIndex) = direction.zenithAngle();
      cones(2,coneIndex) = distance.distance();
      cones(3,coneIndex) = type.type();
      coneIndex++;
      it++;
    }
    performSLAM(cones);
  }
}

bool Slam::isKeyframe(){
  cluon::data::TimeStamp startTime = cluon::time::now();
  double timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_keyframeTimeStamp,startTime)))/1000;
  //std::cout << "Time ellapsed is: " << timeElapsed << std::endl;
  if(timeElapsed>m_timeBetweenKeyframes){//Keyframe candidate is based on time difference from last keyframe
    m_keyframeTimeStamp = startTime;
    return true;
  }
  return false;
}


void Slam::performSLAM(Eigen::MatrixXd cones){
  
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  Eigen::Vector3d pose;
  if(!m_initialized){
    Initialize(cones,pose);
  }
  if(m_readyStateMachine && m_readyState)
  {
    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    pose = m_odometryData;
    m_poses.push_back(pose);  
  }else{
    pose = m_sendPose;
    m_slamStartX = pose(0);
    m_slamStartY = pose(1);
    m_slamStartHeading = pose(2);
    m_poses.push_back(pose);

  }
  if(m_initialized){
    std::vector<std::pair<int,Eigen::Vector3d>> matchedCones = matchCones(cones,pose);
    std::cout << "Cones matched: " << matchedCones.size() << std::endl; 
    if(localizable(matchedCones)){
      pose = localizer(pose,matchedCones);
      if(m_poseId - m_lastOptimizedPoseId > 100){
        //optimizeGraph();
        m_lastOptimizedPoseId = m_poseId;
        std::cout << "Optimized" << std::endl;
      }
      if(checkOffset()){
        sendPose();
        sendCones();
        return;
      }
    }
    if(m_readyState){
      m_sendPose = m_odometryData;
    }
    sendCones();
  }
}

void Slam::Initialize(Eigen::MatrixXd cones,Eigen::Vector3d pose){
  std::vector<int> matchDistance;
  std::vector<std::pair<int,Eigen::Vector3d>> matchedCones;
  for(uint32_t j = 0; j<4; j++){
    int minDistidx = 1000;
    double minDistance = 1000000;
    for(uint32_t i = 0; i<cones.cols();i++){
      if(std::find(matchDistance.begin(), matchDistance.end(), i) != matchDistance.end()){
        continue;
      }
      double distance = cones(2,i);
      if(distance<minDistance){
        minDistance = distance;
        minDistidx = i;
      }
    }
    if(minDistance < 20){
        matchDistance.push_back(minDistidx);
    }
  }
  int nLeft = 0;
  int nRight = 0;
  if(matchDistance.size() < 4){
    return;
  }

  if(cones(0,matchDistance[0])>0){
     nLeft++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[0]),cones(1,matchDistance[0]),cones(2,matchDistance[0]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(0,obs);
     matchedCones.push_back(matchedObs);
  }
  else{
     nRight++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[0]),cones(1,matchDistance[0]),cones(2,matchDistance[0]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(1,obs);
     matchedCones.push_back(matchedObs);
  }  
  if(cones(0,matchDistance[1])>0){
     nLeft++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[1]),cones(1,matchDistance[1]),cones(2,matchDistance[1]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(0,obs);
     matchedCones.push_back(matchedObs);
  }
  else{
     nRight++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[1]),cones(1,matchDistance[1]),cones(2,matchDistance[1]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(1,obs);
     matchedCones.push_back(matchedObs);
  }
  if(cones(0,matchDistance[2])>0){
     nLeft++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[2]),cones(1,matchDistance[2]),cones(2,matchDistance[2]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(2,obs);
     matchedCones.push_back(matchedObs);
  }
  else{
     nRight++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[2]),cones(1,matchDistance[2]),cones(2,matchDistance[2]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(3,obs);
     matchedCones.push_back(matchedObs);
  }
  if(cones(0,matchDistance[3])>0){
     nLeft++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[3]),cones(1,matchDistance[3]),cones(2,matchDistance[3]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(2,obs);
     matchedCones.push_back(matchedObs);
  }
  else{
     nRight++;
     Eigen::Vector3d obs = Spherical2Cartesian(cones(0,matchDistance[3]),cones(1,matchDistance[3]),cones(2,matchDistance[3]));
     std::pair<int,Eigen::Vector3d> matchedObs = std::make_pair(3,obs);
     matchedCones.push_back(matchedObs);
  }
  if(nRight == 2 && nLeft == 2){
    Eigen::Vector3d localizedPose = localizer(pose,matchedCones);
    std::cout << "Start pose " << localizedPose << std::endl;
    m_slamStartX = pose(0) - localizedPose(0);
    m_slamStartY = pose(1) - localizedPose(1);
    m_slamStartHeading = pose(2) - localizedPose(2);
    m_initialized = true;
  }
}

int Slam::updateCurrentCone(std::vector<std::pair<int,Eigen::Vector3d>> matchedConeVector){
  double minX = 10;
  uint32_t minIndex = 0;
  for(uint32_t i = 0; i<matchedConeVector.size(); i++){
    Eigen::Vector3d localCone = std::get<1>(matchedConeVector[i]);
    if(localCone(0)<minX){
      minX = localCone(0);
      minIndex = i;
    }
  }
  if(minX<10){
    return std::get<0>(matchedConeVector[minIndex]);
  }
  else{
    return m_currentConeIndex;
  }
}

int Slam::updateCurrentCone(Eigen::Vector3d pose,uint32_t currentConeIndex, uint32_t remainingIter){
  //currentConeIndex=(currentConeIndex<m_map.size())?(currentConeIndex):(currentConeIndex-m_map.size());
  int pathIndex = m_skidPadList[currentConeIndex];
  Cone currentCone = m_map[pathIndex];
  remainingIter = remainingIter-1;
  auto distance = currentCone.getDistance(pose);
  auto direction = currentCone.getDirection(pose);
  //std::cout << "hej" << std::endl;
  if(remainingIter == 0){
    return currentConeIndex-1;
  }
  if(distance.distance() < 10.0f && fabs(direction.azimuthAngle())>80.0f){
    if(distance.distance()>m_behindThreshold){
      currentConeIndex = updateCurrentCone(pose,currentConeIndex+1,remainingIter);
    }
  }
  return currentConeIndex;
}

std::vector<std::pair<int,Eigen::Vector3d>> Slam::matchCones(Eigen::MatrixXd cones, Eigen::Vector3d &pose){
  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //Find match in conelist
  std::vector<int> coneIndices;
  int mapSize = m_map.size();
  for(int i = 0; i<mapSize; i++){
    auto distance = m_map[i].getDistance(pose);
    auto direction = m_map[i].getDirection(pose);
    if(distance.distance()<10 && fabs(direction.azimuthAngle())<80){
      coneIndices.push_back(i);
    }
    else if(distance.distance()<2){
      coneIndices.push_back(i);
    }
  }
  std::pair<double,std::vector<int>> scoredMatch = evaluatePose(cones,pose,coneIndices);
  double avgDistance = std::get<0>(scoredMatch)/cones.cols();
  std::cout << "avgDistance: " << avgDistance << std::endl;
  if(avgDistance < 1.5){
    return filterMatch(cones,pose,scoredMatch);
  }
  double headingCone = PI/4;
  double headingStep = headingCone/50;
  pose(2) = pose(2) - headingCone/2;
  std::vector<std::pair<double,std::vector<int>>> matchVector;
  double minDistance = 1000;
  int minDistidx = 0;
  double bestPose = pose(2);
  for(int i = 0; i<50;i++){
    scoredMatch = evaluatePose(cones,pose,coneIndices);
    matchVector.push_back(scoredMatch);
    double distance = std::get<0>(scoredMatch);
    if(distance<minDistance){
      minDistance = distance;
      minDistidx = i;
      bestPose = pose(2);
    }
    pose(2) = pose(2) + headingStep;
  }
  bestPose = (bestPose > PI)?(bestPose-2*PI):(bestPose);
  bestPose = (bestPose < -PI)?(bestPose+2*PI):(bestPose);
  pose(2) = bestPose;
  /*bestPose = pose(0);
  double distanceStep = 6/50;
  pose(0) = pose(0)-3;
  for(int i  = 0; i< 50; i++){
    scoredMatch = evaluatePose(cones,pose,coneIndices);
    matchVector.push_back(scoredMatch);
    double distance = std::get<0>(scoredMatch);
    if(distance<minDistance){
      minDistance = distance;
      minDistidx = i;
      bestPose = pose(0);
    }
    pose(0) = pose(0) + distanceStep;
  }
  pose(0) = bestPose;
  bestPose = pose(1);
  pose(1) = pose(1)-3;
  for(int i  = 0; i< 50; i++){
    scoredMatch = evaluatePose(cones,pose,coneIndices);
    matchVector.push_back(scoredMatch);
    double distance = std::get<0>(scoredMatch);
    if(distance<minDistance){
      minDistance = distance;
      minDistidx = i;
      bestPose = pose(1);
    }
    pose(1) = pose(1) + distanceStep;
  }
  pose(1) = bestPose;*/
  //std::cout << "bestHeading " << bestPose << std::endl;
  std::vector<std::pair<int,Eigen::Vector3d>> matchedCones = filterMatch(cones,pose,matchVector[minDistidx]);
  return matchedCones;
}

std::pair<double,std::vector<int>> Slam::evaluatePose(Eigen::MatrixXd cones, Eigen::Vector3d pose, std::vector<int> coneIndices){
  double sumDistance = 0;
  std::vector<int> matchedCone;
  for(int i = 0; i<cones.cols();i++){
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i));
    double minDistance = 100;
    int minIndex = 100;
    for(uint32_t j = 0; j<coneIndices.size();j++){
      double distance = coneToMeasurementDistance(globalCone,m_map[coneIndices[j]]);
      if(distance<minDistance){
        minDistance = distance;
        minIndex = coneIndices[j];
      }
    }
    sumDistance = sumDistance+minDistance;
    matchedCone.push_back(minIndex);
  }
  std::pair<double,std::vector<int>> scoredMatches = std::make_pair(sumDistance,matchedCone);
  return scoredMatches;
}

double Slam::coneToMeasurementDistance(Eigen::Vector3d globalMeasurement,Cone cone){
  double distance = std::sqrt((globalMeasurement(0)-cone.getX())*(globalMeasurement(0)-cone.getX()) + (globalMeasurement(1)-cone.getY())*(globalMeasurement(1)-cone.getY()));
  return distance;
}

std::vector<std::pair<int,Eigen::Vector3d>> Slam::filterMatch(Eigen::MatrixXd cones, Eigen::Vector3d pose,std::pair<double,std::vector<int>> matchedCones){
  std::vector<int> matchedIndices = std::get<1>(matchedCones);
  std::vector<std::pair<int,Eigen::Vector3d>> matchedConeVector;
  for(int i = 0; i<cones.cols();i++){
    if(fabs(cones(0,i))>90){
      continue;
    }
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i));
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i),cones(1,i),cones(2,i));
    double distance = coneToMeasurementDistance(globalCone,m_map[matchedIndices[i]]);
    if(distance<1.0){
      std::pair<int,Eigen::Vector3d> match = std::make_pair(matchedIndices[i],localCone);
      matchedConeVector.push_back(match);
    }
  }
  m_currentConeIndex = updateCurrentCone(pose,m_currentConeIndex,10);
  //m_currentConeIndex = updateCurrentCone(matchedConeVector);
  return matchedConeVector;
}

bool Slam::localizable(std::vector<std::pair<int,Eigen::Vector3d>> matchedCones){
  return matchedCones.size()>1;
}

Eigen::Vector3d Slam::localizer(Eigen::Vector3d pose, std::vector<std::pair<int,Eigen::Vector3d>> matchedCones){

  g2o::SparseOptimizer localGraph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  localGraph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //localGraph.setVerbose(true);

  if(matchedCones.size() > 1){  
    //Create graph
    //Add pose vertex
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(1000);
    poseVertex->setEstimate(pose);
    localGraph.addVertex(poseVertex);

    //Add cone vertex
    Eigen::Vector2d coneMeanXY;
    for(uint32_t i = 0; i < matchedCones.size(); i++){
      Eigen::Vector3d localObs = std::get<1>(matchedCones[i]);
      int index = std::get<0>(matchedCones[i]);
      g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
      coneMeanXY << m_map[index].getX(),m_map[index].getY();
      g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
      coneVertex->setId(i);
      coneVertex->setEstimate(coneMeanXY);
      coneVertex->setFixed(true);      
      localGraph.addVertex(coneVertex);

      //Add edge between pose and cone i
      Eigen::Vector2d xyMeasurement;
      coneMeasurement->vertices()[0] = localGraph.vertex(1000);
      coneMeasurement->vertices()[1] = localGraph.vertex(i);
      xyMeasurement << localObs(0),localObs(1);
      coneMeasurement->setMeasurement(xyMeasurement);

      //Eigen::Vector2d covXY = m_coneList[matchedConeIndex[i]].getCovariance();
      Eigen::Matrix2d informationMatrix;
      informationMatrix << 1/0.1,0,
                              0,1/0.1;
      coneMeasurement->setInformation(informationMatrix);
      localGraph.addEdge(coneMeasurement);
    }
    
    localGraph.initializeOptimization();
    localGraph.optimize(10); //Add config for amount of iterations??
    //std::cout << "Optimizing done." << std::endl;

    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(localGraph.vertex(1000));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    std::lock_guard<std::mutex> lockMap(m_mapMutex);
    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    for(uint32_t i = 0; i<matchedCones.size(); i++){
      Eigen::Vector3d localObs = std::get<1>(matchedCones[i]);
      int index = std::get<0>(matchedCones[i]);
      double distance = std::sqrt(localObs(0)*localObs(0)+localObs(1)*localObs(1));
      if(distance<m_coneMappingThreshold){
        m_map[index].addObservation(localObs,updatedPose,m_poseId);
      }
    }
    //m_poses.push_back(updatedPose);
    m_poseId++;
    {
      std::lock_guard<std::mutex> lockSend(m_sendMutex); 
      double xError = updatedPose(0) - m_odometryData(0);
      double yError = updatedPose(1) - m_odometryData(1);
      double headingError = updatedPose(2) - m_odometryData(2);
      bool goodError = (fabs(xError)<2 && fabs(yError)<2 && fabs(headingError)<0.3);
      if(goodError && m_readyState){
        //m_xError = m_xError+xError;
        //m_yError = m_yError+yError;
        //m_headingError = m_headingError+headingError;
        //m_errorCounter++;
      }
      m_sendPose << updatedPose(0),updatedPose(1),updatedPose(2);
      std::cout << "pose: " << m_sendPose(0) << " : " << m_sendPose(1) << " : " << m_sendPose(2) << std::endl;
    }

  }else{
    m_sendPose << pose(0),pose(1),pose(2);
  }
  return m_sendPose;
}

void Slam::optimizeGraph(){

  //Initialize graph
  g2o::SparseOptimizer Graph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  Graph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //essentialGraph.setVerbose(true);

  std::vector<int> posesToGraph;
  //Find cones of conespan and extract poses
  for(uint32_t i = 0; i < m_map.size(); i++){
    if(m_map[i].isSeen()){
      std::vector<int> currentConnectedPoses = m_map[i].getConnectedPoses();
      for(uint32_t j = 0; j < currentConnectedPoses.size(); j++){
        posesToGraph.push_back(currentConnectedPoses[j]);
      }      
    }
  }
  if(posesToGraph.size() > 0){
    uint32_t max = *std::max_element(posesToGraph.begin(), posesToGraph.end());
    uint32_t min = *std::min_element(posesToGraph.begin(), posesToGraph.end());
    //add poses to graph based on min and max
    for(uint32_t k = min; k < max+1; k++){

      //Add vertex
      g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
      poseVertex->setId(k);
      poseVertex->setEstimate(m_poses[k-1000]);

      Graph.addVertex(poseVertex);

      //Add edge
      if(k > min){
        g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;
        odometryEdge->vertices()[0] = Graph.vertex(k-1);
        odometryEdge->vertices()[1] = Graph.vertex(k);
        g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(Graph.vertex(k-1));
        g2o::SE2 prevPose = prevVertex->estimate();
        g2o::SE2 currentPose = g2o::SE2(m_poses[k-1000](0), m_poses[k-1000](1), m_poses[k-1000](2));
        g2o::SE2 measurement = prevPose.inverse()*currentPose;
        odometryEdge->setMeasurement(measurement);
        odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
        Graph.addEdge(odometryEdge);
      }
    }

    //Connect cones to poses
    for(uint32_t i = 0; i < m_map.size(); i++){
        if(m_map[i].isSeen()){
          Eigen::Vector2d coneMeanXY;
          if(!m_map[i].isOptimized()){
            m_map[i].calculateMean();
            coneMeanXY << m_map[i].getMeanX(),m_map[i].getMeanY();
          }else{
            coneMeanXY << m_map[i].getOptX(),m_map[i].getOptY();
          }

          g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
          coneVertex->setId(m_map[i].getId());
          coneVertex->setEstimate(coneMeanXY);
          Graph.addVertex(coneVertex);
          //Connect cones to POses
          g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
          std::vector<int> connectedPoses = m_map[i].getConnectedPoses();

          for(uint32_t j = 0; j < connectedPoses.size(); j++){
            Eigen::Vector2d xyMeasurement;
            xyMeasurement = getConeToPoseMeasurement(i,j);
            //std::cout << "x: " << xyMeasurement(0) << " y: " << xyMeasurement(1) << std::endl; 
            coneMeasurement->vertices()[0] = Graph.vertex(connectedPoses[j]);
            coneMeasurement->vertices()[1] = Graph.vertex(m_map[i].getId());
            coneMeasurement->setMeasurement(xyMeasurement);

            Eigen::Vector2d covXY = m_map[i].getCovariance();
            Eigen::Matrix2d informationMatrix;
            informationMatrix << 1/covXY(0),0,
                              0,1/covXY(1);
            coneMeasurement->setInformation(informationMatrix); //Placeholder value
            //std::cout << "cX: " << covXY(0) << " cY: " << covXY(1) << std::endl;
            Graph.addEdge(coneMeasurement);
          }
        }
      }
    g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(Graph.vertex(min));
    firstRobotPose->setFixed(true);

    /*g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(essentialGraph.vertex(graphIndexStart));
    firstCone->setFixed(true);*/
    //std::cout << "Optimizing" << std::endl;
    Graph.initializeOptimization();
    Graph.optimize(10); //Add config for amount of iterations??
    //std::cout << "Optimizing done." << std::endl;

    updateFromEssential(min, max, Graph);
  }
}

void Slam::updateFromEssential(uint32_t poseStart, uint32_t poseEnd, g2o::SparseOptimizer &essentialGraph){

  //Update pose vector

  for(uint32_t i = poseStart; i < poseEnd; i++){
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(essentialGraph.vertex(i));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    m_poses[i-1000] << updatedPose(0),updatedPose(1),updatedPose(2);
  }
  //Set optimized cone positions

   Eigen::Vector2d updatedConeXY;
   g2o::VertexPointXY* updatedConeVertex;

   for(uint32_t i = 0; i < m_map.size(); i++){
    if(m_map[i].isSeen()){
      updatedConeVertex = static_cast<g2o::VertexPointXY*>(essentialGraph.vertex(i));
      updatedConeXY = updatedConeVertex->estimate();
      m_map[i].setOptX(updatedConeXY(0));
      m_map[i].setOptY(updatedConeXY(1));
      m_map[i].setOptimized(); 
    }   
  }
}

Eigen::Vector3d Slam::updatePoseFromGraph(){

  g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(m_poseId-1));
  g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
  Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
  return updatedPose;
}

void Slam::addPosesToGraph(){

  for(uint32_t i = 0; i < m_poses.size(); i++){
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(i+1000);
    poseVertex->setEstimate(m_poses[i]);

    m_optimizer.addVertex(poseVertex);

    addOdometryMeasurement(m_poses[i],i);

    std::vector<int> poseVector;
    m_connectivityGraph.push_back(poseVector);
  }
}


void Slam::addOdometryMeasurement(Eigen::Vector3d pose, uint32_t i){
  if(i > 0){
    g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;

    odometryEdge->vertices()[0] = m_optimizer.vertex(i+999);
    odometryEdge->vertices()[1] = m_optimizer.vertex(i+1000);
    g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(i+999));
    g2o::SE2 prevPose = prevVertex->estimate();
    g2o::SE2 currentPose = g2o::SE2(pose(0), pose(1), pose(2));
    g2o::SE2 measurement = prevPose.inverse()*currentPose;
    odometryEdge->setMeasurement(measurement);
    odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
    m_optimizer.addEdge(odometryEdge);
  }
}


Eigen::Vector3d Slam::coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd cones){
  Eigen::Vector3d cone = Spherical2Cartesian(cones(0), cones(1), cones(2));
  //convert from local to global coordsystem
  double newX = cone(0)*cos(pose(2))-cone(1)*sin(pose(2));
  double newY = cone(0)*sin(pose(2))+cone(1)*cos(pose(2));
  cone(0) = newX+pose(0);
  cone(1) = newY+pose(1);
  cone(2) = cones(3);
  return cone;
}

Eigen::Vector2d Slam::transformConeToCoG(double angle, double distance, bool behindCar){
  const double lidarDistToCoG = 1.5;
  double sign = angle/std::fabs(angle);
  angle = PI - std::fabs(angle*DEG2RAD); 
  double distanceNew = std::sqrt(lidarDistToCoG*lidarDistToCoG + distance*distance - 2*lidarDistToCoG*distance*std::cos(angle));
  double angleNew = (std::sin(angle)*distance)/distanceNew; 
  if(behindCar){
    angleNew = std::asin(angleNew)+2*(PI/2-std::asin(angleNew));
    angleNew = angleNew*RAD2DEG;   
  }
  else{
      angleNew = std::asin(angleNew)*RAD2DEG;
  }
  Eigen::Vector2d transformed;
  transformed << angleNew*sign,distanceNew;
  return transformed;
}

void Slam::addConesToGraph(){
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    Eigen::Vector2d coneMeanXY;
    if(!m_coneList[i].isOptimized()){
      m_coneList[i].calculateMean();
      coneMeanXY << m_coneList[i].getMeanX(),m_coneList[i].getMeanY();
    }else{
      coneMeanXY << m_coneList[i].getOptX(),m_coneList[i].getOptY();
    }
    g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
    coneVertex->setId(m_coneList[i].getId());
    coneVertex->setEstimate(coneMeanXY);
    m_optimizer.addVertex(coneVertex);
    addConeMeasurements(i);
  }
}

void Slam::addConeMeasurements(int i){
  g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;

  std::vector<int> connectedPoses = m_coneList[i].getConnectedPoses();

  for(uint32_t j = 0; j < connectedPoses.size(); j++){
  Eigen::Vector2d xyMeasurement;
  xyMeasurement = getConeToPoseMeasurement(i,j);

  coneMeasurement->vertices()[0] = m_optimizer.vertex(connectedPoses[j]);
  coneMeasurement->vertices()[1] = m_optimizer.vertex(m_coneList[i].getId());
  coneMeasurement->setMeasurement(xyMeasurement);

  Eigen::Vector2d covXY = m_coneList[i].getCovariance();
  Eigen::Matrix2d informationMatrix;
  informationMatrix << 1/covXY(0),0,
                       0,1/covXY(1);
  coneMeasurement->setInformation(informationMatrix); //Placeholder value

  m_optimizer.addEdge(coneMeasurement);

  }

  //m_connectivityGraph[m_poseId-1001].push_back(cone.getId());
}

Eigen::Vector2d Slam::getConeToPoseMeasurement(int i,int j){
  
  Eigen::Vector2d cone;
  cone = m_map[i].getLocalConeObservation(j); 
  Eigen::Vector2d measurement;
  measurement << cone(0), cone(1);  

  return measurement;
}
Eigen::Vector2d Slam::getLocalConeToPoseMeasurement(Eigen::Vector3d pose, Eigen::Vector2d cone){
  
  Eigen::Vector2d measurement;
  measurement << cone(0)-pose(0), cone(1)-pose(1);  

  return measurement;
}
Eigen::Vector3d Slam::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  //azimuth = (azimuth > PI)?(azimuth-2*PI):(azimuth);
  //azimuth = (azimuth < -PI)?(azimuth+2*PI):(azimuth);
  double oldX = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  Eigen::Vector2d transformedCone = transformConeToCoG(azimuth,distance,oldX<-1.5);
  azimuth = transformedCone(0);
  distance = transformedCone(1);
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}

void Slam::sendCones()
{
  Eigen::Vector3d pose;
  {
    std::lock_guard<std::mutex> lockSend(m_sendMutex); 
    pose = m_sendPose;
  }//mapmutex too
  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = m_lastTimeStamp;
  for(uint32_t i = 0; i< m_conesPerPacket;i++){ //Iterate through the cones ahead of time the path planning recieves
    if(m_currentConeIndex+i<m_skidPadList.size()){
      int coneIndex = m_skidPadList[m_currentConeIndex+i];
      opendlv::logic::perception::ObjectDirection directionMsg = m_map[coneIndex].getDirection(pose); //Extract cone direction
      directionMsg.objectId(m_conesPerPacket-1-i);
      od4.send(directionMsg,sampleTime,m_senderStamp);
      opendlv::logic::perception::ObjectDistance distanceMsg = m_map[coneIndex].getDistance(pose); //Extract cone distance
      distanceMsg.objectId(m_conesPerPacket-1-i);
      od4.send(distanceMsg,sampleTime,m_senderStamp);
      opendlv::logic::perception::ObjectType typeMsg;
      if(m_currentConeIndex>m_skidPadList.size()-3){
        typeMsg.type(50);
      }
      else{
        typeMsg.type(m_typeList[m_currentConeIndex+i]); //Extract cone type
      }
      typeMsg.objectId(m_conesPerPacket-1-i);
      od4.send(typeMsg,sampleTime,m_senderStamp);
    }
  }
}

void Slam::sendPose(){
  opendlv::logic::sensation::Geolocation poseMessage;
  std::lock_guard<std::mutex> lockSend(m_sendMutex); 
  if(m_readyState){
    double x = m_sendPose(0)-m_xOffset;
    double y = m_sendPose(1)-m_yOffset;
    double heading = m_sendPose(2)-m_headingOffset;
    double newX = x*cos(-m_headingOffset)-y*sin(-m_headingOffset);
    double newY = x*sin(-m_headingOffset)+y*cos(-m_headingOffset);
    //std::array<double,2> cartesianPos;
    //cartesianPos[0] = m_sendPose(0);
    //cartesianPos[1] = m_sendPose(1);
    //std::array<double,2> sendGPS = wgs84::fromCartesian(m_gpsReference, cartesianPos);
    poseMessage.longitude(newX);
    poseMessage.latitude(newY);
    poseMessage.heading(static_cast<float>(heading));
    //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = m_geolocationReceivedTime;
    od4.send(poseMessage, sampleTime ,m_senderStamp);
  }
}

double Slam::distanceBetweenCones(Cone c1, Cone c2){
  c1.calculateMean();
  c2.calculateMean();
  double distance = std::sqrt( (c1.getMeanX()-c2.getMeanX())*(c1.getMeanX()-c2.getMeanX()) + (c1.getMeanY()-c2.getMeanY())*(c1.getMeanY()-c2.getMeanY()) );
  return distance;
}

double Slam::distanceBetweenConesOpt(Cone c1, Cone c2){
  double distance = std::sqrt( (c1.getOptX()-c2.getMeanX())*(c1.getOptX()-c2.getMeanX()) + (c1.getOptY()-c2.getMeanY())*(c1.getOptY()-c2.getMeanY()) );
  return distance;
}

void Slam::updateMap(uint32_t start, uint32_t end, bool updateToGlobal){
  for(uint32_t i = start; i < end; i++){

    if(updateToGlobal && m_coneList[i].isValid()){
      m_map.push_back(m_coneList[i]);
    }else{
      m_essentialMap.push_back(m_coneList[i]);
    }
  }
}

void Slam::filterMap(){

  //Filter on mean and optimized value
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    double distance = distanceBetweenConesOpt(m_coneList[i],m_coneList[i]);
    if(distance > m_newConeThreshold){
      m_coneList[i].setValidState(false);

    }
  }
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    for(uint32_t j = 0; j < m_coneList.size(); j++){
      if(i != j){
        double distance = std::sqrt( (m_coneList[i].getOptX() - m_coneList[j].getOptX() )*(m_coneList[i].getOptX() - m_coneList[j].getOptX()) + (m_coneList[i].getOptY() - m_coneList[j].getOptY())*(m_coneList[i].getOptY() - m_coneList[j].getOptY()) );

        if(distance < m_newConeThreshold && m_coneList[i].isValid() && m_coneList[j].isValid()){
          m_coneList[j].setValidState(false);
        }
      } 
    }
  }  


  //Check closest pose didstance
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    double closestPoseDistance = 10000;
    uint32_t closestPoseId = 0;
    for(uint32_t j = 0; j < m_poses.size(); j++){
      double distance = std::sqrt( (m_poses[j](0)-m_coneList[i].getOptX())*(m_poses[j](0)-m_coneList[i].getOptX()) + (m_poses[j](1)-m_coneList[i].getOptY())*(m_poses[j](1)-m_coneList[i].getOptY()) );
      if(distance < closestPoseDistance){
        closestPoseDistance = distance;
        closestPoseId = j;
      }

    }
    if(closestPoseDistance > 3){
      m_coneList[i].setValidState(false);
    }
    else{
      auto direction = m_coneList[i].getDirection(m_poses[closestPoseId]);
      if(direction.azimuthAngle()>0){
        m_coneList[i].setType(1);
      }
      else{
        m_coneList[i].setType(2);
      }
      
    }

  }
}

void Slam::setUp(std::map<std::string, std::string> configuration)
{

  m_timeDiffMilliseconds = static_cast<uint32_t>(std::stoi(configuration["gatheringTimeMs"]));
  m_newConeThreshold = static_cast<double>(std::stod(configuration["sameConeThreshold"]));
  m_gpsReference[0] = static_cast<double>(std::stod(configuration["refLatitude"]));
  m_gpsReference[1] = static_cast<double>(std::stod(configuration["refLongitude"]));
  m_timeBetweenKeyframes = static_cast<double>(std::stod(configuration["timeBetweenKeyframes"]));
  m_conesPerPacket = static_cast<int>(std::stoi(configuration["conesPerPacket"]));
  std::string mapFilePath = configuration["mapFilePath"];
  std::string pathFile = configuration["pathFilePath"];
  m_senderStamp = static_cast<int>(std::stoi(configuration["id"]));
  m_offsetLimit = std::stod(configuration["offsetDistanceLimit"]);
  m_offsetHeadingLimit = std::stod(configuration["offsetHeadingLimit"]);
  m_behindThreshold = std::stod(configuration["behindThreshold"]);
  loadMap(mapFilePath); 
  loadPath(pathFile);


}

void Slam::loadMap(std::string mapFilePath){
  std::ifstream mapStream (mapFilePath);
  std::string line;
  if (mapStream.is_open())
  {
    int counter1 = 0;
    while (getline(mapStream,line))
    {
      std::stringstream linestream(line);
      std::string value;
      int counter2 = 0;
      int type = 0;
      double x = 0;
      double y = 0;
      while(getline(linestream,value,','))
      {
        if(counter2 == 0){
          x = std::stod(value);
        }else if(counter2 == 1){
          y = std::stod(value); 
        }else if(counter2 == 2){
          type = std::stoi(value);
        }
      counter2++; 
      }
      Cone cone = Cone(x,y,type,counter1);
      m_map.push_back(cone);
      counter1++;
      std::cout << "Line Finished" << std::endl;
    }
    mapStream.close();
  }
}

void Slam::loadPath(std::string pathFile){
  std::ifstream pathStream (pathFile);
  std::string line;
  if(pathStream.is_open()){
    while(getline(pathStream,line)){
      std::stringstream linestream(line);
      std::string value;
      int counter = 1;
      while(getline(linestream,value,',')){
        if(counter == 1){
          int type = std::stoi(value);
          m_typeList.push_back(type);          
        }
        if(counter == 2){
          int id = std::stoi(value);
          m_skidPadList.push_back(id);
        }
        counter++;
      }
    }
  }
  pathStream.close();
}

void Slam::initializeModule(){
  //local Gps Vars
  double lastOdoX = 100000;
  double lastOdoY = 100000;
  int validGpsMeasurements = 0;
  bool gpsReadyState = false;

  //Local IMU vars
  bool imuReadyState = false;
  float lastVel = 100000;
  float lastHead = 100000;
  int validVelMeasurements = 0;
  int validHeadMeasurements = 0;
  while(!m_readyState){
    bool sleep = true;
    auto start = std::chrono::system_clock::now();

    while(sleep)
    {
      auto now = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

      if(elapsed.count() > 50*1000){
        //std::cout << "Timed out" << std::endl;
        sleep = false;
      }
    }
    //GPS

    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    if(!gpsReadyState){

      if( std::fabs(m_odometryData(0) - lastOdoX) > 0.001 && std::fabs(m_odometryData(1) - lastOdoY) > 0.001){
        if(m_odometryData(0) < 200 && m_odometryData(1) < 200){
          lastOdoX = m_odometryData(0);
          lastOdoY = m_odometryData(1);
          validGpsMeasurements++;
        }
      }else{}

      if(validGpsMeasurements > 2){
        gpsReadyState = true;
        std::cout << "GPS Ready .." << std::endl;
      }
    }//GPS end  
    //IMU
    if(!imuReadyState){

      std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
      if(std::fabs(m_groundSpeed - lastVel) > 0.001){ 
        lastVel = m_groundSpeed;  
        validVelMeasurements++;
      }
      if(std::fabs(m_odometryData(2) - lastHead) > 0.001){
        lastHead = static_cast<float>(m_odometryData(2));  
        validHeadMeasurements++;
      }
      if(validVelMeasurements > 3 && validHeadMeasurements > 3){
        imuReadyState = true;
        std::cout << "IMU Ready .." << std::endl;
      }
    }

    if(gpsReadyState && imuReadyState){
      m_headingOffset = m_sendPose(2)-lastHead;
      double newX = lastOdoX*cos(m_headingOffset)-lastOdoY*sin(m_headingOffset);
      double newY = lastOdoX*sin(m_headingOffset)+lastOdoY*cos(m_headingOffset);
      m_xOffset = m_sendPose(0)-newX;
      m_yOffset = m_sendPose(1)-newY;
      m_readyState = true;
      std::cout << "Slam ready check done !" << std::endl;  
    }
  }//While
}

bool Slam::checkOffset(){
  m_sendPose(2) = (m_sendPose(2) > PI)?(m_sendPose(2)-2*PI):(m_sendPose(2));
  m_sendPose(2) = (m_sendPose(2) < -PI)?(m_sendPose(2)+2*PI):(m_sendPose(2));
  double headingOffset = m_headingOffset + m_sendPose(2) - m_odometryData(2);
  double xOffset = m_xOffset + m_sendPose(0)-m_odometryData(0);
  double yOffset = m_yOffset + m_sendPose(1)-m_odometryData(1);
  bool goodError = (fabs(xOffset-m_xOffset)<m_offsetLimit && fabs(yOffset-m_yOffset)<m_offsetLimit && fabs(headingOffset-m_headingOffset)<m_offsetHeadingLimit);
  std::cout << "xOffset: " << fabs(xOffset-m_xOffset) << " yOffset " << fabs(yOffset-m_yOffset) << " headingOffset " << fabs(headingOffset-m_headingOffset) << std::endl;
  if(goodError){
    //m_headingError = headingOffset-m_headingOffset;
    m_xOffset = xOffset;
    m_yOffset = yOffset;
  }
  return goodError;
}
void Slam::setStateMachineStatus(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  auto machineStatus = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
  int state = machineStatus.state();
  if(state == 2){
    m_readyStateMachine = true;
  }
  
}
bool Slam::getModuleState(){

  return m_readyState;

}
std::vector<Eigen::Vector3d> Slam::drawPoses(){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  return m_poses;
}

std::vector<Cone> Slam::drawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_map;
}
std::vector<Cone> Slam::drawRawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_coneList;
}
std::vector<Cone> Slam::drawLocalOptimizedCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_essentialMap;
}
Eigen::Vector3d Slam::drawCurrentPose(){
  std::lock_guard<std::mutex> lock(m_sendMutex);
  return m_sendPose;
}

std::vector<std::vector<int>> Slam::drawGraph(){
  std::lock_guard<std::mutex> lock1(m_mapMutex);
  std::lock_guard<std::mutex> lock2(m_sensorMutex);
  return m_connectivityGraph;
 
}
void Slam::writeToPoseAndMapFile()
{
  std::string filepathMap;
  filepathMap = "./map.txt";
	
		std::ofstream f;
    	f.open(filepathMap.c_str());
		for(uint32_t i = 0; i<m_map.size(); i++){

				f << std::setprecision(9) << m_map[i].getX() << "\t" << m_map[i].getY() << std::endl;
		}
		f.close();
		std::cout << "map with " << m_map.size() << " points saved" << std::endl;


    std::string filepathPose;
    filepathPose = "./pose.txt";
		std::ofstream p;
    p.open(filepathPose.c_str());
		for(uint32_t i = 0; i<m_poses.size(); i++){

				p << std::setprecision(9) << m_poses[i](0) << "\t" << m_poses[i](1) << "\t" << m_poses[i](2) << std::endl;
		}
		p.close();

}
void Slam::tearDown()
{
}
Slam::~Slam()
{

}
