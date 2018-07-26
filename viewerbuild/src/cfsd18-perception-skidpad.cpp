/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "g2o/core/sparse_optimizer.h"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "slam.hpp"
#include "cone.hpp"
#include "drawer.hpp"
#include "viewer.hpp"
#include "collector.hpp"
#include <Eigen/Dense>

#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>
typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

void sendCones(std::vector<ConePackage> cones,cluon::OD4Session &od4, uint32_t const senderStamp){
  for(uint32_t i = 0; i<cones.size(); i++){
    std::chrono::system_clock::time_point tp;
    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    ConePackage cone = cones[i];
    opendlv::logic::perception::ObjectDirection directionMsg = std::get<0>(cone);
    od4.send(directionMsg,sampleTime,senderStamp);
    opendlv::logic::perception::ObjectDistance distanceMsg = std::get<1>(cone);
    od4.send(distanceMsg,sampleTime,senderStamp);
    opendlv::logic::perception::ObjectType typeMsg = std::get<2>(cone);
    od4.send(typeMsg,sampleTime,senderStamp);
  }
}


int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<10) {
    std::cerr << argv[0] << " is a skidpad for the CFSD18 project." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=120 --detectConeId=118 --estimationId=114 --gatheringTimeMs=10 --sameConeThreshold=1.2 --refLatitude=48.123141 --refLongitude=12.34534 --timeBetweenKeyframes=0.5 --coneMappingThreshold=50 --conesPerPacket=20" <<  std::endl;
    retCode = 1;
  } else {
    //uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    g2o::SparseOptimizer optimizer;
    (void)VERBOSE;
    // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
    cluon::data::Envelope data;
    //std::shared_ptr<Slam> slammer = std::shared_ptr<Slam>(new Slam(10));
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    Slam slam(commandlineArguments,od4);
    int gatheringTimeMs = (commandlineArguments.count("gatheringTimeMs")>0)?(std::stoi(commandlineArguments["gatheringTimeMs"])):(10);
    Collector collector(slam,gatheringTimeMs,2);
    Collector collectorCv(slam,gatheringTimeMs,3);
    Drawer drawer(commandlineArguments,slam);
    Viewer viewer(commandlineArguments,drawer);
    std::thread viewThread (&Viewer::Run,viewer); 

    uint32_t detectconeCvStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeCvId"]));
    uint32_t detectconeStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeId"]));
    uint32_t estimationStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["estimationId"]));
    uint32_t slamStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["id"])); 
    uint32_t stateMachineStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["stateMachineId"]));

    auto poseEnvelope{[&slammer = slam,senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          slammer.nextPose(envelope);
        }
      } 
    };

    auto coneEnvelope{[&slammer = slam, cvStamp = detectconeCvStamp, attentionStamp = detectconeStamp,&collector,&collectorCv](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == attentionStamp){
          collector.CollectCones(envelope);
        }else if(envelope.senderStamp() == cvStamp){
          collectorCv.CollectCones(envelope);
        }
      }
    };

    auto splitPoseEnvelope{[&slammer = slam, senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          slammer.nextSplitPose(envelope);
        }
      }
    };
    auto yawRateEnvelope{[&slammer = slam, senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          slammer.nextYawRate(envelope);
        }
      }
    };
    auto groundSpeedEnvelope{[&slammer = slam, senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          slammer.nextGroundSpeed(envelope);
        }
      }
    };
    auto stateMachineStatusEnvelope{[&slammer = slam, senderStamp = stateMachineStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){

          slammer.setStateMachineStatus(envelope);
        }
      }
    };
    od4.dataTrigger(opendlv::proxy::GeodeticWgs84Reading::ID(),splitPoseEnvelope);
    od4.dataTrigger(opendlv::proxy::GeodeticHeadingReading::ID(),splitPoseEnvelope);
    od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(),poseEnvelope);
    od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(),yawRateEnvelope);
    od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),groundSpeedEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),coneEnvelope);

    od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(),stateMachineStatusEnvelope);

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    bool readyState = false;
    while (od4.isRunning()) {
     
      if(readyState){
        opendlv::system::SignalStatusMessage ssm;
        ssm.code(1);
        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(ssm, sampleTime ,slamStamp);
      }else{
        slam.initializeModule();
        readyState = slam.getModuleState();
      }
      std::this_thread::sleep_for(0.1s);
      std::chrono::system_clock::time_point tp;
    }
  }
  return retCode;
}


