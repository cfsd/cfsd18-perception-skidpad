


#ifndef DRAWER_HPP
#define DRAWER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "cone.hpp"
#include "slam.hpp"




class Drawer{
    public:
        Drawer(std::map<std::string,std::string> commandlineArgs, Slam &slam);
        void drawPoses();
        void drawRawCones();
        void drawOptimizedCones();
        void drawEssentialCones();
        void drawCurrentPose();
        void drawGraph();

    private:
        Slam& slam;
        std::vector<Cone> m_cones = {};
        std::vector<Eigen::Vector3d> m_poses = {};
        Eigen::Vector3d m_pose = {};

};
#endif