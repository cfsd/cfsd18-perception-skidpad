#include "drawer.hpp"


Drawer::Drawer(std::map<std::string,std::string> commandlineArgs, Slam &a_slam):
slam(a_slam)
{
    std::cout << commandlineArgs.count("cid") << std::endl;
}


void Drawer::drawRawCones(){
    m_cones = slam.drawRawCones();
    uint32_t nPoints = static_cast<unsigned int>(m_cones.size());
    if(nPoints == 0){
        return;
    }    
    glBegin(GL_POINTS);
    for(uint32_t i = 0; i<nPoints; i++){
        float x = static_cast<float>(m_cones[i].getMeanX()/5);
        float y = static_cast<float>(m_cones[i].getMeanY()/5);
        float z = 0.0f;
        if(m_cones[i].getType() == 2){
            glColor3f(1.0,1.0,0.0);//yellow
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 1){
            glColor3f(0.0,0.0,1.0);//blue
            glPointSize(10);    
        }
        else if(m_cones[i].getType() == 3){
            glColor3f(1.0,0.5,0.0);//little orange
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 4){
            glColor3f(1.0,0.5,0.0);//big orange
            glPointSize(15);
        }
        else{
            glColor3f(0.5,0.5,0.5);
            glPointSize(10);
        }
        glVertex3f(x,y,z);
    }

    glEnd();
}
void Drawer::drawOptimizedCones(){
    m_cones = slam.drawCones();
    uint32_t nPoints = static_cast<unsigned int>(m_cones.size());
    if(nPoints == 0){
        return;
    }    
    glBegin(GL_POINTS);
    for(uint32_t i = 0; i<nPoints; i++){
        float x = static_cast<float>(m_cones[i].getX()/5);
        float y = static_cast<float>(m_cones[i].getY()/5);
        float z = 0.0f;
        if(m_cones[i].getType() == 2){
            glColor3f(1.0,1.0,0.0);//yellow
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 1){
            glColor3f(0.0,0.0,1.0);//blue
            glPointSize(10);    
        }
        else if(m_cones[i].getType() == 3){
            glColor3f(1.0,0.5,0.0);//little orange
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 4){
            glColor3f(1.0,0.5,0.0);//big orange
            glPointSize(15);
        }
        else if(m_cones[i].getType() == 0){
            glColor3f(1.0f,0.0f,1.0f);
            glPointSize(10);
        }
        glVertex3f(x,y,z);
    }

    glEnd();
}
void Drawer::drawEssentialCones(){

    m_cones = slam.drawLocalOptimizedCones();
    uint32_t nPoints = static_cast<unsigned int>(m_cones.size());
    if(nPoints == 0){
        return;
    }    
    glBegin(GL_POINTS);
    for(uint32_t i = 0; i<nPoints; i++){
        float x = static_cast<float>(m_cones[i].getX()/5);
        float y = static_cast<float>(m_cones[i].getY()/5);
        float z = 0.0f;
        if(m_cones[i].getType() == 2){
            glColor3f(1.0,1.0,0.0);//yellow
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 1){
            glColor3f(0.0,0.0,1.0);//blue
            glPointSize(10);    
        }
        else if(m_cones[i].getType() == 3){
            glColor3f(1.0,0.5,0.0);//little orange
            glPointSize(10);
        }
        else if(m_cones[i].getType() == 4){
            glColor3f(1.0,0.5,0.0);//big orange
            glPointSize(15);
        }
        else{
            glColor3f(0.5f,1.0f,0.0f);
            glPointSize(10);
        }
        glVertex3f(x,y,z);
    }

    glEnd();

}
void Drawer::drawPoses(){
    m_poses = slam.drawPoses();
    uint32_t nPoints = static_cast<unsigned int>(m_poses.size());
    if(nPoints == 0){
        return;
    }    
    glPointSize(7);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(uint32_t i = 0; i<nPoints; i++){
        float x = static_cast<float>(m_poses[i](0)/5);
        float y = static_cast<float>(m_poses[i](1)/5);
        float z = 0.0f;
        glVertex3f(x,y,z);
    }
    glEnd();
}


void Drawer::drawCurrentPose(){
    m_pose = slam.drawCurrentPose();
    uint32_t nPoints = static_cast<unsigned int>(m_pose.rows());
    if(nPoints == 0){
        return;
    }    
    glPointSize(12);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(uint32_t i = 0; i<nPoints; i++){
        float x = static_cast<float>(m_pose(0)/5);
        float y = static_cast<float>(m_pose(1)/5);
        float z = 0.0f;
        glVertex3f(x,y,z);
        glColor3f(0.0,1.0,0.0);
        glVertex3f(x+static_cast<float>(2*cos(m_pose(2))/5),y+static_cast<float>(2*sin(m_pose(2))/5),z);
    }
    glEnd();
}

void Drawer::drawGraph(){
    m_poses = slam.drawPoses();
    uint32_t nPoints = static_cast<unsigned int>(m_poses.size());
    if(nPoints == 0){
        return;
    }
    m_cones = slam.drawLocalOptimizedCones();
    nPoints = static_cast<unsigned int>(m_cones.size());
    if(nPoints == 0){
        return;
    }

    glLineWidth(1);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

    for(uint32_t i = 0; i < m_cones.size(); i++ ){
        std::vector<int> connections = m_cones[i].getConnectedPoses();
        for(uint32_t j = 0; j < connections.size(); j++){
            glVertex3f(static_cast<float>(m_poses[connections[j]-1000](0)/5),static_cast<float>(m_poses[connections[j]-1000](1)/5),0.0f);
            glVertex3f(static_cast<float>(m_cones[i].getOptX()/5),static_cast<float>(m_cones[i].getOptY()/5),0.0f);
        }
    }
    glEnd();

}