




#include "viewer.hpp"
#include <pangolin/pangolin.h>


Viewer::Viewer(std::map<std::string,std::string> commandlineArgs, Drawer &drawer)
: m_drawer(drawer){
    std::cout << commandlineArgs.count("cid") << std::endl;    
}

void Viewer::Run(){
    pangolin::CreateWindowAndBind("SLAM Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuShowRawCones("menu.ShowRawCones",true,true);
    pangolin::Var<bool> menuShowOptimizedCones("menu.ShowOptimizedCones",true,true);
    pangolin::Var<bool> menuShowPoses("menu.ShowPoses",true,true);
    pangolin::Var<bool> menuShowEssentialCones("menu.ShowEssentialCones",true,true);
    pangolin::Var<bool> menuShowCurrentPose("menu.ShowCurrentPose",true,true);
    pangolin::Var<bool> menuShowGraph("menu.ShowGraph",true,true);
    pangolin::Var<bool> menuExit("menu.Exit",false,false);

    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,2000,2000,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-10,10, 0,0,0,0.0,1.0, 0.0)
                );
    
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(1){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //m_drawer.drawCurrentCar(Twc);
        if(menuShowRawCones)
            m_drawer.drawRawCones();
        if(menuShowOptimizedCones)
            m_drawer.drawOptimizedCones();
        if(menuShowEssentialCones)
            m_drawer.drawEssentialCones();    
        if(menuShowPoses)
            m_drawer.drawPoses();
        if(menuShowCurrentPose)
            m_drawer.drawCurrentPose();
        if(menuShowGraph)
            m_drawer.drawGraph();
        pangolin::FinishFrame();

        if(menuExit)
            break;
    }

}